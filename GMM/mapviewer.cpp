#include "mapviewer.h"
#include "ui_mapviewer.h"

#include <Qt/qsvggenerator.h>
#include <Qt/qprinter.h>
#include <Qt/qfiledialog.h>

#include "utils.h"

using namespace std;

MapViewer::MapViewer(Objects *objects,QWidget *parent) : QMainWindow(parent), objects(objects), ui(new Ui::MapViewer) {

  // instanciate coordinate transformations using first point coordinates
  coord = 0;
  for (Objects::objects_t::iterator it=objects->begin(); it!=objects->end(); it++) {
    if (objects->isType<Objects::tPoint>(*it)) {
      origin = objects->getPayload<Objects::tPoint>(*it);
      coord = new Coordinates(origin.geo);
      break;
    }
  }

  // if no point has been found
  if (!coord) {
    cout << "ERROR: Couldn't instanciate map viewer as no points have been found." << endl;
    exit(0);
  }

  // setup user interface
  ui->setupUi(this);
  mapwidget = new MapWidget(this);
  ui->horizontalLayout->addWidget(mapwidget);
}

MapViewer::~MapViewer() {
  delete coord;
  delete mapwidget;
  delete ui;
}

void MapViewer::plotPolyline (Objects::tObject *object,QPainter &painter,float width,float height) {

  // get polyline
  Objects::tPolyline &polyline = objects->getPayload<Objects::tPolyline>(object);
  vector<Objects::tReference> &points = polyline.points;

  // allocate point array
  QPointF *qpoints = new QPointF[points.size()];

  // set points of polyline
  int k = 0;
  for (vector<Objects::tReference>::iterator it=points.begin(); it!=points.end(); it++) {
    Objects::tPoint &point = objects->getPayload<Objects::tPoint>(it->ptr);
    Coordinates::tCartesian cart = coord->latLonToCartesian(point.geo);
    double u,v;
    mapwidget->cartesianToScreen(cart,u,v);
    qpoints[k++] = QPointF(width*u,height-height*v);
  }

  // draw polyline
  painter.drawPolyline(qpoints,points.size());

  // release memory
  delete qpoints;
}

void MapViewer::on_actionCapture_triggered () {

  // get viewport dimensions
  float width  = mapwidget->size().width();
  float height = mapwidget->size().height();

  // get screen range in geographic coordinates
  Objects::tRange range;
  range.geo_min = mapwidget->screenToLatLon(0,0);
  range.geo_max = mapwidget->screenToLatLon(1,1);

  // svg generator, printer and painter
  QSvgGenerator generator;
  QPrinter printer(QPrinter::HighResolution);
  QPainter painter;

  // get filename and extension
  QString file_name = QFileDialog::getSaveFileName(this,"Select Filename","","Graphic Files [*.pdf *.svg] (*.pdf *.svg)");
  string  file_ext  = utils::get_file_extension(file_name.toStdString());

  // attach svg generator to painter
  if (file_ext=="svg") {
    generator.setFileName(file_name);
    generator.setSize(QSize(width,height));
    generator.setViewBox(QRect(0,0,width,height));
    painter.begin(&generator);

  // attach pdf printer to painter
  } else {
    printer.setOutputFileName(file_name);
    printer.setPaperSize(QSizeF(width,height),QPrinter::DevicePixel);
    printer.setPageMargins(0,0,0,0,QPrinter::DevicePixel);
    painter.begin(&printer);
  }

  // retrieve all objects which are currently visible
  Objects::objects_t objects_in_range = objects->getObjectsInRange(range);

  // draw streets
  painter.setPen(QPen(Qt::red,0.5,Qt::SolidLine));
  for (Objects::objects_t::iterator it=objects_in_range.begin(); it!=objects_in_range.end(); it++)
    if (objects->isType<Objects::tPolyline>(*it,"street"))
      plotPolyline(*it,painter,width,height);

  // draw buildings
  painter.setPen(QPen(Qt::green,0.25,Qt::SolidLine));
  for (Objects::objects_t::iterator it=objects_in_range.begin(); it!=objects_in_range.end(); it++)
    if (objects->isType<Objects::tPolyline>(*it,"building"))
      plotPolyline(*it,painter,width,height);
      
  // print
  painter.end();
}

