#include "mapwidget.h"
#include "mapviewer.h"

#include <QStatusBar>

#include <boost/utility.hpp>

using namespace std;

MapWidget::MapWidget (MapViewer *mapviewer,QWidget *parent) : QGLWidget(parent), mv(mapviewer) {

  // get map range in cartesian coordinates
  Objects::tRange map_range = mv->objects->getMapRange();
  Coordinates::tCartesian cart_min = mv->coord->latLonToCartesian(map_range.geo_min);
  Coordinates::tCartesian cart_max = mv->coord->latLonToCartesian(map_range.geo_max);
  x_min = cart_min.x; y_min = cart_min.y;
  x_max = cart_max.x; y_max = cart_max.y;
  delta = min(x_max-x_min,y_max-y_min);

  // init map scale and offset
  scale = 1;
  x_pos = 0;
  y_pos = 0;

  // enable mouse tracking for continuous updates
  setMouseTracking(true);
  
  // init marker to disabled
  marker = 0;
}

MapWidget::~MapWidget () {
}

void MapWidget::cartesianToScreen (Coordinates::tCartesian cart,double &u,double &v) {
  u = scale_x*(scale*(cart.x-x_min)/delta+x_pos);
  v = scale_y*(scale*(cart.y-y_min)/delta+y_pos);
}

void MapWidget::cartesianToUnit (Coordinates::tCartesian cart,double &u,double &v) {
  u = scale*(cart.x-x_min)/delta+x_pos;
  v = scale*(cart.y-y_min)/delta+y_pos;
}

Coordinates::tCartesian MapWidget::screenToCartesian (double u,double v) {
  Coordinates::tCartesian cart;
  cart.x = (u/scale_x-x_pos)*delta/scale+x_min;
  cart.y = (v/scale_y-y_pos)*delta/scale+y_min;
  return cart;
}

Coordinates::tGeographic MapWidget::screenToLatLon (double u,double v) {
  return mv->coord->cartesianToLatLon(screenToCartesian(u,v));
}

void MapWidget::plotEndPoint (Objects::tReference &ref1,Objects::tReference &ref2) {

  // get references to points
  Objects::tPoint &p1 = mv->objects->getPayload<Objects::tPoint>(ref1.ptr);
  Objects::tPoint &p2 = mv->objects->getPayload<Objects::tPoint>(ref2.ptr);

  // get cartesian coordinates
  double p1x,p1y,p2x,p2y;
  Coordinates::tCartesian cart1 = mv->coord->latLonToCartesian(p1.geo);
  Coordinates::tCartesian cart2 = mv->coord->latLonToCartesian(p2.geo);
  p1x = cart1.x; p1y = cart1.y;
  p2x = cart2.x; p2y = cart2.y;

  // get vector in direction of polyline
  double vx = p2x-p1x;
  double vy = p2y-p1y;

  // compute vector norm
  double norm = sqrt(vx*vx+vy*vy);
  if (norm<1e-5) return;

  // normalize vector in direction of polyline
  vx /= 2.0*norm;
  vy /= 2.0*norm;

  // compute normalized screen coordinates
  double p1u,p1v,p2u,p2v;
  cartesianToScreen(Coordinates::tCartesian(p1x+vx+vy,p1y+vy-vx),p1u,p1v);
  cartesianToScreen(Coordinates::tCartesian(p1x+vx-vy,p1y+vy+vx),p2u,p2v);

  // draw line
  glBegin(GL_LINES);
  glVertex3f(p1u*2.0-1.0,p1v*2.0-1.0,0);
  glVertex3f(p2u*2.0-1.0,p2v*2.0-1.0,0);
  glEnd();
}

void MapWidget::plotPolyline (Objects::tObject *object) {

  // get polyline
  Objects::tPolyline &polyline = mv->objects->getPayload<Objects::tPolyline>(object);
  vector<Objects::tReference> &points = polyline.points;

  // start polyline
  glBegin(GL_LINE_STRIP);

  // plot polyline
  for (vector<Objects::tReference>::iterator it=points.begin(); it!=points.end(); it++) {
    Objects::tPoint &point = mv->objects->getPayload<Objects::tPoint>(it->ptr);
    Coordinates::tCartesian cart = mv->coord->latLonToCartesian(point.geo);
    double u,v;
    cartesianToScreen(cart,u,v);
    glVertex3f(u*2.0-1.0,v*2.0-1.0,0);
  }

  // end polyline
  glEnd();

  // plot end points
  int n = points.size();
  if (n>1) {
    plotEndPoint(points[0],points[1]);
    plotEndPoint(points[n-1],points[n-2]);
  }
}

void MapWidget::plotPoint (Objects::tPoint &point) {

  // map central point to screen
  Coordinates::tCartesian cart = mv->coord->latLonToCartesian(point.geo);
  double u,v;
  cartesianToScreen(cart,u,v);

  // draw map central point
  glBegin(GL_POINTS);
  glVertex3f(u*2.0-1.0,v*2.0-1.0,0);
  glEnd();
}

void MapWidget::plotRange () {

  // compute corners in normalized screen coordinates
  double p1u,p1v,p2u,p2v,p3u,p3v,p4u,p4v;
  cartesianToScreen(Coordinates::tCartesian(x_min,y_min),p1u,p1v);
  cartesianToScreen(Coordinates::tCartesian(x_max,y_min),p2u,p2v);
  cartesianToScreen(Coordinates::tCartesian(x_max,y_max),p3u,p3v);
  cartesianToScreen(Coordinates::tCartesian(x_min,y_max),p4u,p4v);

  // start dotted lines
  glLineStipple(1, 0xAAAA);
  glEnable(GL_LINE_STIPPLE);

  // start closed polyline
  glBegin(GL_LINE_LOOP);

  // draw rectangle
  glVertex3f(p1u*2.0-1.0,p1v*2.0-1.0,0);
  glVertex3f(p2u*2.0-1.0,p2v*2.0-1.0,0);
  glVertex3f(p3u*2.0-1.0,p3v*2.0-1.0,0);
  glVertex3f(p4u*2.0-1.0,p4v*2.0-1.0,0);

  // end polyline
  glEnd();

  // end dotted lines
  glDisable(GL_LINE_STIPPLE);
}

void MapWidget::paintGL () {

  // clear screen
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  // get screen range in geographic coordinates
  Objects::tRange range;
  range.geo_min = screenToLatLon(0,0);
  range.geo_max = screenToLatLon(1,1);

  // retrieve all objects which are currently visible
  Objects::objects_t objects = mv->objects->getObjectsInRange(range);

  // draw streets
  glColor3f(1,0,0);
  for (Objects::objects_t::iterator it=objects.begin(); it!=objects.end(); it++)
    if (mv->objects->isType<Objects::tPolyline>(*it,"street"))
      plotPolyline(*it);

  // draw buildings
  glColor3f(0,1,0);
  for (Objects::objects_t::iterator it=objects.begin(); it!=objects.end(); it++)
    if (mv->objects->isType<Objects::tPolyline>(*it,"building"))
      plotPolyline(*it);
      
  // draw location marker
  if (marker) {
    glPointSize(5);
    glColor3f(1,1,0);
    plotPoint(*marker);
  }

  // draw rectangular range of the map
  if (0) {
    glColor3f(0.8,0.8,0.8);
    plotRange();
  }
}

void MapWidget::resizeGL (int w, int h) {

  // compute viewport scalings
  scale_x = 1;
  scale_y = 1;
  if (w<h) scale_y = (double)w/max((double)h,1.0);
  else     scale_x = (double)h/max((double)w,1.0);

  // set viewport
  glViewport(0,0,w,h);
  paintGL();
  updateStatusBar();
}

void MapWidget::mousePressEvent (QMouseEvent* event) {

  // set mouse position
  x_mouse = event->posF().x();
  y_mouse = event->posF().y();
  
  // on right click, retrieve all polylines which are connected to a point
  if (event->buttons()==Qt::RightButton) {
  
    Coordinates::tCartesian  cart = getCurrentLocationCartesian();
    Coordinates::tGeographic geo  = mv->coord->cartesianToLatLon(cart);
    Objects::tObject* object      = mv->objects->getClosestPoint (geo);

    // plot infos    
    if (object) {
      cout << endl;
      cout << object->type << " (sha: " << object->sha;
      Objects::tPoint &point = mv->objects->getPayload<Objects::tPoint>(object);
      marker = &point;
      cout << ", lat: " << point.geo.lat << ", lon: " << point.geo.lon << ", alt: " << point.alt << ")" << endl;    
      for (Objects::objects_t::iterator it=object->use.begin(); it!=object->use.end(); it++) {
        if (mv->objects->isType<Objects::tPolyline>(*it)) {
          cout << ">> type: '" << (*it)->type << "'";
          if ((*it)->kind.size()>0)
            cout << ", kind: '" << (*it)->kind << "'";
          cout << " (sha: " << (*it)->sha << ")" << endl;
          if ((*it)->tag.size()>0) {
            cout << "   ";
            for (Objects::tags_t::iterator it_tag=(*it)->tag.begin(); it_tag!=(*it)->tag.end(); it_tag++) {
              cout << it_tag->first << ": '" << it_tag->second << "'";
              if (boost::next(it_tag)!=(*it)->tag.end())
                cout << ", ";
            }
            cout << endl;
          }
        }
      }
    } else {
      marker = 0;
    }
  }
  
  // refresh
  updateGL();
  updateStatusBar();
}

Coordinates::tCartesian MapWidget::getCurrentLocationCartesian () {
  double u = x_mouse/(double)size().width();
  double v = 1.0-y_mouse/(double)size().height();
  Coordinates::tCartesian cart = screenToCartesian(u,v);
  return cart;
}

void MapWidget::updateStatusBar () {

  // set status message
  Coordinates::tCartesian  cart  = getCurrentLocationCartesian();
  Coordinates::tGeographic geo   = mv->coord->cartesianToLatLon(cart);
  Coordinates::tCartesian  dcart = screenToCartesian(1,1)-screenToCartesian(0,0);
  char msg[2048];
  sprintf(msg,"x: %1.2f m, y: %1.2f m, lat: %1.5f, lon: %1.5f, scale: %1.2f, w: %1.2f m, h: %1.2f m",cart.x,cart.y,geo.lat,geo.lon,scale,dcart.x,dcart.y);
  mv->statusBar()->showMessage(msg);
}

void MapWidget::mouseMoveEvent (QMouseEvent* event){

  // translate map
  if (event->buttons()==Qt::LeftButton) {
    double s  = max(min(size().width(),size().height()),1);
    double dx = ((double)event->posF().x()-x_mouse)/s;
    double dy = ((double)event->posF().y()-y_mouse)/s;
    x_pos += dx;
    y_pos -= dy;
  }

  // update current mouse position
  x_mouse = event->posF().x();
  y_mouse = event->posF().y();

  // refresh
  updateGL();
  updateStatusBar();
}

void MapWidget::wheelEvent (QWheelEvent* event){

  // mouse position
  x_mouse = event->pos().x();
  y_mouse = event->pos().y();

  // widget size
  double w = max(size().width(),1);
  double h = max(size().height(),1);

  // compute new scaling
  double delta = (double)event->delta();
  double scale_old = scale;
  if (delta<0 && scale>0.001) scale /= 1.2;
  if (delta>0 && scale<100.0) scale *= 1.2;

  // adjust viewport position (centered on mouse)
  double x_screen = (x_mouse/(scale_x*w)-x_pos)/scale_old;
  double y_screen = (1.0-y_mouse/(scale_y*h)-y_pos)/scale_old;
  x_pos -= (scale-scale_old)*x_screen;
  y_pos -= (scale-scale_old)*y_screen;

  // refresh
  updateGL();
  updateStatusBar();
}

