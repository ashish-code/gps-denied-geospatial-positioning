#pragma once

#include <QMainWindow>

#include "mapwidget.h"
#include "objects.h"
#include "coordinates.h"

namespace Ui {
  class MapViewer;
}

class MapViewer : public QMainWindow {

  Q_OBJECT

public:

  MapViewer (Objects *objects,QWidget *parent = 0);
  ~MapViewer ();

public:

  Objects        *objects;
  Coordinates    *coord;
  Objects::tPoint origin;

private slots:

  void on_actionCapture_triggered();

private:

  void plotPolyline (Objects::tObject *object,QPainter &painter,float width,float height);

  Ui::MapViewer *ui;
  MapWidget     *mapwidget;

};
