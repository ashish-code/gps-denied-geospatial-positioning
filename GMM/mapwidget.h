#pragma once

#include <QGLWidget>
#include <QMouseEvent>
#include <QWheelEvent>

#include "objects.h"

class MapViewer;

class MapWidget : public QGLWidget {

  Q_OBJECT

public:

  MapWidget (MapViewer *mapviewer,QWidget *parent = 0);
  ~MapWidget ();

public:

  // convert from cartesian coordinates to normalized screen coordinates (center: bottom left!)
  void cartesianToScreen (Coordinates::tCartesian cart,double &u,double &v);

  // convert from cartesian coordinates to unit coordinates (0 to 1, aspect ratio 1:1)
  void cartesianToUnit (Coordinates::tCartesian cart,double &u,double &v);

  // convert from normalized screen coordinates (center: bottom left!) to cartesian coordinates
  Coordinates::tCartesian screenToCartesian (double u,double v);

  // convert from normalized screen coordinates to geographic coordinates
  Coordinates::tGeographic screenToLatLon (double u,double v);

protected:

  // plot viewport
  void paintGL ();

  // resize widget
  void resizeGL (int w, int h);

  // mouse pressed
  void mousePressEvent (QMouseEvent* event);

  // mouse moved
  void mouseMoveEvent (QMouseEvent *event);

  // mouse wheel changed
  void wheelEvent (QWheelEvent* event);

private:

  // plot polyline end marker
  void plotEndPoint (Objects::tReference &ref1,Objects::tReference &ref2);

  // plots a single polyline onto the viewport
  void plotPolyline (Objects::tObject *object);

  // plots point where the map is centered
  void plotPoint (Objects::tPoint &point);

  // plots the rectangular range of the map
  void plotRange ();

  // retrieve cartesian coordinates from current mouse pointer location  
  Coordinates::tCartesian getCurrentLocationCartesian ();

  // update status bar information
  void updateStatusBar ();

private:

  // status bar
  MapViewer *mv;

  // map extend in cartesian coordinates
  double x_min,x_max;
  double y_min,y_max;
  double delta; // min delta

  // map scale and offset in normalized coordinates
  double scale;
  double x_pos;
  double y_pos;

  // mouse position
  double x_mouse;
  double y_mouse;

  // viewport scaling
  double scale_x;
  double scale_y;
  
  // location marker
  Objects::tPoint *marker ;
};

