#ifndef __OSM__COORDINATES_H
#define __OSM__COORDINATES_H

class Coordinates {

public:

  // point in geographic and cartesian/mercator coordinates
  struct tGeographic { // geographic coordinates
    double lat,lon;
    tGeographic () {}
    tGeographic (double lat, double lon) : lat(lat), lon(lon) {}
    tGeographic operator+ (tGeographic geo) { geo.lat = lat+geo.lat; geo.lon = lon+geo.lon; return geo; }
    tGeographic operator- (tGeographic geo) { geo.lat = lat-geo.lat; geo.lon = lon-geo.lon; return geo; }
    tGeographic operator* (double val)      { return tGeographic(lat*val,lon*val);                      }
    tGeographic operator/ (double val)      { return tGeographic(lat/val,lon/val);                      }
  };
  struct tCartesian { // cartesian/mercator coordinates
    double x,y;
    tCartesian () {}
    tCartesian (double x, double y) : x(x), y(y) {}
    tCartesian operator+ (tCartesian cart) { cart.x = x+cart.x; cart.y = y+cart.y; return cart; }
    tCartesian operator- (tCartesian cart) { cart.x = x-cart.x; cart.y = y-cart.y; return cart; }
    tCartesian operator* (double val)      { return tCartesian(x*val,x*val);                }
    tCartesian operator/ (double val)      { return tCartesian(x/val,x/val);                }
  };

  // constructor sets scale from initial position
  Coordinates (tGeographic geo);

  // converts lat/lon to 0-normalized mercator (=cartesian)
  tCartesian latLonToCartesian (tGeographic geo);

  // converts 0-normalized mercator (=cartesian) to lat/lon
  tGeographic cartesianToLatLon (tCartesian cart);

  // convert 2 points in lat/lon coordinates to metric distance
  double latLonToDistance (tGeographic geo0, tGeographic geo1);

private:

  // converts lat/lon to mercator
  tCartesian latLonToMercator (tGeographic geo);

  // converts mercator to lat/lon
  tGeographic mercatorToLatLon (tCartesian cart);

  // fixed global scale and cartesian coordinate origin
  double     scale;
  tCartesian origin;

};

#endif

