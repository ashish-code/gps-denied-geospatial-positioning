#include "coordinates.h"
#include "math.h"

using namespace std;

// average earth radius at the equator
#define EARTH_RADIUS_EQUA 6378137.0

Coordinates::Coordinates(tGeographic geo) {

  // compute scale
  scale = cos(geo.lat * M_PI / 180.0);

  // compute cartesian coordinate system origin
  origin = latLonToMercator(geo);
}

Coordinates::tCartesian Coordinates::latLonToCartesian (tGeographic geo) {
  return latLonToMercator(geo) - origin;
}

Coordinates::tGeographic Coordinates::cartesianToLatLon (tCartesian cart) {
  return mercatorToLatLon(cart + origin);
}

Coordinates::tCartesian Coordinates::latLonToMercator (tGeographic geo) {
  tCartesian cart;
  cart.x = scale * geo.lon * M_PI * EARTH_RADIUS_EQUA / 180.0;
  cart.y = scale * EARTH_RADIUS_EQUA * log( tan((90.0+geo.lat) * M_PI / 360.0) );
  return cart;
}

Coordinates::tGeographic Coordinates::mercatorToLatLon (tCartesian cart) {
  tGeographic geo;
  geo.lon = cart.x * 180.0 / (M_PI * EARTH_RADIUS_EQUA * scale);
  geo.lat = 360.0 * atan( exp(cart.y/(EARTH_RADIUS_EQUA * scale)) ) / M_PI - 90.0;
  return geo;
}

double Coordinates::latLonToDistance (tGeographic geo0,tGeographic geo1) {
  tCartesian dcart = latLonToCartesian(geo1) - latLonToCartesian(geo0);
  return sqrt(dcart.x*dcart.x+dcart.y*dcart.y);
}

