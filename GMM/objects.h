#ifndef __OSM__OBJECTS_H
#define __OSM__OBJECTS_H

#include <string>
#include <vector>
#include <set>
#include <map>

#include <boost/variant.hpp>
#include <boost/unordered_map.hpp>

#include <boost/serialization/nvp.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/set.hpp>
#include <boost/serialization/map.hpp>
#include <boost/serialization/variant.hpp>

#include "coordinates.h"
#include "utils.h"

class Objects {

///////////////////////////////////////////////////////////////////////////////
// object definitions
public:

  // forward declarations
  struct tObject;
  typedef std::set<tObject*> objects_t;

  // lat/lon range
  struct tRange {
    Coordinates::tGeographic geo_min; // lat_min, lon_min
    Coordinates::tGeographic geo_max; // lat_max, lon_max
  };

  // object size
  struct tSize {
    double length,width,height;
    tSize() {}
    tSize(double length, double width, double height) : length(length), width(width), height(height) {}
    template<typename A> void serialize(A &ar, const unsigned int){
      ar & BOOST_SERIALIZATION_NVP(length)
         & BOOST_SERIALIZATION_NVP(width)
         & BOOST_SERIALIZATION_NVP(height);
    }
  };

  // object orientation in euler angles
  struct tOrientation {
    double roll,pitch,yaw;
    tOrientation() {}
    tOrientation(double roll, double pitch, double yaw) : roll(roll), pitch(pitch), yaw(yaw) {}
    template<typename A> void serialize(A &ar, const unsigned int){
      ar & BOOST_SERIALIZATION_NVP(roll)
         & BOOST_SERIALIZATION_NVP(pitch)
         & BOOST_SERIALIZATION_NVP(yaw);
    }
  };

  // reference to other object
  struct tReference {
    std::string  sha; // sha key of referenced object
    tObject     *ptr; // pointer to referenced object
    tReference () { ptr = 0; }
    tReference (std::string sha) : sha(sha) { ptr = 0; }
    template<typename A> void serialize (A &ar, const unsigned int){
      ar & BOOST_SERIALIZATION_NVP(sha);
    }
    bool operator==(const tReference &b) {
      return this->sha == b.sha && this->ptr == b.ptr;
    }
  };

  // point in geographic coordinates
  struct tPoint {
    Coordinates::tGeographic geo;
    double alt;
    tPoint() {}
    tPoint(Coordinates::tGeographic geo, double alt) : geo(geo), alt(alt) {}
    template<typename A> void serialize(A &ar, const unsigned int){
      ar & BOOST_SERIALIZATION_NVP(geo.lat)
         & BOOST_SERIALIZATION_NVP(geo.lon)
         & BOOST_SERIALIZATION_NVP(alt);
    }
  };

  // polyline object
  struct tPolyline {
    std::vector<tReference> points;
    tPolyline() {}
    template<typename A> void serialize(A &ar, const unsigned int){
      ar & BOOST_SERIALIZATION_NVP(points);
    }
  };

  // box object
  struct tBox {
    tReference   pos;
    tOrientation ori;
    tSize        size;
    tBox() {}
    template<typename A> void serialize(A &ar, const unsigned int){
      ar & BOOST_SERIALIZATION_NVP(pos)
         & BOOST_SERIALIZATION_NVP(ori)
         & BOOST_SERIALIZATION_NVP(size);
    }
  };

  // relation between objects or relations
  struct tRelation {
    std::vector<tReference> objects;
    tRelation() {}
    template<typename A> void serialize(A &ar, const unsigned int){
      ar & BOOST_SERIALIZATION_NVP(objects);
    }
  };

  // object payload variant (may be any of the object types)
  typedef boost::variant<tPoint,tPolyline,tBox,tRelation> tPayload;

  // stores auxiliary tags which can be attached to an object
  typedef std::map<std::string,std::string> tags_t;

  // object => meta info (sha,type,usage) + data payload
  struct tObject {
    std::string sha;     // unique object sha key
    std::string type;    // object type (mandatory)
    std::string kind;    // object sub-type (not mandatory)
    tags_t      tag;     // auxiliary tags attached to this object
    objects_t   use;     // objects pointing to this object
    tPayload    payload; // payload this object is storing
    tObject () {}
    tObject (tPayload payload) : payload(payload) { sha = utils::rand_sha(); }
    template<typename A> void serialize(A &ar, const unsigned int){
      ar & BOOST_SERIALIZATION_NVP(sha)
         & BOOST_SERIALIZATION_NVP(type)
         & BOOST_SERIALIZATION_NVP(kind)
         & BOOST_SERIALIZATION_NVP(tag)
         & BOOST_SERIALIZATION_NVP(payload);
    }
    template<typename T>
    T &getPayload() { return boost::get<T>(payload); }
    template<typename T>
    bool isType() const { return payload.type()==typeid(T); }
    const std::string &getTag(const std::string &name) { return tag[name]; }
  };

  // serialization of all objects
  template<typename A> void serialize(A &ar, const unsigned int){
    ar & BOOST_SERIALIZATION_NVP(objects);
  }

///////////////////////////////////////////////////////////////////////////////
// public functions
public:

  // constructor, seeds random number generator for sha keys of new objects
  Objects (bool verbose=false);

  // deconstructor, frees memory occupied by objects
  virtual ~Objects ();

  // loads objects from binary, text or xml file (*.bin, *.txt, *.xml)
  bool loadFromFile (std::string filename);

  // stores objects to binary, text or xml file (*.bin, *.txt, *.xml)
  bool saveToFile (std::string filename);

  // add and tracks a new object, all dependencies must exist
  // note: all object additions must pass through this function
  void addObject (tObject *object);

  // delete object at iterator it
  // note: all object deletions must pass through this function
  void delObject (objects_t::iterator &it);
  
  Objects::tObject *getObject(const std::string &sha) { return map_objects[sha]; }

  // add/delete reference pointer to target object and add object to target object usages
  void modPointer (tReference &ref,Objects::tObject *object,bool add);

  // add/delete reference pointers and usage pointers of/to object
  void modPointers (tObject *object,bool add);

  // deletes all invalid objects (type="")
  int deleteInvalidObjects ();

  // deletes all points which are not referenced by any other object
  int deleteUnusedPoints ();

  // get number of objects currently stored
  int numberOfObjects ();

  // returns iterator to begin of objects
  objects_t::iterator begin () { return objects.begin(); }

  // returns iterator to end of objects
  objects_t::iterator end () { return objects.end(); }

  // returns list with objects of specified type T and object type
  template<class T> objects_t getObjectsOfType (std::string type="") {
    objects_t objects_type;
    for (objects_t::iterator it=begin(); it!=end(); it++)
      if (isType<T>(*it) && (type=="" || (*it)->type==type))
        objects_type.insert(*it);
    return objects_type;
  }

  // get payload of object
  template<class T> T& getPayload (tObject *object) {
    return boost::get<T>(object->payload);
  }

  // check if data type of object equals T
  template<class T> bool isType (tObject *object) {
    return object->payload.type()==typeid(T);
  }

  // check if data type of object equals T and additionally has certain type
  template<class T> bool isType (tObject *object,std::string type) {
    return object->payload.type()==typeid(T) && object->type==type;
  }

  // retrieve range of map (min and max lat/lon)
  tRange getMapRange ();

  // retrieve objects within range
  objects_t getObjectsInRange (tRange range);
  
  // retrieve closest point in cell (null if no points in cell available)
  tObject* getClosestPoint (Coordinates::tGeographic geo);

  // splits all polylines of specified type at points, which
  // are referenced by other objects (e.g., intersections)
  void splitPolylines (std::string type);

  // delete polyline of specified type and kind; if kind is
  // an empty string, this deletes all polylines of specified type
  void deletePolylines (std::string type,std::string kind="");
  
  // deletes all polylines of specified type which kind is not any of the given ones
  void keepPolylines (std::string type,std::vector<std::string> kind);

  // delete small segments of specified type and kind; if kind is
  // an empty string, this deletes all polylines of specified type
  // note: this does not delete any first, last or intersecting points
  void deleteSmallSegments (double min_dist,std::string type,std::string kind="");
  
///////////////////////////////////////////////////////////////////////////////
// private functions
private:

  // compute spatial hash from bin
  size_t spatialHash (double x,double y);
  
  // compute spatial hash from lat/lon coordinate
  size_t spatialHash (Coordinates::tGeographic geo);

  // add objects and its ancestors recursively to set_objects
  void addObjectsAndParentsToSet (tObject *object,objects_t &set_objects);

///////////////////////////////////////////////////////////////////////////////
// private members
private:

  // vector referencing all objects stored by this class
  objects_t objects;

  // spatial hash multimap (for quickly accessing objects at a specific location)
  typedef boost::unordered_multimap<size_t,tObject*> map_spatial_t;
  map_spatial_t map_spatial;

  // hash map: sha => object (needed internally for setting up pointers)
  typedef boost::unordered_map<std::string,tObject*> map_objects_t;
  map_objects_t map_objects;

  // show progress information
  bool verbose;
};

///////////////////////////////////////////////////////////////////////////////
// versioning information
BOOST_CLASS_VERSION(Objects::tReference, 0)
BOOST_CLASS_VERSION(Objects::tSize, 0)
BOOST_CLASS_VERSION(Objects::tOrientation, 0)
BOOST_CLASS_VERSION(Objects::tPoint, 0)
BOOST_CLASS_VERSION(Objects::tPolyline, 0)
BOOST_CLASS_VERSION(Objects::tBox, 0)
BOOST_CLASS_VERSION(Objects::tRelation, 0)
BOOST_CLASS_VERSION(Objects::tObject, 0)
BOOST_CLASS_VERSION(Objects, 0)

#endif

