#include "objects.h"

#include <boost/utility.hpp>

#define SPATIAL_HASH_FACTOR 1e2

using namespace std;

Objects::Objects (bool verbose) : verbose(verbose) {

  // seed random number generator (for generating sha keys)
  srand (time(NULL));
}

Objects::~Objects () {

  // delete all objects
  for (objects_t::iterator it=begin(); it!=end(); it++)
    delete(*it);
}

bool Objects::loadFromFile (string filename) {

  // verbose message
  if (verbose)
    cout << "loading: " << filename << " ... " << flush;

  // load objects from file
  if (!utils::load_from_file<Objects>(filename,"objects",this)) {
    if (verbose)
      cout << "failed!" << endl;
    return false;
  }

  // add sha keys to hash map
  for (objects_t::iterator it=begin(); it!=end(); it++)
    map_objects.insert(pair<string,tObject*>((*it)->sha,*it));

  // add pointers
  for (objects_t::iterator it=begin(); it!=end(); it++)
    modPointers(*it,true);

  // verbose message
  if (verbose)
    cout << objects.size() << " objects read." << endl;

  // success
  return true;
}

bool Objects::saveToFile (string filename) {

  // verbose message
  if (verbose)
    cout << "saving: " << filename << " ... ";

  // save objects to file
  bool success = utils::save_to_file<Objects>(filename,"objects",this);

  // verbose message
  if (verbose)
    cout << objects.size() << " objects saved." << endl;

  // save objects to file
  return success;
}

void Objects::addObject (tObject *object) {

  // store object pointer
  objects.insert(object);

  // add sha key of object to hash map
  map_objects.insert(pair<string,tObject*>(object->sha,object));

  // add reference and usage pointers
  modPointers(object,true);
}

void Objects::delObject (objects_t::iterator &it) {

  // remove pointers
  modPointers (*it,false);

  // remove sha key in hash map
  map_objects.erase((*it)->sha);

  // delete object
  delete *it;

  // remove object from list
  objects.erase(it);
}

int Objects::numberOfObjects () {
  return objects.size();
}

void Objects::modPointer (tReference &ref,Objects::tObject *object,bool add) {

  // get target index
  tObject *target = map_objects[ref.sha];

  // add pointers
  if (add) {
    ref.ptr = target;
    target->use.insert(object);

  // delete pointers
  } else {
    ref.ptr = 0;
    target->use.erase(object);
  }
}

void Objects::modPointers (Objects::tObject *object,bool add) {

  // point (modify spatial hash)
  if (isType<Objects::tPoint>(object)) {
    Objects::tPoint &point = getPayload<Objects::tPoint>(object);
    size_t hash = spatialHash(point.geo);
    if (add) {
      map_spatial.insert(pair<size_t,Objects::tObject*>(hash,object));
    } else {
      pair<map_spatial_t::iterator,map_spatial_t::iterator> range = map_spatial.equal_range(hash);
      map_spatial_t::iterator it=range.first;
      while (it!=range.second)
        if (it->second==object)
          it = map_spatial.erase(it);
        else
          it++;
    }

  // polyline (modify pointers)
  } else if (isType<Objects::tPolyline>(object)) {
    Objects::tPolyline &polyline = getPayload<Objects::tPolyline>(object);
    for (int i=0; i<(int)polyline.points.size(); i++)
      modPointer(polyline.points[i],object,add);

  // box (modify pointers)
  } else if (isType<Objects::tBox>(object)) {
    Objects::tBox &box = getPayload<Objects::tBox>(object);
    modPointer(box.pos,object,add);

  // relation (modify pointers)
  } else if (isType<Objects::tRelation>(object)) {
    Objects::tRelation &relation = getPayload<Objects::tRelation>(object);
    for (int i=0; i<(int)relation.objects.size(); i++)
      modPointer(relation.objects[i],object,add);
  }
}

size_t Objects::spatialHash (double x,double y) {
  size_t hash = 0xdeadbeef;
  boost::hash_combine(hash,x);
  boost::hash_combine(hash,y);
  return hash;
}

size_t Objects::spatialHash (Coordinates::tGeographic geo) {
  return spatialHash((int)(geo.lat*SPATIAL_HASH_FACTOR),
                     (int)(geo.lon*SPATIAL_HASH_FACTOR));
}

Objects::tRange Objects::getMapRange () {

  // verbose message
  if (verbose)
    cout << "calculating map range: ";

  // map range
  tRange range;

  // initialize range
  range.geo_min.lat = +numeric_limits<double>::max();
  range.geo_max.lat = -numeric_limits<double>::max();
  range.geo_min.lon = +numeric_limits<double>::max();
  range.geo_max.lon = -numeric_limits<double>::max();

  // loop through all points and adjust map range
  for (objects_t::iterator it=begin(); it!=end(); it++) {
    if (isType<Objects::tPoint>(*it)) {
      Objects::tPoint &point = getPayload<Objects::tPoint>(*it);
      if (point.geo.lat<range.geo_min.lat) range.geo_min.lat = point.geo.lat;
      if (point.geo.lat>range.geo_max.lat) range.geo_max.lat = point.geo.lat;
      if (point.geo.lon<range.geo_min.lon) range.geo_min.lon = point.geo.lon;
      if (point.geo.lon>range.geo_max.lon) range.geo_max.lon = point.geo.lon;
    }
  }

  // verbose message
  if (verbose)
    cout << "lat " << range.geo_min.lat << " - " << range.geo_max.lat << ", lon " << range.geo_min.lon << " - " << range.geo_max.lon << endl;

  // return map range
  return range;
}

void Objects::addObjectsAndParentsToSet (tObject *object,objects_t &set_objects) {

  // add object
  set_objects.insert(object);

  // add ancestors
  for (objects_t::iterator it=object->use.begin(); it!= object->use.end(); it++)
    addObjectsAndParentsToSet(*it,set_objects);
}

Objects::objects_t Objects::getObjectsInRange (tRange range) {

  // set of objects in range
  objects_t set_objects;

  // is range query valid?
  if (range.geo_max.lat<range.geo_min.lat || range.geo_max.lon<range.geo_min.lon)
    return set_objects;

  // geographic coordinates => grid cell indices
  int x_min = (int)(range.geo_min.lat*SPATIAL_HASH_FACTOR);
  int x_max = (int)(range.geo_max.lat*SPATIAL_HASH_FACTOR);
  int y_min = (int)(range.geo_min.lon*SPATIAL_HASH_FACTOR);
  int y_max = (int)(range.geo_max.lon*SPATIAL_HASH_FACTOR);

  // for all grid cells do
  for (int x=x_min; x<=x_max; x++) {
    for (int y=y_min; y<=y_max; y++) {

      // get grid cell hash
      size_t hash = spatialHash(x,y);

      // for all points in cell do
      pair<map_spatial_t::iterator,map_spatial_t::iterator> r = map_spatial.equal_range(hash);
      for (map_spatial_t::iterator it=r.first; it!=r.second; ++it) {

        // get object (only points are stored in map_spatial)
        Objects::tObject* object = it->second;
        Objects::tPoint &point = getPayload<Objects::tPoint>(object);

        // if point is (really) within range, add it and all ancestors to set_objects
        if (point.geo.lat>=range.geo_min.lat && point.geo.lat<=range.geo_max.lat && point.geo.lon>=range.geo_min.lon && point.geo.lon<=range.geo_max.lon)
          addObjectsAndParentsToSet(object,set_objects);
      }
    }
  }

  // return set of objects in range
  return set_objects;
}

Objects::tObject* Objects::getClosestPoint (Coordinates::tGeographic geo) {

  // init empty object and dist
  Objects::tObject* object_min = 0;
  double dist_min = numeric_limits<double>::max();

  // get current grid cell
  int x0 = (int)(geo.lat*SPATIAL_HASH_FACTOR);
  int y0 = (int)(geo.lon*SPATIAL_HASH_FACTOR);
 
  // for all grid cells around that cell do
  for (int x=x0-1; x<=x0+1; x++) {
    for (int y=y0-1; y<=y0+1; y++) {
    
      // get hash of this cell
      size_t hash = spatialHash(x,y);
  
      // for all points in cell do
      pair<map_spatial_t::iterator,map_spatial_t::iterator> r = map_spatial.equal_range(hash);
      for (map_spatial_t::iterator it=r.first; it!=r.second; ++it) {

        // get object (only points are stored in map_spatial)
        Objects::tObject* object = it->second;
        Objects::tPoint &point = getPayload<Objects::tPoint>(object);
        
        // compute pseudo-geographic distance (should be fine)
        double dlat = point.geo.lat-geo.lat;
        double dlon = point.geo.lon-geo.lon;
        double dist = sqrt(dlat*dlat+dlon*dlon);

        // if closer, update pointer
        if (dist<dist_min) {
          dist_min = dist;
          object_min = object;
        }
      }
    }
  }
  
  // return pointer to closest point (or NULL)
  return object_min;
}

int Objects::deleteInvalidObjects () {

  // verbose message
  if (verbose)
    cout << "deleting invalid objects ... ";

  // get number of objects prior to deletion
  int num = objects.size();

  // delete all objects without type
  objects_t::iterator it=begin();
  while (it!=end()) {
    objects_t::iterator it2 = it; ++it;
    if ((*it2)->type=="")
      delObject(it2);
  }

  // verbose message
  if (verbose)
    cout << num-objects.size() << " objects deleted." << endl;

  // return number of objects deleted
  return num-objects.size();
}

int Objects::deleteUnusedPoints () {

  // verbose message
  if (verbose)
    cout << "deleting unused points ... ";

  // get number of objects prior to deletion
  int num = objects.size();

  // delete all points which are not used by any other object
  objects_t::iterator it=begin();
  while (it!=end()) {
    objects_t::iterator it2 = it; ++it;
    if (isType<Objects::tPoint>(*it2) && (*it2)->use.size()==0)
      delObject(it2);
  }

  // verbose message
  if (verbose)
    cout << num-objects.size() << " points deleted." << endl;

  // return number of objects deleted
  return num-objects.size();
}

void Objects::deletePolylines (string type,string kind) {

  // verbose message
  if (verbose) {
    cout << "deleting polylines of type '" << type;
    if (kind!="")
      cout << "' and kind '" << kind;
    cout << "' ... ";
  }
  
  int num_deletes = 0;

  // for all objects do
  objects_t::iterator it=begin();
  while (it!=end()) {

    // increment it, work with copy of it
    objects_t::iterator it2 = it; ++it;

    // only delete polylines of correct type (e.g., street)
    if (isType<tPolyline>(*it2) && (*it2)->type==type && (kind=="" || (*it2)->kind==kind)) {

      // delete polyline
      delObject(it2);
      num_deletes++;
    }
  }

  // verbose message
  if (verbose)
    cout << num_deletes << " polylines deleted." << endl;
}

void Objects::keepPolylines (string type,vector<string> kind) {

  // verbose message
  if (verbose) {
    cout << "deleting polylines of type '" << type;
    cout << "' and not of kind: '";
    for (vector<string>::iterator it=kind.begin(); it!=kind.end(); it++) {
      cout << *it;
      if (boost::next(it)!=kind.end())
        cout << "', '";
    }
    cout << "' ... ";
  }
  
  int num_deletes = 0;

  // for all objects do
  objects_t::iterator it=begin();
  while (it!=end()) {

    // increment it, work with copy of it
    objects_t::iterator it2 = it; ++it;

    // only delete polylines of correct type (e.g., street) which are not any of kind
    if (isType<tPolyline>(*it2) && (*it2)->type==type) {
      
      // should we keep this polyline?
      bool keep = false;
      for (vector<string>::iterator it3=kind.begin(); it3!=kind.end(); it3++)
        if (*it3 == (*it2)->kind)
          keep = true;

      // delete polyline
      if (!keep) {
        delObject(it2);
        num_deletes++;
      }
    }
  }

  // verbose message
  if (verbose)
    cout << num_deletes << " polylines deleted." << endl;
}

void Objects::splitPolylines (string type) {

  // verbose message
  if (verbose)
    cout << "splitting polylines of type '" << type << "' ... ";

  // copy vector pointing to objects
  int num_splits = 0;

  // for all objects do
  objects_t::iterator it=begin();
  while (it!=end()) {

    // increment it, work with copy of it
    objects_t::iterator it2 = it; ++it;

    // only split polylines of correct type (e.g., street)
    if (!isType<tPolyline>(*it2) || (*it2)->type!=type)
      continue;

    // get payload
    tPolyline &polyline = getPayload<tPolyline>(*it2);
    vector<tReference> &points = polyline.points;

    // splint indices
    vector<int> split_idx;

    // loop through all points (except first and last), compute split indices
    for (int i=1; i<(int)points.size()-1; i++)
      if (points[i].ptr->use.size()>1)
        split_idx.push_back(i);

    // if this polyline shares 1 or more points => split polyline
    if (split_idx.size()>0) {

      // increment split counter
      num_splits++;

      // for all split indices do
      for (int i=0; i<=(int)split_idx.size(); i++) {

        // compute start and end point index
        int i0,i1;
        if (i==0)                          { i0 = 0;              i1 = split_idx[0];    }
        else if (i==(int)split_idx.size()) { i0 = split_idx[i-1]; i1 = points.size()-1; }
        else                               { i0 = split_idx[i-1]; i1 = split_idx[i];    }

        // create new polyline
        tObject* object_split = new tObject(tPolyline());
        tPolyline &polyline_split = boost::get<tPolyline>(object_split->payload);

        // set type and kind
        object_split->type = (*it2)->type;
        object_split->kind = (*it2)->kind;
        object_split->tag = (*it2)->tag;
        if (i == (int)split_idx.size()/2)
          object_split->tag.insert(pair<string,string>("split_middle","true"));
        else
          object_split->tag.insert(pair<string,string>("split_middle","false"));

        // copy points
        for (int j=i0; j<=i1; j++)
          polyline_split.points.push_back(points[j]);

        // add polyline to objects
        addObject(object_split);
      }

      // delete original polyline
      delObject(it2);
    }
  }

  // verbose message
  if (verbose)
    cout << num_splits << " polylines split." << endl;
}

void Objects::deleteSmallSegments (double min_dist,string type,string kind) {

  // verbose message
  if (verbose) {
    cout << "deleting small segments of type '" << type;
    if (kind!="")
      cout << "' and kind '" << kind;
    cout << "' ... ";
  }

  int num_deletes = 0;

  // for all objects do
  for (objects_t::iterator it=begin(); it!=end(); it++) {

    // only delete segments of correct polyline type (e.g., street)
    if (!isType<tPolyline>(*it) || (*it)->type!=type || (kind!="" && (*it)->kind!=kind))
      continue;

    // get payload
    tPolyline &polyline = getPayload<tPolyline>(*it);

    // new point list to be filled
    vector<tReference> points;

    // walk through list of all point references of this polyline
    for (int i=0; i<(int)polyline.points.size(); i++) {

      // gather information about this point
      bool   is_first_point     = i==0;
      bool   is_last_point      = i==(int)polyline.points.size()-1;
      bool   used_by_others     = polyline.points[i].ptr->use.size()>1;
      double dist_to_prev_point = numeric_limits<double>::max();

      // get distance to previous point in new list
      if (points.size()>0) {
        tPoint &p_last = getPayload<tPoint>(points.back().ptr);
        tPoint &p_curr = getPayload<tPoint>(polyline.points[i].ptr);
        Coordinates coord(p_last.geo);
        dist_to_prev_point = coord.latLonToDistance(p_last.geo,p_curr.geo);
      }

      // copy current point to new vector
      if (is_first_point || is_last_point || used_by_others || dist_to_prev_point>min_dist)
        points.push_back(polyline.points[i]);
    }

    // update number of segment deletions
    num_deletes += polyline.points.size() - points.size();

    // replace points with new vector
    polyline.points = points;
  }

  // verbose message
  if (verbose)
    cout << num_deletes << " small segments deleted." << endl;
}

