#ifndef OSMREADER_H
#define OSMREADER_H

#include <boost/unordered_map.hpp>

#include "objects.h"

#define MAX_BYTES_PER_LINE 4096

class OpenStreetMap {

public:

  // constructor, takes object class to read from / write to
  OpenStreetMap (Objects *objects,bool verbose=false) : objects(objects), verbose(verbose) {}

  // deconstructor
  ~OpenStreetMap (){}

  // read osm file (*.osm), store objects
  bool read (std::string filename);

private:

  int         stringToValue (std::string str, int val);
  double      stringToValue (std::string str, double val);
  std::string stringToValue (std::string str, std::string val);

  template <class T> T getValue (std::string current_line,std::string key,T val);
  std::string readTag (std::string current_line);

  void readNode (std::string current_line);
  void readWay (std::string current_line);
  void readWayNode (std::string current_line);
  void readWayTag (std::string current_line);
  void parseCurrentLine (const char* current_line_cstr,int &num_objects);

  enum tState {NONE,NODE,WAY,RELATION};
  tState current_state;

  Objects *objects;
  FILE    *file;

  boost::unordered_map<int,std::string> map_nodes;
  boost::unordered_map<int,std::string> map_ways;

  Objects::tObject *object;

  bool verbose;

};

#endif // OSMREADER_H
