#include "osm.h"

using namespace std;

int OpenStreetMap::stringToValue (string str, int val) {
  sscanf(str.c_str(),"%d",&val);
  return val;
}

double OpenStreetMap::stringToValue (string str, double val) {
  sscanf(str.c_str(),"%lf",&val);
  return val;
}

string OpenStreetMap::stringToValue (string str, string val) {
  if (val==val) return str;
  else          return str;
}

template <class T> T OpenStreetMap::getValue (string current_line,string key,T val) {
  size_t i,j;
  i = current_line.find(key);
  i = current_line.find_first_of("\'\"",i)+1;
  j = current_line.find_first_of("\'\"",i)-1;
  if (i!=string::npos && j!=string::npos && j>=i)
    val = stringToValue(current_line.substr(i,j-i+1),val);
  return val;
}

string OpenStreetMap::readTag (string current_line) {
  string tag;
  size_t i,j;
  for (i=0; i<current_line.size(); ++i)
    if (current_line[i]=='<')
      break;
  i++;
  for (j=i; j<current_line.size(); ++j)
    if (current_line[j]==' ' || current_line[j]=='>')
      break;
  j--;
  if (j>i && j-i<20)
    tag.assign(current_line,i,j-i+1);
  return tag;
}

void OpenStreetMap::readNode (string current_line) {
  int    id  = getValue<int>   (current_line,"id", 0);
  double lat = getValue<double>(current_line,"lat",0);
  double lon = getValue<double>(current_line,"lon",0);
  Objects::tPoint point(Coordinates::tGeographic(lat,lon),-1);
  object = new Objects::tObject(point);
  object->type = "point";
  object->tag.insert(pair<string,string>("id",getValue<string>(current_line,"id", "")));
  map_nodes.insert(pair<int,string>(id,object->sha));
}

void OpenStreetMap::readWay (string current_line) {
  int id = getValue<int>(current_line,"id",0);
  object = new Objects::tObject(Objects::tPolyline());
  object->tag.insert(pair<string,string>("id",getValue<string>(current_line,"id", "")));
  map_nodes.insert(pair<int,string>(id,object->sha));
}

void OpenStreetMap::readWayNode (string current_line) {
  string sha = map_nodes[getValue<int>(current_line,"ref",0)];
  Objects::tPolyline &polyline = boost::get<Objects::tPolyline>(object->payload);
  polyline.points.push_back(Objects::tReference(sha));
}

void OpenStreetMap::readWayTag (string current_line) {
  string key = getValue<string>(current_line,"k","");
  string val = getValue<string>(current_line,"v","");
  if (current_state==WAY) {
    if (key=="building" && val=="yes") {
      object->type = "building";
    } else if (key=="highway") {
      object->type = "street";
      object->kind = val;
    } else if (key=="lanes" || key=="lit" || key=="maxspeed" || key=="name" || key=="oneway" || key=="ref") {
      object->tag.insert(pair<string,string>(key,val));
    } else if (key=="addr:city" || key=="addr:housenumber" || key=="addr:postcode" || key=="addr:street") {
      object->tag.insert(pair<string,string>(key.substr(5,key.size()-5),val));
    }
  }
}

void OpenStreetMap::parseCurrentLine (const char* current_line_cstr,int &num_objects) {

  // convert to stl string
  string current_line = current_line_cstr;

  // get tag
  string tag = readTag(current_line);

  // process current line of file
  if (tag=="node") {
    readNode(current_line);
    objects->addObject(object);
    current_state = NODE;
    num_objects++;
  } else if (tag=="way") {
    readWay(current_line);
    current_state = WAY;
  } else if (current_state==WAY && tag=="nd") {
    readWayNode(current_line);
  } else if (current_state==WAY && tag=="tag") {
    readWayTag(current_line);
  } else if (current_state==WAY && tag=="/way") {
    objects->addObject(object);
    num_objects++;
  }
}

bool OpenStreetMap::read (string filename) {

  // verbose message
  if (verbose)
    cout << "loading: " << filename << " ... ";

  // space for storing current line
  char current_line[MAX_BYTES_PER_LINE];

  // open file
  file = fopen(filename.c_str(),"r");
  if (file==NULL)
    return false;

  // init
  current_state = NONE;
  map_nodes.clear();
  map_ways.clear();

  // object count
  int num_objects = 0;

  // read file
  while (fgets(current_line,MAX_BYTES_PER_LINE,file)!=NULL)
    parseCurrentLine(current_line,num_objects);

  // close file
  fclose(file);

  // verbose message
  if (verbose)
    cout << num_objects << " objects read." << endl;

  // return success
  return true;
}

