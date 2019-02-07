#include <iostream>
#include <stdio.h>

#include <boost/program_options.hpp>
#include <QApplication>
#include <QDesktopWidget>

#include "mapviewer.h"
#include "osm.h"

using namespace std;

namespace po = boost::program_options;

int main (int argc, char *argv[]) {

  // options
  string inputfile               = "";
  string outputfile              = "";
  bool   display                 = false;
  bool   prune_buildings         = false;
  bool   prune_non_drivables     = false;
  bool   prune_tracks            = false;
  bool   prune_service_roads     = false;
  bool   split_streets           = false;
  double street_segment_min_dist = -1;

  // parse options
  po::options_description desc("Options");
  desc.add_options()
  ( "help,h", "help message")
  ( "input,i", po::value(&inputfile), "input file (*.osm,*.bin,*.txt,*.xml)")
  ( "output,o", po::value(&outputfile), "output file (*.osm,*.bin,*.txt,*.xml)")
  ( "display,d", "display map")
  ( "prune-buildings", "remove buildings from map")
  ( "prune-non-drivables", "remove non-drivable streets from map")
  ( "prune-tracks", "remove gravel/agricultural streets from map")
  ( "prune-service-roads", "remove service roads from map")
  ( "split-streets", "split streets at intersections")
  ( "street-segment-min-dist", po::value(&street_segment_min_dist), "merge street segments smaller than given threshold (in meters)")
  ;
  po::variables_map vm;
  po::store(po::parse_command_line(argc,argv,desc),vm);
  po::notify(vm);
  if (vm.count("help") || inputfile=="") { cout << desc << endl; return 0; }
  if (vm.count("display")              ) { display = true;                 }
  if (vm.count("prune-buildings")      ) { prune_buildings = true;         }
  if (vm.count("prune-non-drivables")  ) { prune_non_drivables = true;     }
  if (vm.count("prune-tracks")         ) { prune_tracks = true;            }
  if (vm.count("prune-service-roads")  ) { prune_service_roads = true;     }
  if (vm.count("split-streets")        ) { split_streets = true;           }

  // get file extensions
  string inputext  = utils::get_file_extension(inputfile);
  string outputext = utils::get_file_extension(outputfile);

  // check extensions
  bool input_ok = inputext=="osm" || inputext=="bin" || inputext=="txt" || inputext=="xml";
  bool output_ok = outputfile=="" || outputext=="bin" || outputext=="txt" || outputext=="xml";
  if (!input_ok || !output_ok) {
    cout << "Only [osm|bin|txt|xml] => [bin|txt|xml] conversions supported at the moment" << endl;
    return 0;
  }

  // instanciate objects and osm interface
  Objects *objects = new Objects(true);
  OpenStreetMap *osm = new OpenStreetMap(objects,true);

  // load file
  if (inputext=="osm") {
    osm->read(inputfile);
    objects->deleteInvalidObjects();
    objects->deleteUnusedPoints();
  } else {
    objects->loadFromFile(inputfile);
  }

  // remove buildings
  if (prune_buildings) {
    objects->deletePolylines("building");
    objects->deleteUnusedPoints();
  }

  // remove non-drivable streets
  if (prune_non_drivables) {
    vector<string> drivables;
    drivables.push_back("motorway");
    drivables.push_back("motorway_link");
    drivables.push_back("trunk");
    drivables.push_back("trunk_link");
    drivables.push_back("primary");
    drivables.push_back("primary_link");
    drivables.push_back("secondary");
    drivables.push_back("secondary_link");
    drivables.push_back("tertiary");
    drivables.push_back("tertiary_link");
    drivables.push_back("living_street");
    drivables.push_back("residential");
    drivables.push_back("service");
    drivables.push_back("unclassified");
    drivables.push_back("track");
    objects->keepPolylines("street",drivables);
    objects->deleteUnusedPoints();
  }

  if (prune_service_roads) {
    objects->deletePolylines("street","service");
    objects->deleteUnusedPoints();
  }
 
  // remove tracks (gravel/agricultural roads)
  if (prune_tracks) {
    objects->deletePolylines("street","track");
    objects->deleteUnusedPoints();
  }

  // split streets at intersections
  if (split_streets) {
    objects->splitPolylines("street");
  }

  // remove small street segments
  if (street_segment_min_dist>0) {
    objects->deleteSmallSegments(street_segment_min_dist,"street");
    objects->deleteUnusedPoints();
  }

  // store results
  if (outputext!="") {
    objects->saveToFile(outputfile);
  }

  // display map
  if (display) {
    QApplication app(argc,argv);
    MapViewer mapviewer(objects);
    mapviewer.show();
    app.exec();
  }

  // release memory
  delete osm;
  delete objects;

  // exit
  return 0;
}
