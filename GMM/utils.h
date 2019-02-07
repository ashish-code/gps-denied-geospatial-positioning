#ifndef __OSM__UTILS_H
#define __OSM__UTILS_H

#include <cmath>
#include <string>

#include <iostream>
#include <sstream>
#include <fstream>

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>

namespace utils {

  // generate random sha key
  std::string rand_sha (int length=40);

  // get file extension from file name
  std::string get_file_extension (const std::string& filename);

  // load object from binary, text or xml file
  template <class T> bool load_from_file (std::string filename,std::string name,T *object) {
    try {
      std::string ext = get_file_extension(filename);
      if (ext=="bin") {
        std::ifstream ifs(filename.c_str(),std::ifstream::binary);
        boost::archive::binary_iarchive ia(ifs);
        ia >> *object;
      } else if (ext=="txt") {
        std::ifstream ifs(filename.c_str());
        boost::archive::text_iarchive ia(ifs);
        ia >> *object;
      } else if (ext=="xml") {
        std::ifstream ifs(filename.c_str());
        boost::archive::xml_iarchive ia(ifs);
        ia >> boost::serialization::make_nvp(name.c_str(),*object);
      } else {
        return false;
      }
      return true;
    } catch (...) {
      return false;
    }
  }

  // save object to binary, text or xml file
  template <class T> bool save_to_file (std::string filename,std::string name,T *object) {
    try {
      std::string ext = get_file_extension(filename);
      if (ext=="bin") {
        std::ofstream ofs(filename.c_str(),std::ofstream::binary);
        boost::archive::binary_oarchive oa(ofs);
        oa << *object;
      } else if (ext=="txt") {
        std::ofstream ofs(filename.c_str());
        boost::archive::text_oarchive oa(ofs);
        oa << *object;
      } else if (ext=="xml") {
        std::ofstream ofs(filename.c_str());
        boost::archive::xml_oarchive oa(ofs);
        oa << boost::serialization::make_nvp(name.c_str(),*object);
      } else {
        return false;
      }
      return true;
    } catch (...) {
      return false;
    }
  }
  
  template <typename T> inline T wrap_to_2pi (T phi) {
    bool negative_input = phi<0;
    phi = fmod(phi, static_cast<T>(2*M_PI));
    if(negative_input)
      phi += static_cast<T>(2*M_PI);
    return phi;
  }
  
  template <typename T> inline T wrap_to_pi (T phi) {
    phi = wrap_to_2pi(phi + static_cast<T>(M_PI)) - static_cast<T>(M_PI);
    return phi;
  }

  template <typename T> std::string num_to_string (T number) {
    std::ostringstream oss;
    oss << number;
    return oss.str();
  }
}

#endif

