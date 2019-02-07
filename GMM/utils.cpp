#include "utils.h"

using namespace std;

namespace utils {

  // generate random sha key
  string rand_sha (int length) {
    string sha((size_t)length,' ');
    for (int i=0; i<length; i++) {
      switch (rand()%16) {
        case 0:  sha[i] = '0'; break;
        case 1:  sha[i] = '1'; break;
        case 2:  sha[i] = '2'; break;
        case 3:  sha[i] = '3'; break;
        case 4:  sha[i] = '4'; break;
        case 5:  sha[i] = '5'; break;
        case 6:  sha[i] = '6'; break;
        case 7:  sha[i] = '7'; break;
        case 8:  sha[i] = '8'; break;
        case 9:  sha[i] = '9'; break;
        case 10: sha[i] = 'a'; break;
        case 11: sha[i] = 'b'; break;
        case 12: sha[i] = 'c'; break;
        case 13: sha[i] = 'd'; break;
        case 14: sha[i] = 'e'; break;
        case 15: sha[i] = 'f'; break;
      }
    }
    return sha;
  }

  // get file extension from file name
  string get_file_extension (const string& filename) {
    if(filename.find_last_of(".")!=string::npos)
      return filename.substr(filename.find_last_of(".")+1);
    return "";
  }
}

