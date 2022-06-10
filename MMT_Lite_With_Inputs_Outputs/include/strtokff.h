// build_8, 170512, Ray Sells, DESE Research, Inc.
#ifndef STRTOKFF_H
#define STRTOKFF_H

#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
using namespace std;

namespace tframes {

class StrTokff {
  public:
    StrTokff( char *fname, char *delims);
    StrTokff( string fname, string delims);
    vector<string> str_split( string s);
    vector<string> readlines();
  private:
    char *delims;
    char *fname;
};

}
#endif
