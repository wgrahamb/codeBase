// v50207, by Ray Sells, DESE Research, Inc.
#ifndef STRTOK_H
#define STRTOK_H

#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
using namespace std;

namespace tframes {

class StrTok {
  public:
    StrTok( char *fname, char *delims);
    StrTok( string fname, string delims);
    vector<string> str_split( string s);
    vector<string> readlines();
  private:
    char *delims;
    char *fname;
};

}
#endif
