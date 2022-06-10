//-----------------------------------------------------------//
// File: Filer.h
// 
// This class provides methods to read in data from input files.
//                                                           
// Developer: Ray Sells                            
// Other Developers: Dennis Strickland
//-----------------------------------------------------------//

#ifndef FILER_H
#define FILER_H

#include <iostream>
#include <string>
#include <vector>
#include "strtok.h"

class Filer {
  public:
    Filer( string file, string delim = " \t=;");
    double x;
    string getParam( string name);
    int getInt( string name);
    long getLong( string name);
    double getDouble( string name);
    string getString( string name);
    void setLine0( string s);
    int iValue;
    FILE *fp;
  private:
    int line0;
    vector<string> vlines;
    tframes::StrTok *f;
};

#endif
