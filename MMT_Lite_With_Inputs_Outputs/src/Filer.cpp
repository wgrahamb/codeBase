//-----------------------------------------------------------//
// File: Filer.cpp
// 
// This class provides methods to read in data from input files.
//                                                           
// Developer: Ray Sells                            
// Other Developers: Dennis Strickland
//-----------------------------------------------------------//

#include "Filer.h"
#include "stdlib.h"

Filer::Filer( string file, string delim): iValue( 1){
  f = new tframes::StrTok( file, delim);
  vlines = f->readlines();
  line0 = 0;
}

string Filer::getParam( string name) {

  for( int i = line0; i < ( int)vlines.size(); i++) {
    vector<string> vs = f->str_split( vlines[i]);
      //if( vs[0] == name) {
      if( vs.at(0) == name) {
        //Store data for output later
        //ds->pushInputs(vs.at(0), vs.at(1));
        //
        //return vs[iValue];
        return vs.at(iValue);
      }
  }
  cout << "Error: " << name << " not found\n";
  exit( 1);
  return "Error";
}

void Filer::setLine0( string name) {
  for( int i = 0; i < ( int)vlines.size(); i++) {
    if( vlines[i].find( name) != string::npos) {
      line0 = i + 1;
      return;
    }
  }
  cout << "Error (setLine0): " << name << " not found\n";
  exit( 1);
}

long Filer::getLong( string name) {
  return atol( this->getParam( name).c_str());
}

int Filer::getInt( string name) {
  return atoi( this->getParam( name).c_str());
}

double Filer::getDouble( string name) {
  return atof( this->getParam( name).c_str());
}

string Filer::getString( string name) {
  return this->getParam( name);
}

