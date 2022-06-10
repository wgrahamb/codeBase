// build_8, 170512, Ray Sells, DESE Research, Inc.
#include "strtokff.h"

namespace tframes {

StrTokff::StrTokff( char *fname, char *delims) {
  this->delims = delims;
  this->fname = fname;
}

StrTokff::StrTokff( string fname, string delims) {
  this->fname = new char[fname.size() + 1];
  this->delims = new char[delims.size() + 1];
  strcpy( this->fname, ( char *)fname.c_str());
  strcpy( this->delims, ( char *)delims.c_str());
}

vector<string> StrTokff::readlines() {
  vector<string> vs;

  ifstream in( fname);
  string line;
  while( getline( in, line)) {
    vs.push_back( line);
  }
  in.close();
  return vs;
}

vector<string> StrTokff::str_split( string s) {
  //char *p = new char[s.size() + 1];
  char p[256];
  strcpy( p, ( char *)s.c_str());

  vector<string> v;
  char *token = strtok( p, delims);
  while( token != NULL) {
    string st = token;
    v.push_back( st);
    token = strtok( NULL, delims);
  }
  if( v.empty()) {
    v.push_back( "");
  }
  //delete [] p;
  return v;
}

/*
vector<string> StrTokff::str_split( string s) {
  // Home-grown tokenizer since strtok() seems to no work
  // properly in gcc3.2 compiler.
  // Ray Sells, 20 August 2003.
  string ds = delims;
  vector<string> v;
  if( s.size() == 0) {
    v.push_back( "");
    return v;
  }
  size_t n0 = 0, n1 = 0, len;
  do {
    n0 = s.find_first_not_of( ds, n0);
    n1 = s.find_first_of( ds, n0 + 1);
    if( n1 == string::npos) {
      n1 = s.size();
    }
    len = n1 - n0;
    v.push_back( s.substr( n0, len));
    n0 = n1;
  } while( n1 != s.size());
  if( v.empty()) {
    v.push_back( "");
  }
  return v;
}
*/

}



