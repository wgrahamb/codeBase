// build_8, 170512, Ray Sells, DESE Research, Inc.
#ifndef TABLE2FF_H
#define TABLE2FF_H
#include "tableff.h"
#ifdef SIXDOF
#include "strtokff.h"
#endif


namespace tframes {

class Table2ff : public Tableff {
  public:
    Table2ff( const char *fname);
	#ifdef SIXDOF
    void read( char *tabname, bool echo);
	#endif
    float interp( float xi1, float xi2);
    int test( int n);
    float operator()( float x, float y) { return interp( x, y);};
  private:
	#ifdef SIXDOF
    string x1name, x2name, yname;
	#endif
    float *y;
    int nx1, nx2;
    float *x1, *x2;
	#ifdef SIXDOF
    friend ostream &operator<<( ostream &stream, Table2ff);
	#endif
};
#ifdef SIXDOF
ostream &operator<<( ostream &stream, Table2ff);
#endif
}
#endif
