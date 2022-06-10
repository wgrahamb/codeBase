// build_8, 170512, Ray Sells, DESE Research, Inc.
#ifndef TABLE1FF_H
#define TABLE1FF_H
#include "tableff.h"
#ifdef SIXDOF
#include "strtokff.h"
#endif


namespace tframes {

class Table1ff : public Tableff {
  public:
    Table1ff( const char *fname);
	#ifdef SIXDOF
    void read( const char *tabname, bool echo);
    //void read_db( char *tabname, DataBox *db, bool echo);
	#endif
    float interp( float xi);
	#ifdef SIXDOF
    int test( int n);
	#endif
    float operator()( float x) { return interp( x);};
  private:
	#ifdef SIXDOF
    string x1name, yname;
	#endif
    float *y;
    int nx1;
    float *x1;

	#ifdef SIXDOF
    friend ostream &operator<<( ostream &stream, Table1ff);
	#endif
};

#ifdef SIXDOF
ostream &operator<<( ostream &stream, Table1ff);
#endif
}
#endif
