// v130802
#ifndef TABLE2_H
#define TABLE2_H
#include "table.h"
#ifdef SIXDOF
#include "strtok.h"
#endif


namespace tframes {

class Table2 : public Table {
  public:
    Table2( const char *fname);
	#ifdef SIXDOF
    void read( const char *tabname, bool echo);
	#endif
    double interp( double xi1, double xi2);
    int test( int n);
    double operator()( double x, double y) { return interp( x, y);};
  private:
	#ifdef SIXDOF
    string x1name, x2name, yname;
	#endif
    double *y;
    int nx1, nx2;
    double *x1, *x2;
	#ifdef SIXDOF
    friend ostream &operator<<( ostream &stream, Table2);
	#endif
};
#ifdef SIXDOF
ostream &operator<<( ostream &stream, Table2);
#endif
}
#endif
