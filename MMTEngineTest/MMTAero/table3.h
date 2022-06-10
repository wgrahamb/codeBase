// v50907, by Ray Sells, DESE Research, Inc.
#ifndef TABLE3_H
#define TABLE3_H
#include "table.h"
#ifdef SIXDOF
#include "strtok.h"
#endif

namespace tframes {

class Table3 : public Table {
  public:
    Table3( const char *fname);
      #ifdef SIXDOF
    void read( const char *tabname, bool echo);
      #endif
    double interp( double xi1, double xi2, double xi3);
    int test( int n);
    double operator()( double x, double y, double z)
      { return interp( x, y, z);};
  private:
      #ifdef SIXDOF
    string x1name, x2name, x3name, yname;
      #endif
    double *y;
    int nx1, nx2, nx3;
    double *x1, *x2, *x3;
      #ifdef SIXDOF
    friend ostream &operator<<( ostream &stream, Table3);
      #endif
};

#ifdef SIXDOF
ostream &operator<<( ostream &stream, Table3);
#endif

}
#endif
