//Developed by:  Dennis Strickland
//
#ifndef TABLE5_H
#define TABLE5_H
#include "table.h"
#ifdef SIXDOF
#include "strtok.h"
#endif

namespace tframes {

class Table5 : public Table {
  public:
    Table5( const char *fname);
       #ifdef SIXDOF
    void read( const char *tabname, bool echo);
       #endif
    double interp( double xi1, double xi2, double xi3, double xi4, double xi5);
    int test( int n);
    double operator()( double w, double x, double y, double z, double a)
      { return interp( w, x, y, z, a);};
  private:
       #ifdef SIXDOF
    string x1name, x2name, x3name, x4name, x5name, yname;
       #endif
    double *y;
    int nx1, nx2, nx3, nx4, nx5;
    double *x1, *x2, *x3, *x4, *x5;
       #ifdef SIXDOF
    friend ostream &operator<<( ostream &stream, Table5);
       #endif
};

#ifdef SIXDOF
ostream &operator<<( ostream &stream, Table5);
#endif

}
#endif
