//Developed by:  Dennis Strickland
//
#ifndef TABLE4_H
#define TABLE4_H
#include "table.h"
#ifdef SIXDOF
#include "strtok.h"
#endif

namespace tframes {

class Table4 : public Table {
  public:
    Table4( const char *fname);
       #ifdef SIXDOF
    void read( const char *tabname, bool echo);
       #endif
    double interp( double xi1, double xi2, double xi3, double xi4);
    int test( int n);
    double operator()( double w, double x, double y, double z)
      { return interp( w, x, y, z);};
  private:
       #ifdef SIXDOF
    string x1name, x2name, x3name, x4name, yname;
       #endif
    double *y;
    int nx1, nx2, nx3, nx4;
    double *x1, *x2, *x3, *x4;
       #ifdef SIXDOF
    friend ostream &operator<<( ostream &stream, Table4);
       #endif
};

#ifdef SIXDOF
ostream &operator<<( ostream &stream, Table4);
#endif

}
#endif
