#define SIXDOF
#ifndef TABLE_H
#define TABLE_H

#include <iostream>
#include <cmath>
using namespace std;

namespace tframes {

class Table {
  public:
    Table( const char *fname);
  protected:
    int tabnameFound;
    int tabRead;
    double rand0( double b1, double b2);
    double limit( double x, double b1, double b2);
    bool echo;
    const char *fname;
    const char *tabname;
    void binsearch( double x, double v[],
      int n, int *il, int *im, double *d);
};

}
#endif

