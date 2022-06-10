// build_8, 170512, Ray Sells, DESE Research, Inc.
#define SIXDOF
#ifndef TABLEFF_H
#define TABLEFF_H

#include <iostream>
#include <cmath>
using namespace std;

namespace tframes {

class Tableff {
  public:
    Tableff( const char *fname);
  protected:
    int tabnameFound;
    int tabRead;
    float rand0( float b1, float b2);
    float limit( float x, float b1, float b2);
    bool echo;
    const char *fname;
    const char *tabname;
    void binsearch( float x, float v[],
      int n, int *il, int *im, float *d);
};

}
#endif

