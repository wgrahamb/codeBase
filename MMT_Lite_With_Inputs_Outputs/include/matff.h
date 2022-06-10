// build_8, 170512, Ray Sells, DESE Research, Inc.
#ifndef MATFF_H
#define MATFF_H

#include <iostream>
#include <cmath>
using namespace std;

#include "vecff.h"

namespace tframes {

class Quatff;

class Matff {
  public:
    Vecff v0;
    Vecff v1;
    Vecff v2;
    Matff();
    Matff( Vecff, Vecff, Vecff);
    Matff( float, float, float,
         float, float, float,
         float, float, float);
    Vecff &operator[]( int i);
    Matff operator()( Vecff x, Vecff y, Vecff z);
    Matff operator()( float a00, float a01, float a02,
                    float a10, float a11, float a12,
                    float a20, float a21, float a22);
    Matff apply( float (*fn)( float x));
    Matff scale( float a);
    Matff operator+( Matff mm);
    Matff operator-( Matff mm);
    Matff operator*( float a);
    Matff operator/( float a);
    Matff operator*=( float a);
    Matff transpose();
    Matff operator*( Matff m1);
    Vecff operator*( Vecff v);
    float det();
    Matff inv();

    Vecff getEuler();
    Quatff getQuatff();
  private:
};

#ifdef SIXDOF
ostream &operator<<( ostream &stream, Matff m);
#endif
} // end namespace tframes

#endif

