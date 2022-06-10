// build_8, 170512, Ray Sells, DESE Research, Inc.
#ifndef QUATFF_H
#define QUATFF_H

#ifdef SIXDOF
#include <iostream>
#endif

#include <cmath>
using namespace std;

#include "matff.h"

namespace tframes {

class Quatff {
  public:
    float s, x, y, z;
    Quatff( float s, float x, float y, float z);
    Quatff();
    Quatff operator()( float, float, float, float);
    float &operator[]( int i);
    Quatff normalize();
    Matff getDCM();
  private:
};
#ifdef SIXDOF
ostream &operator<<( ostream &stream, Quatff q);
#endif
} // end namespace tframes

#endif

