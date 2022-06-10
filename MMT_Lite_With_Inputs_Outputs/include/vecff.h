// build_8, 170512, Ray Sells, DESE Research, Inc.
#define SIXDOF
#ifdef SIXDOF
#include <iostream>
#endif
#include <cmath>

#ifndef VECFF_H
#define VECFF_H

using namespace std;

namespace tframes {

class Matff;
class Quatff;

class Vecff {
  public:
    float x, y, z, m;
    Vecff( float x, float y, float z);
    Vecff();
    Vecff extract( float &x, float &y, float &z);
    Vecff operator()( float, float, float);
    float &operator[]( int i);
    Vecff operator+( Vecff v0);
    Vecff operator*( float a);
    Vecff operator=( float a);
    Vecff operator/( float a);
    Vecff operator*=( float a);
    Vecff operator-( Vecff v0);
    Vecff scale( float a);
    float mag();
    Vecff apply( float (*fn)( float x));

    Matff getDCM();
    Matff getDCM_PYR();
    Vecff unit();
    float dot( Vecff);
    Vecff cross( Vecff);
    Quatff getQuatff();
  private:
};
#ifdef SIXDOF
ostream &operator<<( ostream &stream, Vecff v);
#endif
} // end namespace tframes

#endif

