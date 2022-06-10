// v130802
#define SIXDOF
#ifdef SIXDOF
#include <iostream>
#endif
#include <cmath>

#ifndef VEC_H
#define VEC_H

using namespace std;

namespace tframes {

class Mat;
class Quat;

class Vec {
  public:
    double x, y, z, m;
    Vec( double x, double y, double z);
    Vec();
    Vec extract( double &x, double &y, double &z);
    Vec operator()( double, double, double);
    double &operator[]( int i);
    Vec operator+( Vec v0);
    Vec operator*( double a);
    Vec operator=( double a);
    Vec operator/( double a);
    Vec operator*=( double a);
    Vec operator-( Vec v0);
    Vec scale( double a);
    double mag();
    Vec apply( double (*fn)( double x));

    Mat getDCM();
    Mat getDCM_PYR();
    Vec unit();
    double dot( Vec);
    Vec cross( Vec);
    Quat getQuat();
  private:
};
#ifdef SIXDOF
ostream &operator<<( ostream &stream, Vec v);
#endif
} // end namespace tframes

#endif

