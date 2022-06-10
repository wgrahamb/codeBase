// v130802
#ifndef QUAT_H
#define QUAT_H

#ifdef SIXDOF
#include <iostream>
#endif

#include <cmath>
using namespace std;

#include "mat.h"

namespace tframes {

class Quat {
  public:
    double s, x, y, z;
    Quat( double s, double x, double y, double z);
    Quat();
    Quat operator()( double, double, double, double);
    double &operator[]( int i);
    Quat normalize();
    Mat getDCM();
  private:
};
#ifdef SIXDOF
ostream &operator<<( ostream &stream, Quat q);
#endif
} // end namespace tframes

#endif

