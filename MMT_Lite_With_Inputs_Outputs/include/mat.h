// v130802
#ifndef MAT_H
#define MAT_H

#include <iostream>
#include <cmath>
using namespace std;

#include "vec.h"

namespace tframes {

class Quat;

class Mat {
  public:
    Vec v0;
    Vec v1;
    Vec v2;
    Mat();
    Mat( Vec, Vec, Vec);
    Mat( double, double, double,
         double, double, double,
         double, double, double);
    Vec &operator[]( int i);
    Mat operator()( Vec x, Vec y, Vec z);
    Mat operator()( double a00, double a01, double a02,
                    double a10, double a11, double a12,
                    double a20, double a21, double a22);
    Mat apply( double (*fn)( double x));
    Mat scale( double a);
    Mat operator+( Mat mm);
    Mat operator-( Mat mm);
    Mat operator*( double a);
    Mat operator/( double a);
    Mat operator*=( double a);
    Mat transpose();
    Mat operator*( Mat m1);
    Vec operator*( Vec v);
    double det();
    Mat inv();

    Vec getEuler();
    Quat getQuat();
  private:
};

#ifdef SIXDOF
ostream &operator<<( ostream &stream, Mat m);
#endif
} // end namespace tframes

#endif

