// v130802
#include "vec.h"
#include "mat.h"
#include "quat.h"
#include <stdlib.h>

namespace tframes {

Vec::Vec() {
  x = 0.0;
  y = 0.0;
  z = 0.0;
}

Vec::Vec( double x, double y, double z) {
  this->x = x;
  this->y = y;
  this->z = z;
}

Vec Vec::operator()( double x, double y, double z) {
  this->x = x;
  this->y = y;
  this->z = z;
  return *this;
}

Vec Vec::extract( double &x, double &y, double &z) {
  x = this->x;
  y = this->y;
  z = this->z;
  return *this;
}

double &Vec::operator[]( int i) {
  switch ( i) {
    case 0:
      return x;
    case 1:
      return y;
    case 2:
      return z;
  }
#ifdef SIXDOF
  cout << "Invalid index for accessing Vec - " << i << endl;
#endif
  exit(1);
  return x;
}

#ifdef SIXDOF
ostream &operator<<( ostream &stream, Vec v) {
  stream << v.x << " " << v.y << " " << v.z;
  return stream;
}
#endif

double Vec::mag() {
  double x = this->x;
  double y = this->y;
  double z = this->z;
  double mag = sqrt( x * x + y * y + z * z);
  return mag;
}

Vec Vec::apply( double (*fn)( double x)) {
  Vec v;
  v.x = ( *fn)( this->x);
  v.y = ( *fn)( this->y);
  v.z = ( *fn)( this->z);
  return v;
}

Vec Vec::scale( double a) {
  Vec v;
  v.x = this->x * a;
  v.y = this->y * a;
  v.z = this->z * a;
  return v;
}

Vec Vec::operator=( double a) {
  this->x = a;
  this->y = a;
  this->z = a;
  return *this;
}

Vec Vec::operator*( double a) {
  Vec v;
  v.x = this->x * a;
  v.y = this->y * a;
  v.z = this->z * a;
  return v;
}

Vec Vec::operator/( double a) {
  Vec v;
  v.x = this->x / a;
  v.y = this->y / a;
  v.z = this->z / a;
  return v;
}

Vec Vec::operator*=( double a) {
  this->x *= a;
  this->y *= a;
  this->z *= a;
  return *this;
}

Vec Vec::operator+( Vec v0) {
  Vec v;
  v.x = this->x + v0.x;
  v.y = this->y + v0.y;
  v.z = this->z + v0.z;
  return v;
}

Vec Vec::operator-( Vec v0) {
  Vec v;
  v.x = this->x - v0.x;
  v.y = this->y - v0.y;
  v.z = this->z - v0.z;
  return v;
}

Vec Vec::unit() {
  Vec v;
  double a = this->mag();

  v.x = this->x / a;
  v.y = this->y / a;
  v.z = this->z / a;
  return v;
}

Mat Vec::getDCM() {
  /* 3-2-1 sequence */
  Mat m;
  double phi, theta, psi;

  (*this).extract( phi, theta, psi);

	double c1 = cos( phi);
	double c2 = cos( theta);
	double c3 = cos( psi);
	double s1 = sin( phi);
	double s2 = sin( theta);
	double s3 = sin( psi);

	m[0][0] = c2 * c3;
	m[0][1] = c2 * s3;
	m[0][2] = -s2;
	m[1][0] = s1 * s2 * c3 - c1 * s3;
	m[1][1] = s1 * s2 * s3 + c1 * c3;
	m[1][2] = c2 * s1;
	m[2][0] = c1 * s2 * c3 + s1 * s3;
	m[2][1] = c1 * s2 * s3 - s1 * c3;
	m[2][2] = c2 * c1;
  return m;
}

Mat Vec::getDCM_PYR() {
  /* 2-3-1 sequence */
  Mat m;
  double phi, theta, psi;

  (*this).extract( phi, theta, psi);

        double c1 = cos( phi);
        double c2 = cos( theta);
        double c3 = cos( psi);
        double s1 = sin( phi);
        double s2 = sin( theta);
        double s3 = sin( psi);

        m[0][0] = c2 * c3;
        m[0][1] = s3;
        m[0][2] = -s2 * c3;
        m[1][0] = s1 * s2 - c2 * s3 * c1;
        m[1][1] = c3 * c1;
        m[1][2] = s2 * s3 * c1 + c2 * s1;
        m[2][0] = c2 * s3 * s1 + s2 * c1;
        m[2][1] = -c3 * s1;
        m[2][2] = -s2 * s3 * s1 + c2 * c1;
  return m;
}

double Vec::dot( Vec v) {
  return this->x * v.x + this->y * v.y + this->z * v.z;
}

Vec Vec::cross( Vec v) {
  Vec vv;
  vv.x = this->y * v.z - this->z * v.y;
  vv.y = this->z * v.x - this->x * v.z;
  vv.z = this->x * v.y - this->y * v.x;
  return vv;
}

Quat Vec::getQuat() {
  Vec vEuler = *this;
  Quat q;

  double phi, theta, psi;
  vEuler.extract( phi, theta, psi);

  double cpsi2 = cos( psi / 2.0);
  double spsi2 = sin( psi / 2.0);
  double ctht2 = cos( theta / 2.0);
  double stht2 = sin( theta / 2.0 );
  double cphi2 = cos( phi / 2.0 );
  double sphi2 = sin( phi / 2.0 );

  q[0] = (cpsi2 * ctht2 * cphi2) + (spsi2 * stht2 * sphi2);
  q[1] = (cpsi2 * ctht2 * sphi2) - (spsi2 * stht2 * cphi2);
  q[2] = (cpsi2 * stht2 * cphi2) + (spsi2 * ctht2 * sphi2);
  q[3] = (spsi2 * ctht2 * cphi2) - (cpsi2 * stht2 * sphi2);

  return q;
}
 
} // end of namespace tframes












