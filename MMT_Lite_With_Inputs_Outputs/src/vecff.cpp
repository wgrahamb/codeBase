// build_8, 170512, Ray Sells, DESE Research, Inc.
#include "vecff.h"
#include "matff.h"
#include "quatff.h"
#include <stdlib.h>

namespace tframes {

Vecff::Vecff() {
  x = 0.0;
  y = 0.0;
  z = 0.0;
}

Vecff::Vecff( float x, float y, float z) {
  this->x = x;
  this->y = y;
  this->z = z;
}

Vecff Vecff::operator()( float x, float y, float z) {
  this->x = x;
  this->y = y;
  this->z = z;
  return *this;
}

Vecff Vecff::extract( float &x, float &y, float &z) {
  x = this->x;
  y = this->y;
  z = this->z;
  return *this;
}

float &Vecff::operator[]( int i) {
  switch ( i) {
    case 0:
      return x;
    case 1:
      return y;
    case 2:
      return z;
  }
#ifdef SIXDOF
  cout << "Invalid index for accessing Vecff - " << i << endl;
#endif
  exit(1);
  return x;
}

#ifdef SIXDOF
ostream &operator<<( ostream &stream, Vecff v) {
  stream << v.x << " " << v.y << " " << v.z;
  return stream;
}
#endif

float Vecff::mag() {
  float x = this->x;
  float y = this->y;
  float z = this->z;
  float mag = sqrt( x * x + y * y + z * z);
  return mag;
}

Vecff Vecff::apply( float (*fn)( float x)) {
  Vecff v;
  v.x = ( *fn)( this->x);
  v.y = ( *fn)( this->y);
  v.z = ( *fn)( this->z);
  return v;
}

Vecff Vecff::scale( float a) {
  Vecff v;
  v.x = this->x * a;
  v.y = this->y * a;
  v.z = this->z * a;
  return v;
}

Vecff Vecff::operator=( float a) {
  this->x = a;
  this->y = a;
  this->z = a;
  return *this;
}

Vecff Vecff::operator*( float a) {
  Vecff v;
  v.x = this->x * a;
  v.y = this->y * a;
  v.z = this->z * a;
  return v;
}

Vecff Vecff::operator/( float a) {
  Vecff v;
  v.x = this->x / a;
  v.y = this->y / a;
  v.z = this->z / a;
  return v;
}

Vecff Vecff::operator*=( float a) {
  this->x *= a;
  this->y *= a;
  this->z *= a;
  return *this;
}

Vecff Vecff::operator+( Vecff v0) {
  Vecff v;
  v.x = this->x + v0.x;
  v.y = this->y + v0.y;
  v.z = this->z + v0.z;
  return v;
}

Vecff Vecff::operator-( Vecff v0) {
  Vecff v;
  v.x = this->x - v0.x;
  v.y = this->y - v0.y;
  v.z = this->z - v0.z;
  return v;
}

Vecff Vecff::unit() {
  Vecff v;
  float a = this->mag();

  v.x = this->x / a;
  v.y = this->y / a;
  v.z = this->z / a;
  return v;
}

Matff Vecff::getDCM() {
  /* 3-2-1 sequence */
  Matff m;
  float phi, theta, psi;

  (*this).extract( phi, theta, psi);

	float c1 = cos( phi);
	float c2 = cos( theta);
	float c3 = cos( psi);
	float s1 = sin( phi);
	float s2 = sin( theta);
	float s3 = sin( psi);

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

Matff Vecff::getDCM_PYR() {
  /* 2-3-1 sequence */
  Matff m;
  float phi, theta, psi;

  (*this).extract( phi, theta, psi);

        float c1 = cos( phi);
        float c2 = cos( theta);
        float c3 = cos( psi);
        float s1 = sin( phi);
        float s2 = sin( theta);
        float s3 = sin( psi);

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

float Vecff::dot( Vecff v) {
  return this->x * v.x + this->y * v.y + this->z * v.z;
}

Vecff Vecff::cross( Vecff v) {
  Vecff vv;
  vv.x = this->y * v.z - this->z * v.y;
  vv.y = this->z * v.x - this->x * v.z;
  vv.z = this->x * v.y - this->y * v.x;
  return vv;
}

Quatff Vecff::getQuatff() {
  Vecff vEuler = *this;
  Quatff q;

  float phi, theta, psi;
  vEuler.extract( phi, theta, psi);

  float cpsi2 = cos( psi / 2.0);
  float spsi2 = sin( psi / 2.0);
  float ctht2 = cos( theta / 2.0);
  float stht2 = sin( theta / 2.0 );
  float cphi2 = cos( phi / 2.0 );
  float sphi2 = sin( phi / 2.0 );

  q[0] = (cpsi2 * ctht2 * cphi2) + (spsi2 * stht2 * sphi2);
  q[1] = (cpsi2 * ctht2 * sphi2) - (spsi2 * stht2 * cphi2);
  q[2] = (cpsi2 * stht2 * cphi2) + (spsi2 * ctht2 * sphi2);
  q[3] = (spsi2 * ctht2 * cphi2) - (cpsi2 * stht2 * sphi2);

  return q;
}

} // end of namespace tframes












