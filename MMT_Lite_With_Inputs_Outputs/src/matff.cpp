// build_8, 170512, Ray Sells, DESE Research, Inc.
#include "matff.h"
#include "quatff.h"
#include <stdlib.h>

namespace tframes {

Matff::Matff() {
  v0( 0.0, 0.0, 0.0);
  v1( 0.0, 0.0, 0.0);
  v2( 0.0, 0.0, 0.0);
}

Matff::Matff( Vecff v0, Vecff v1, Vecff v2) {
  this->v0 = v0;
  this->v1 = v1;
  this->v2 = v2;
}

Matff::Matff( float a00, float a01, float a02,
          float a10, float a11, float a12,
          float a20, float a21, float a22) {
  v0( a00, a01, a02);
  v1( a10, a11, a12);
  v2( a20, a21, a22);
}

Vecff &Matff::operator[]( int i) {
  switch ( i) {
    case 0:
      return v0;
    case 1:
      return v1;
    case 2:
      return v2;
  }
#ifdef SIXDOF
  cout << "Invalid index for accessing Matff - " << i << endl;
#endif
  exit(1);
  return v0;
}

#ifdef SIXDOF
ostream &operator<<( ostream &stream, Matff m) {
  stream << m[0][0] << " " << m[0][1] << " " << m[0][2] << " ";
  stream << m[1][0] << " " << m[1][1] << " " << m[1][2] << " ";
  stream << m[2][0] << " " << m[2][1] << " " << m[2][2] ;
  return stream;
}
#endif

Matff Matff::operator()( Vecff x, Vecff y, Vecff z) {
  this->v0 = x;
  this->v1 = y;
  this->v2 = z;
  return *this;
}

Matff Matff::operator()( float a00, float a01, float a02,
                     float a10, float a11, float a12,
                     float a20, float a21, float a22) {
  this->v0( a00, a01, a02);
  this->v1( a10, a11, a12);
  this->v2( a20, a21, a22);
  return *this;
}

Matff Matff::scale( float a) {
  Matff m;
  m.v0 = this->v0.scale( a);
  m.v1 = this->v1.scale( a);
  m.v2 = this->v2.scale( a);
  return m;
}

Matff Matff::operator+( Matff mm) {
  Matff m;
  m[0] = this->v0 + mm[0];
  m[1] = this->v1 + mm[1];
  m[2] = this->v2 + mm[2];
  return m;
}

Matff Matff::operator-( Matff mm) {
  Matff m;
  m[0] = this->v0 - mm[0];
  m[1] = this->v1 - mm[1];
  m[2] = this->v2 - mm[2];
  return m;
}

Vecff Matff::operator*( Vecff v) {
  Vecff vm;
  Matff m1 = *this;

	vm.x = m1[0][0] * v.x + m1[0][1] * v.y + m1[0][2] * v.z;
	vm.y = m1[1][0] * v.x + m1[1][1] * v.y + m1[1][2] * v.z;
	vm.z = m1[2][0] * v.x + m1[2][1] * v.y + m1[2][2] * v.z;
  return vm;
}

Matff Matff::operator*=( float a) {
  this->v0 *= a;
  this->v1 *= a;
  this->v2 *= a;
  return *this;
}

Matff Matff::operator*( Matff m2) {
  Matff m;
  Matff m1 = *this;

	m[0][0] = m1[0][0] * m2[0][0] + m1[0][1] * m2[1][0] + m1[0][2] * m2[2][0];
	m[0][1] = m1[0][0] * m2[0][1] + m1[0][1] * m2[1][1] + m1[0][2] * m2[2][1];
	m[0][2] = m1[0][0] * m2[0][2] + m1[0][1] * m2[1][2] + m1[0][2] * m2[2][2];
	m[1][0] = m1[1][0] * m2[0][0] + m1[1][1] * m2[1][0] + m1[1][2] * m2[2][0];
	m[1][1] = m1[1][0] * m2[0][1] + m1[1][1] * m2[1][1] + m1[1][2] * m2[2][1];
	m[1][2] = m1[1][0] * m2[0][2] + m1[1][1] * m2[1][2] + m1[1][2] * m2[2][2];
	m[2][0] = m1[2][0] * m2[0][0] + m1[2][1] * m2[1][0] + m1[2][2] * m2[2][0];
	m[2][1] = m1[2][0] * m2[0][1] + m1[2][1] * m2[1][1] + m1[2][2] * m2[2][1];
	m[2][2] = m1[2][0] * m2[0][2] + m1[2][1] * m2[1][2] + m1[2][2] * m2[2][2];
  return m;
}

Matff Matff::operator*( float a) {
  Matff m;
  m[0] = this->v0 * a;
  m[1] = this->v1 * a;
  m[2] = this->v2 * a;
  return m;
}

Matff Matff::operator/( float a) {
  Matff m;
  m[0] = this->v0 / a;
  m[1] = this->v1 / a;
  m[2] = this->v2 / a;
  return m;
}

Matff Matff::transpose() {
  Matff m;
  m[0]( this->v0[0], this->v1[0], this->v2[0]);
  m[1]( this->v0[1], this->v1[1], this->v2[1]);
  m[2]( this->v0[2], this->v1[2], this->v2[2]);
  return m;
}

Matff Matff::apply( float (*fn)( float x)) {
  Matff m;

  m[0] = (this->v0).apply( fn);
  m[1] = (this->v1).apply( fn);
  m[2] = (this->v2).apply( fn);
  return m;
}

float Matff::det() {
  float a, b, c, d, e, f, g, h, i;
  Matff m = *this;

	a = m[0][0];
	b = m[0][1];
	c = m[0][2];
	d = m[1][0];
	e = m[1][1];
	f = m[1][2];
	g = m[2][0];
	h = m[2][1];
	i = m[2][2];

	return  a * e * i + b * f * g + c * d * h -
	        c * e * g - b * d * i - a * f * h;
}

Matff Matff::inv() {
  float a, b, c, d, e, f, g, h, i;
  Matff m = *this;
  Matff minv;

  float xdet = det();
	if( fabs( xdet) < 1e-6) {
#ifdef SIXDOF
		cout << "warning - matrix may be singular\n";
#endif
	}

	a = m[0][0];
	b = m[0][1];
	c = m[0][2];
	d = m[1][0];
	e = m[1][1];
	f = m[1][2];
	g = m[2][0];
	h = m[2][1];
	i = m[2][2];
	minv[0][0] = ( e * i - f * h) / xdet;
	minv[0][1] = ( c * h - b * i) / xdet;
	minv[0][2] = ( b * f - e * c) / xdet;
	minv[1][0] = ( g * f - d * i) / xdet;
	minv[1][1] = ( a * i - g * c) / xdet;
	minv[1][2] = ( d * c - a * f) / xdet;
	minv[2][0] = ( d * h - g * e) / xdet;
	minv[2][1] = ( g * b - a * h) / xdet;
	minv[2][2] = ( a * e - b * d) / xdet;
  return minv;
}

Vecff Matff::getEuler() {
  /* 3-2-1 sequence */
  Matff m = *this;
  Vecff v;
	float	x[4], y[4];

  x[1]  = m[0][1];
  y[1]  = m[0][0];
  v.x  = atan2( x[1], y[1]);
  x[1] = sin( v.x);
  y[1] = cos( v.x);
  x[3] = m[2][0] * x[1] - m[2][1] * y[1];
  y[3] = m[1][1] * y[1] - m[1][0] * x[1];
  x[2] = -m[0][2];
  y[2] = m[2][2] * y[3] + m[1][2] * x[3];

	v.z = atan2( x[3], y[3]);
	v.y = atan2( x[2], y[2]);

	/* reverse rotation sequence */
	float vv = v.x;
	v.x = v.z;
	v.z = vv;

	return v;
}

Quatff Matff::getQuatff() {
  Quatff q;
  Matff m = *this;
  q[0] = 0.5 * sqrt( m[0][0] + m[1][1] + m[2][2] + 1.0);
  q[1] = ( m[1][2] - m[2][1]) / ( 4.0 * q[0]);
  q[2] = ( m[2][0] - m[0][2]) / ( 4.0 * q[0]);
  q[3] = ( m[0][1] - m[1][0]) / ( 4.0 * q[0]);
  return q;
}

} // end of namespace tframes





