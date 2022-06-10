// v130802
#include "quat.h"
#include <stdlib.h>

namespace tframes {

Quat::Quat() {
  s = 0.0;
  x = 0.0;
  y = 0.0;
  z = 0.0;
}

Quat::Quat( double s, double x, double y, double z) {
  this->s = s;
  this->x = x;
  this->y = y;
  this->z = z;
}

Quat Quat::operator()( double s, double x, double y, double z) {
  this->s = s;
  this->x = x;
  this->y = y;
  this->z = z;
  return *this;
}

double &Quat::operator[]( int i) {
  switch ( i) {
    case 0:
      return s;
    case 1:
      return x;
    case 2:
      return y;
    case 3:
      return z;
  }
#ifdef SIXDOF
  cout << "Invalid index for accessing Quat - " << i << endl;
#endif
  exit(1);
  return x;
}
#ifdef SIXDOF
ostream &operator<<( ostream &stream, Quat q) {
  stream << q.s << " " << q.x << " " << q.y << " " << q.z;
  return stream;
}
#endif
Quat Quat::normalize() {
  Quat q = *this, qq;

  double q_mag = sqrt( q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

  qq[0] = q[0] / q_mag;
  qq[1] = q[1] / q_mag;
  qq[2] = q[2] / q_mag;
  qq[3] = q[3] / q_mag;

  return qq;
}

Mat Quat::getDCM() {
  Mat m;
  Quat q = *this;

  m[0][0] = 2.0 * (q[0] * q[0] + q[1] * q[1]) - 1.0;
  m[0][1] = 2.0 * (q[1] * q[2] + q[0] * q[3]);
  m[0][2] = 2.0 * (q[1] * q[3] - q[0] * q[2]);
  m[1][0] = 2.0 * (q[1] * q[2] - q[0] * q[3]);
  m[1][1] = 2.0 * (q[0] * q[0] + q[2] * q[2]) - 1.0;
  m[1][2] = 2.0 * (q[2] * q[3] + q[0] * q[1]);
  m[2][0] = 2.0 * (q[1] * q[3] + q[0] * q[2]);
  m[2][1] = 2.0 * (q[2] * q[3] - q[0] * q[1]);
  m[2][2] = 2.0 * (q[0] * q[0] + q[3] * q[3]) - 1.0;

  return m;

}

} // end of namespace tframes












