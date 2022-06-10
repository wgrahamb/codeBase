// build_8, 170512, Ray Sells, DESE Research, Inc.
#include "tableff.h"
#include <stdlib.h>

namespace tframes {

Tableff::Tableff( const char *fname) {
  this->fname = fname;
  this->tabnameFound = 0;
  this->tabRead = 0;
}

float Tableff::rand0( float b1, float b2) {
  return ( float)rand() / RAND_MAX * ( b2 - b1) + b1;
}

float Tableff::limit( float x, float b1, float b2) {
  float y = x;
  if( x < b1) {
    y = b1;
  }
  else if( x > b2) {
    y = b2;
  }
  return y;
}

void Tableff::binsearch( float x, float v[],
  int n, int *il, int *im, float *d)
/* Binary search of vector v of length n with independent variable x.
   il is bounds x on lower side and im bounds x on upper side with d
   being the location of x between il and im. */
/* v1.00 by Ray Sells */
{
	int high, low, mid;

	if( x <= v[0])
	{
		*il = *im = 0;
		*d = 0.;
		return;
	}
	if( x >= v[n-1])
	{
		*il = *im = n - 1;
		*d = 0.;
		return;
	}
	low = 0;
	high = n - 1;
	while( low <= high)
	{
		mid = ( low + high) / 2;
		if( x < v[mid]) high = mid - 1;
		else if (x > v[mid]) low = mid + 1;
		else
		{
			*il = mid;
			*im = mid + 1;
			*d = ( x - v[*il]) / ( v[*im] - v[*il]);
			return;
		}
	}
	*il = high;
	*im = high + 1;
	*d = ( x- v[*il]) / ( v[*im] - v[*il]);
}
}

