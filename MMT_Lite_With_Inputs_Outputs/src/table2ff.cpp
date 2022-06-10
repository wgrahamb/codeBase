// build_8, 170512, Ray Sells, DESE Research, Inc.
#include "table2ff.h"
#include <stdlib.h>
#include "Output.h"

extern Output *out;

namespace tframes {

Table2ff::Table2ff( const char *fname) : Tableff( fname) {
  //cout << "Table2ff::Table2ff()" << endl;
}

#ifdef SIXDOF
int Table2ff::test( int n) {
  if( !tabRead) {
    cout << "Error - " << tabname << " table not read\n";
    return 0;
  }
  int passed = 1;
  if( strcmp( fname, "table.dat") != 0) {
    cout << "Error:" << endl;
    cout << "Table2ff() must be instantiated with file \"table.dat\"" << endl;
    cout <<  " for Table2ff()->test to work." << endl;
    return 0;
  }
  // test at points
  for( int i = 0; i < nx1; i++) {
    float p1 = x1[i];
    for( int j = 0; j < nx2; j++) {
      float p2 = x2[j];
      float y = p1 * p2;
      float yi = interp( p1, p2);
      if( fabs( yi - y) > 1e-6) {
        passed = 0;
        cout << "failed! " << p1 << " " << p2 << " " << yi << endl;
      }
    }
  }

  // test between points
  for( int i = 0; i < n; i++) {
    float p1 = rand0( 0., 5.);
    float p2 = rand0( 0., 16.);
    float pp1 = limit( p1, 1.0, 4.0);
    float pp2 = limit( p2, 5.0, 15.0);
    float y = pp1 * pp2;
    float yi = interp( p1, p2);
    if( fabs( yi - y) > 1e-6) {
      passed = 0;
      cout << "failed! " << p1 << " " << p2 << " " << yi << endl;
    }
  }
  return passed;
}
#endif

float Table2ff::interp( float xi1, float xi2)
/* Two-dimensional linear interpolation. */
/* v1.00 by Ray Sells */
{
  if( !tabRead) {
	#ifdef SIXDOF
    cout << "Error - " << tabname << " table not read\n";
	#endif
    return 0.0;
  }

	int ilr, imr, ilc, imc;
	float dr, dc;
	float *x;
	float p1, p2, p3, p4, xp1, xp2;

	binsearch( xi1, x1, nx1, &ilr, &imr, &dr);
	binsearch( xi2, x2, nx2, &ilc, &imc, &dc);

	x = y;
	p1 = x[ilr * nx2 + ilc];
	p2 = x[ilr * nx2 + imc];
	p3 = x[imr * nx2 + ilc];
	p4 = x[imr * nx2 + imc];
	xp1 = dr * ( p3 - p1) + p1;
	xp2 = dr * ( p4 - p2) + p2;
	return dc * ( xp2 - xp1) + xp1;
}

#ifdef SIXDOF
void Table2ff::read( char *tabname, bool echo) {
  this->tabname = tabname;

  StrTokff *f = new StrTokff( fname, " =\t");

  vector<string> vlines = f->readlines();

  for( int i = 0; i < ( int)vlines.size(); i++) {
    vector<string> vs;
    vs = f->str_split( vlines[i]);
    if( vs[0] == "") {
      continue;
    }
    if( strcmp( vs[0].c_str(), tabname) == 0) {
      tabnameFound = 1;

      vs = f->str_split( vlines[++i]);
      nx1 = atoi( vs[0].c_str());
      nx2 = atoi( vs[1].c_str());
      x1 = new float[nx1];
      x2 = new float[nx2];
      y = new float[ nx1 * nx2];

      vs = f->str_split( vlines[++i]);
      x1name = vs[0];

      vs = f->str_split( vlines[++i]);
      for( int j = 0; j < nx1; j++) {
        x1[j] = atof( vs[j].c_str());
      }

      vs = f->str_split( vlines[++i]);
      x2name = vs[0];

      vs = f->str_split( vlines[++i]);
      for( int j = 0; j < nx2; j++) {
        x2[j] = atof( vs[j].c_str());
      }

      vs = f->str_split( vlines[++i]);
      yname = vs[0];
      for( int j = 0; j < nx1; j++) {
        vs = f->str_split( vlines[++i]);
        for( int k = 0; k < nx2; k++) {
          y[ j * nx2 + k] = atof( vs[k].c_str());
        }
      }
      tabRead = 1;
      break;
    }
  }
  if( !tabnameFound) {
    cout << "Error - " << tabname << " not found." << endl;
    return;
  }
  if( echo) {
    cout << *this;
  }
}

ostream &operator<<( ostream &stream, Table2ff t) {

  if(out->displayInput == 1) {
    stream << t.tabname << endl;
    stream << t.nx1 << " " << t.nx2 << endl;
    stream << t.x1name << endl;
    for( int j = 0; j < t.nx1; j++) {
      stream << t.x1[j] << " ";
    }
    stream << endl;
    stream << t.x2name << endl;
    for( int j = 0; j < t.nx2; j++) {
      stream << t.x2[j] << " ";
    }
    stream << endl << t.yname << endl;
    for( int j = 0; j < t.nx1; j++) {
      for( int k = 0; k < t.nx2; k++) {
        stream << t.y[ j * t.nx2 + k] << " ";
      }
      stream << endl;
    }
  }
  return stream;
}
#endif
}
