// v50907, by Ray Sells, DESE Research, Inc.
// modified by Dennis Strickland
#include "table3.h"
#include <cstring> //JBD
#include <cstdlib> //JBD
#include "Output.h"

extern Output *out;

namespace tframes {

Table3::Table3( const char *fname) : Table( fname) {
  //cout << "Table3::Table3()" << endl;
}

#ifdef SIXDOF
int Table3::test( int n) {
  if( !tabRead) {
    cout << "Error - " << tabname << " table not read\n";
    return 0;
  }
  int passed = 1;
  if( strcmp( fname, "table.dat") != 0) {
    cout << "Error:" << endl;
    cout << "Table3() must be instantiated with file \"table.dat\"" << endl;
    cout <<  " for Table3()->test to work." << endl;
    return 0;
  }
  // test at points
  for( int i = 0; i < nx1; i++) {
    double p1 = x1[i];
    for( int j = 0; j < nx2; j++) {
      double p2 = x2[j];
      for( int k = 0; k < nx3; k++) {
        double p3 = x3[k];
        double y = p1 * p2 * p3;
        double yi = interp( p1, p2, p3);
        if( fabs( yi - y) > 1e-6) {
          passed = 0;
          cout << "failed! " << p1 << " " << p2 << " " << p3 << " ";
          cout << yi << endl;
        }
      }
    }
  }

  // test between points
  for( int i = 0; i < n; i++) {
    double p1 = rand0( 0., 5.);
    double p2 = rand0( 0., 20.);
    double p3 = rand0( 0., 11.);
    double pp1 = limit( p1, 1., 4.);
    double pp2 = limit( p2, 5., 15.);
    double pp3 = limit( p3, 1., 10.);
    double y = pp1 * pp2 * pp3;
    double yi = interp( p1, p2, p3);
    if( fabs( yi - y) > 1e-6) {
      passed = 0;
      cout << "failed! " << p1 << " " << p2 << " " << p3 << " ";
      cout << yi << endl;
    }
  }
  return passed;
}

double Table3::interp( double xi1, double xi2, double xi3) {
/* Three-dimensional linear interpolation. */
  if( !tabRead) {
    cout << "Error - " << tabname << " table not read\n";
    return 0.0;
  }
	int ilr, imr, ilc, imc, ils, ims, m;
	double dr, dc, ds;
	double *x;
	double p1, p2, p3, p4, p5, p6, p7, p8, xp1, xp2, xp3, xp4, xr1, xr2;

	binsearch( xi1, x1, nx1, &ilr, &imr, &dr);
	binsearch( xi2, x2, nx2, &ilc, &imc, &dc);
	binsearch( xi3, x3, nx3, &ils, &ims, &ds);

	x = y;
	m = nx1 * nx2;

	p1 = x[ils * m + ilr * nx2 + ilc];
	p2 = x[ils * m + ilr * nx2 + imc];
	p3 = x[ils * m + imr * nx2 + ilc];
	p4 = x[ils * m + imr * nx2 + imc];
        //
	p5 = x[ims * m + ilr * nx2 + ilc];
	p6 = x[ims * m + ilr * nx2 + imc];
	p7 = x[ims * m + imr * nx2 + ilc];
	p8 = x[ims * m + imr * nx2 + imc];
        //
	xp1 = dr * ( p3 - p1) + p1;
	xp2 = dr * ( p4 - p2) + p2;
	xp3 = dr * ( p7 - p5) + p5;
	xp4 = dr * ( p8 - p6) + p6;
	xr1 = dc * ( xp2 - xp1) + xp1;
	xr2 = dc * ( xp4 - xp3) + xp3;
	return ds * ( xr2 - xr1) + xr1;
}

void Table3::read( const char *tabname, bool echo) {
  this->tabname = tabname;

  StrTok *f = new StrTok( fname, " =\t");

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
      nx3 = atoi( vs[2].c_str());

      x1 = new double[nx1];
      x2 = new double[nx2];
      x3 = new double[nx3];
      y  = new double[ nx1 * nx2 * nx3];

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
      x3name = vs[0];

      vs = f->str_split( vlines[++i]);
      for( int j = 0; j < nx3; j++) {
        x3[j] = atof( vs[j].c_str());
      }
      vs = f->str_split( vlines[++i]);
      yname = vs[0];

      for( int m = 0; m < nx1; m++) {
        for( int j = 0; j < nx2; j++) {
          do {
            vs = f->str_split( vlines[++i]);
          } while( vs[0] == "");
          for( int k = 0; k < nx3; k++) {
            y[ k * ( nx1 * nx2) + m * nx2 + j] = atof( vs[k].c_str());
          }
        }
      }
      tabRead = 1;
      break;
    } // if
  } // for
  if( !tabnameFound) {
    cout << "Error - " << tabname << " not found." << endl;
    return;
  }
  if( echo) {
    cout << *this;
  }
}

ostream &operator<<( ostream &stream, Table3 t) {

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
    stream << endl;
    stream << t.x2name << endl;
    for( int j = 0; j < t.nx2; j++) {
      stream << t.x2[j] << " ";
    }
    stream << endl << t.yname << endl;
    for( int m = 0; m < t.nx3; m++) {
      for( int j = 0; j < t.nx1; j++) {
        for( int k = 0; k < t.nx2; k++) {
          stream << t.y[ m * ( t.nx1 * t.nx2) + j * t.nx2 + k] << " ";
        }
        stream << endl;
      }
    }
  }
  return stream;
}
#endif
}

