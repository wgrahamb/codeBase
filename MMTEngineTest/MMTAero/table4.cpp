//Developed by:  Dennis Strickland
//
#include "table4.h"
#include <cstring> // JBD
#include <cstdlib> // JBD
#include "Output.h"

extern Output *out;

namespace tframes {

Table4::Table4( const char *fname) : Table( fname) {
  //cout << "Table4::Table4()" << endl;
}

#ifdef SIXDOF
int Table4::test( int n) {
  if( !tabRead) {
    cout << "Error - " << tabname << " table not read\n";
    return 0;
  }
  int passed = 1;
  if( strcmp( fname, "table.dat") != 0) {
    cout << "Error:" << endl;
    cout << "Table4() must be instantiated with file \"table.dat\"" << endl;
    cout <<  " for Table4()->test to work." << endl;
    return 0;
  }
  // test at points
  for( int i = 0; i < nx1; i++) {
    double p1 = x1[i];
    for( int j = 0; j < nx2; j++) {
      double p2 = x2[j];
      for( int k = 0; k < nx3; k++) {
        double p3 = x3[k];
        for ( int l = 0; l < nx4; l++) {
            double p4 = x4[l];
          double y = p1 * p2 * p3 * p4;
          double yi = interp( p1, p2, p3, p4);
          if( fabs( yi - y) > 1e-6) {
            passed = 0;
            cout << "failed at points! " << p1 << " " << p2 << " " << p3 << " " << p4 << " ";
            cout << yi << endl;
          } // end if
        } // end l
      } // end k
    } // end j
  } // end i

  // test between points
  for( int i = 0; i < n; i++) {
    double p1 = rand0( 0., 5.);
    double p2 = rand0( 0., 20.);
    double p3 = rand0( 0., 11.);
    double p4 = rand0( 0., 21.);
    double pp1 = limit( p1, 1., 4.);
    double pp2 = limit( p2, 5., 15.);
    double pp3 = limit( p3, 1., 10.);
    double pp4 = limit( p4, 1., 20.);
    double y = pp1 * pp2 * pp3 * pp4;
    double yi = interp( p1, p2, p3, p4);
    if( fabs( yi - y) > 1e-6) {
      passed = 0;
      cout << "failed b/w points! " << p1 << " " << p2 << " " << p3 << " " << p4 << " ";
      cout << yi << endl;
    }
  }
  return passed;
}

double Table4::interp( double xi1, double xi2, double xi3, double xi4) {
  if( !tabRead) {
    cout << "Error - " << tabname << " table not read\n";
    return 0.0;
  }
	int ilr, imr, ilc, imc, ils, ims, ilq, imq; // indicies of nearest neighbors
    int m, n; 
	double dr, dc, ds, dq; // interpolation slope in direction
	double *x; // data array
	double p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16; // values of nearest neighbors
    double xp1, xp2, xp3, xp4, xp5, xp6, xp7, xp8; // interpolated values in xi1 direction
    double xr1, xr2, xr3, xr4; // interpolated values in xi2 direction
    double xs1, xs2; // interpolated values in xi3 direction

    // find nearest neighbor indicies
	binsearch( xi1, x1, nx1, &ilr, &imr, &dr);
	binsearch( xi2, x2, nx2, &ilc, &imc, &dc);
	binsearch( xi3, x3, nx3, &ils, &ims, &ds);
	binsearch( xi4, x4, nx4, &ilq, &imq, &dq);

    // map to internal array
	x = y;
    // data bounds
	n = nx1 * nx2 * nx3;
        m = nx1 * nx2;
 
        p1 = x[ilq * n + ils * m + ilr * nx2 + ilc];
	p2 = x[ilq * n + ils * m + ilr * nx2 + imc];
	p3 = x[ilq * n + ils * m + imr * nx2 + ilc];
	p4 = x[ilq * n + ils * m + imr * nx2 + imc];
	p5 = x[ilq * n + ims * m + ilr * nx2 + ilc];
	p6 = x[ilq * n + ims * m + ilr * nx2 + imc];
	p7 = x[ilq * n + ims * m + imr * nx2 + ilc];
	p8 = x[ilq * n + ims * m + imr * nx2 + imc];
    // Highest values of xi4
	p9 = x[imq * n + ils * m + ilr * nx2 + ilc];
	p10 = x[imq * n + ils * m + ilr * nx2 + imc];
	p11 = x[imq * n + ils * m + imr * nx2 + ilc];
	p12 = x[imq * n + ils * m + imr * nx2 + imc];
	p13 = x[imq * n + ims * m + ilr * nx2 + ilc];
	p14 = x[imq * n + ims * m + ilr * nx2 + imc];
	p15 = x[imq * n + ims * m + imr * nx2 + ilc];
	p16 = x[imq * n + ims * m + imr * nx2 + imc];

    
    // interpolations in xi1 direction
	xp1 = dr * ( p3 - p1) + p1;
	xp2 = dr * ( p4 - p2) + p2;
	xp3 = dr * ( p7 - p5) + p5;
	xp4 = dr * ( p8 - p6) + p6;
        xp5 = dr * ( p11 - p9) + p9;
        xp6 = dr * ( p12 - p10) + p10;
        xp7 = dr * ( p15 - p13) + p13;
        xp8 = dr * ( p16 - p14) + p14;
    // interpolations in xi2 direction
	xr1 = dc * ( xp2 - xp1) + xp1;
	xr2 = dc * ( xp4 - xp3) + xp3;
	xr3 = dc * ( xp6 - xp5) + xp5;
	xr4 = dc * ( xp8 - xp7) + xp7;
    // interpolations in xi3 direction
        xs1 = ds * (xr2 - xr1) + xr1;
        xs2 = ds * (xr4 - xr3) + xr3;
    // interpolation in xi4 direction
	return dq * ( xs2 - xs1) + xs1;
}

void Table4::read( const char *tabname, bool echo) {
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
      // x dimension sizes  
      nx1 = atoi( vs[0].c_str());
      nx2 = atoi( vs[1].c_str());
      nx3 = atoi( vs[2].c_str());
      nx4 = atoi( vs[3].c_str());
      // x dimension independant values
      x1 = new double[nx1];
      x2 = new double[nx2];
      x3 = new double[nx3];
      x4 = new double[nx4];
      y = new double[ nx1 * nx2 * nx3 * nx4]; // space for all dimensions
      // dimension 1 name & independant values
      vs = f->str_split( vlines[++i]);
      x1name = vs[0];

      vs = f->str_split( vlines[++i]);
      for( int j = 0; j < nx1; j++) {
        x1[j] = atof( vs[j].c_str());
      }
      // dimension 2 name & independant values
      vs = f->str_split( vlines[++i]);
      x2name = vs[0];

      vs = f->str_split( vlines[++i]);
      for( int j = 0; j < nx2; j++) {
        x2[j] = atof( vs[j].c_str());
      }
      // dimension 3 name & independant values
      vs = f->str_split( vlines[++i]);
      x3name = vs[0];

      vs = f->str_split( vlines[++i]);
      for( int j = 0; j < nx3; j++) {
        x3[j] = atof( vs[j].c_str());
      }
      // dimension 4 name & independant values
      vs = f->str_split( vlines[++i]);
      x4name = vs[0];
        
      vs = f->str_split( vlines[++i]);
      for( int j = 0; j < nx4; j++) {
        x4[j] = atof( vs[j].c_str());
      }
      // dependant variable name
      vs = f->str_split( vlines[++i]);
      yname = vs[0];
      // read table data  
      for ( int l = 0; l < nx1; l++ ) { 
        for( int m = 0; m < nx2; m++) {
          for( int j = 0; j < nx3; j++) {
            // skip empty lines 
            do {
              vs = f->str_split( vlines[++i]);
            } while( vs[0] == "");
            // read line data
            for( int k = 0; k < nx4; k++) {
              y[k * (nx1 * nx2 * nx3) + j * ( nx1 * nx2) + l * nx2 + m] = atof( vs[k].c_str());
            } // end for
          } // end j
        } // end m
      } // end l
 
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

ostream &operator<<( ostream &stream, Table4 t) {
  stream << t.tabname << endl;
  stream << t.nx1 << " " << t.nx2 << " " << t.nx3 << " " << t.nx4 << endl;
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
  stream << t.x3name << endl;
  for( int j = 0; j < t.nx3; j++) {
    stream << t.x3[j] << " ";
  }
  stream << endl;
  stream << t.x4name << endl;
  for( int j = 0; j < t.nx4; j++) {
    stream << t.x4[j] << " ";
  }
  stream << endl << t.yname << endl;
  for ( int l = 0; l < t.nx1; l++) {
    for( int m = 0; m < t.nx2; m++) {
      for( int j = 0; j < t.nx3; j++) {
        for( int k = 0; k < t.nx4; k++) {
          stream << t.y[k * ( t.nx1 * t.nx2 * t.nx3) + l * ( t.nx1 * t.nx2) + m * t.nx2 + j] << " ";
        } // end nx2
        stream << endl;
      } // end nx1
    } // end nx3
  } // end nx4
  return stream;
}
#endif
}

