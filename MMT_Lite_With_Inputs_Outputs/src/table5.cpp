//Developed by Dennis Strickland 
//
#include "table5.h"
#include <cstring> // JBD
#include <cstdlib> // JBD
#include "Output.h"

extern Output *out;

namespace tframes {

Table5::Table5( const char *fname) : Table( fname) {
  //cout << "Table5::Table5()" << endl;
}

#ifdef SIXDOF
int Table5::test( int n) {
  if( !tabRead) {
    cout << "Error - " << tabname << " table not read\n";
    return 0;
  }
  int passed = 1;
  if( strcmp( fname, "table.dat") != 0) {
    cout << "Error:" << endl;
    cout << "Table5() must be instantiated with file \"table.dat\"" << endl;
    cout <<  " for Table5()->test to work." << endl;
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
          for ( int m = 0; m < nx5; m++) {
            double p5 = x5[m];
            double y = p1 * p2 * p3 * p4 * p5;
            double yi = interp( p1, p2, p3, p4, p5);
            if( fabs( yi - y) > 1e-6) {
              passed = 0;
              cout << "failed at points! " << p1 << " " << p2 << " " << p3 << " " << p4 << " " << p5 << endl;
              cout << yi << endl;
            } // end if
          } //end m
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
    double p5 = rand0( 0., 32.);
    double pp1 = limit( p1, 1., 4.);
    double pp2 = limit( p2, 5., 15.);
    double pp3 = limit( p3, 1., 10.);
    double pp4 = limit( p4, 1., 20.);
    double pp5 = limit( p5, 1., 25.);
    double y = pp1 * pp2 * pp3 * pp4 * pp5;
    double yi = interp( p1, p2, p3, p4, p5);
    if( fabs( yi - y) > 1e-6) {
      passed = 0;
      cout << "failed b/w points! " << p1 << " " << p2 << " " << p3 << " " << p4 << " " << p5 << endl;
      cout << yi << endl;
    }
  }
  return passed;
}

double Table5::interp( double xi1, double xi2, double xi3, double xi4, double xi5) {
  if( !tabRead) {
    cout << "Error - " << tabname << " table not read\n";
    return 0.0;
  }
    int ilr, imr, ilc, imc, ils, ims, ilq, imq, ilz, imz; // indicies of nearest neighbors
    int m, n, p; 
    double dr, dc, ds, dq, dz; // interpolation slope in direction
    double *x; // data array
    double p1, p2, p3, p4, p5, p6, p7, p8, p9, p10, p11, p12, p13, p14, p15, p16; // values of nearest neighbors
    double p17, p18, p19, p20, p21, p22, p23, p24, p25, p26, p27, p28, p29, p30, p31, p32; // values of nearest neighbors
    double xp1, xp2, xp3, xp4, xp5, xp6, xp7, xp8; // interpolated values in xi1 direction
    double xp9, xp10, xp11, xp12, xp13, xp14, xp15, xp16; // interpolated values in xi1 direction
    double xr1, xr2, xr3, xr4; // interpolated values in xi2 direction
    double xr5, xr6, xr7, xr8; // interpolated values in xi2 direction
    double xs1, xs2, xs3, xs4; // interpolated values in xi3 direction
    double xq1, xq2; // interpolated values in xi4 direction

    // find nearest neighbor indicies
	binsearch( xi1, x1, nx1, &ilr, &imr, &dr);
	binsearch( xi2, x2, nx2, &ilc, &imc, &dc);
	binsearch( xi3, x3, nx3, &ils, &ims, &ds);
	binsearch( xi4, x4, nx4, &ilq, &imq, &dq);
	binsearch( xi5, x5, nx5, &ilz, &imz, &dz);

    // map to internal array
	x = y;
    // data bounds
	//n = nx1 * nx2 * nx3;
        //m = nx1 * nx2;
	p = nx1 * nx2 * nx3 * nx4;
        n = nx1 * nx2 * nx3;
        m = nx1 * nx2;

        p1  = x[ilz * p + ilq * n + ils * m + ilr * nx2 + ilc];
	p2  = x[ilz * p + ilq * n + ils * m + ilr * nx2 + imc];
	p3  = x[ilz * p + ilq * n + ils * m + imr * nx2 + ilc];
	p4  = x[ilz * p + ilq * n + ils * m + imr * nx2 + imc];
	p5  = x[ilz * p + ilq * n + ims * m + ilr * nx2 + ilc];
	p6  = x[ilz * p + ilq * n + ims * m + ilr * nx2 + imc];
	p7  = x[ilz * p + ilq * n + ims * m + imr * nx2 + ilc];
	p8  = x[ilz * p + ilq * n + ims * m + imr * nx2 + imc];
	p9  = x[ilz * p + imq * n + ils * m + ilr * nx2 + ilc];
	p10 = x[ilz * p + imq * n + ils * m + ilr * nx2 + imc];
	p11 = x[ilz * p + imq * n + ils * m + imr * nx2 + ilc];
	p12 = x[ilz * p + imq * n + ils * m + imr * nx2 + imc];
	p13 = x[ilz * p + imq * n + ims * m + ilr * nx2 + ilc];
	p14 = x[ilz * p + imq * n + ims * m + ilr * nx2 + imc];
	p15 = x[ilz * p + imq * n + ims * m + imr * nx2 + ilc];
	p16 = x[ilz * p + imq * n + ims * m + imr * nx2 + imc];
    // Highest values of xi4
        p17 = x[imz * p + ilq * n + ils * m + ilr * nx2 + ilc];
	p18 = x[imz * p + ilq * n + ils * m + ilr * nx2 + imc];
	p19 = x[imz * p + ilq * n + ils * m + imr * nx2 + ilc];
	p20 = x[imz * p + ilq * n + ils * m + imr * nx2 + imc];
	p21 = x[imz * p + ilq * n + ims * m + ilr * nx2 + ilc];
	p22 = x[imz * p + ilq * n + ims * m + ilr * nx2 + imc];
	p23 = x[imz * p + ilq * n + ims * m + imr * nx2 + ilc];
	p24 = x[imz * p + ilq * n + ims * m + imr * nx2 + imc];
	p25 = x[imz * p + imq * n + ils * m + ilr * nx2 + ilc];
	p26 = x[imz * p + imq * n + ils * m + ilr * nx2 + imc];
	p27 = x[imz * p + imq * n + ils * m + imr * nx2 + ilc];
	p28 = x[imz * p + imq * n + ils * m + imr * nx2 + imc];
	p29 = x[imz * p + imq * n + ims * m + ilr * nx2 + ilc];
	p30 = x[imz * p + imq * n + ims * m + ilr * nx2 + imc];
	p31 = x[imz * p + imq * n + ims * m + imr * nx2 + ilc];
	p32 = x[imz * p + imq * n + ims * m + imr * nx2 + imc];
 
    // interpolations in xi1 direction
	xp1  = dr * ( p3 - p1) + p1;
	xp2  = dr * ( p4 - p2) + p2;
	xp3  = dr * ( p7 - p5) + p5;
	xp4  = dr * ( p8 - p6) + p6;
        xp5  = dr * ( p11 - p9) + p9;
        xp6  = dr * ( p12 - p10) + p10;
        xp7  = dr * ( p15 - p13) + p13;
        xp8  = dr * ( p16 - p14) + p14;
        xp9  = dr * ( p19 - p17) + p17;
        xp10 = dr * ( p20 - p18) + p18;
        xp11 = dr * ( p23 - p21) + p21;
        xp12 = dr * ( p24 - p22) + p22;
        xp13 = dr * ( p27 - p25) + p25;
        xp14 = dr * ( p28 - p26) + p26;
        xp15 = dr * ( p31 - p29) + p29;
        xp16 = dr * ( p32 - p30) + p30;
    // interpolations in xi2 direction
	xr1  = dc * ( xp2 - xp1) + xp1;
	xr2  = dc * ( xp4 - xp3) + xp3;
	xr3  = dc * ( xp6 - xp5) + xp5;
	xr4  = dc * ( xp8 - xp7) + xp7;
	xr5  = dc * ( xp10 - xp9) + xp9;
	xr6  = dc * ( xp12 - xp11) + xp11;
	xr7  = dc * ( xp14 - xp13) + xp13;
	xr8  = dc * ( xp16 - xp15) + xp15;
    // interpolations in xi3 direction
        xs1  = ds * (xr2 - xr1) + xr1;
        xs2  = ds * (xr4 - xr3) + xr3;
        xs3  = ds * (xr6 - xr5) + xr5;
        xs4  = ds * (xr8 - xr7) + xr7;
    // interpolations in xi4 direction
        xq1  = dq * (xs2 - xs1) + xs1;
        xq2  = dq * (xs4 - xs3) + xs3;
    // interpolation in xi5 direction
	return dz * ( xq2 - xq1) + xq1;
}

void Table5::read( const char *tabname, bool echo) {
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
      nx5 = atoi( vs[4].c_str());
      // x dimension independant values
      x1 = new double[nx1];
      x2 = new double[nx2];
      x3 = new double[nx3];
      x4 = new double[nx4];
      x5 = new double[nx5];
      y  = new double[ nx1 * nx2 * nx3 * nx4 * nx5]; // space for all dimensions
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
      // dimension 5 name & independant values
      vs = f->str_split( vlines[++i]);
      x5name = vs[0];
        
      vs = f->str_split( vlines[++i]);
      for( int j = 0; j < nx5; j++) {
        x5[j] = atof( vs[j].c_str());
      }
      // dependant variable name
      vs = f->str_split( vlines[++i]);
      yname = vs[0];
      // read table data  
      for ( int l = 0; l < nx1; l++ ) { 
        for( int n = 0; n < nx2; n++) {  
          for( int m = 0; m < nx3; m++) {
            for( int j = 0; j < nx4; j++) { 
              // skip empty lines 
              do {
                vs = f->str_split( vlines[++i]);
              } while( vs[0] == "");
              // read line data
              for( int k = 0; k < nx5; k++) {
                y[k * (nx1 * nx2 * nx3 * nx4) + j * ( nx1 * nx2 * nx3) + m * (nx1 * nx2) + l * nx2 + n] = atof( vs[k].c_str());
              } // end for
            } // end j
          } // end m
        } // end n
      } //end l

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

ostream &operator<<( ostream &stream, Table5 t) {
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

