// v130802
#include "table1.h"
#include <cstdlib>
#include "Output.h"

extern Output *out;

namespace tframes {

Table1::Table1( const char *fname) : Table( fname) {
  //cout << "Table1::Table1()" << endl;
}

#ifdef SIXDOF
int Table1::test( int n) {
  if( !tabRead) {
    cout << "Error - " << tabname << " table not read\n";
    return 0;
  }
  int passed = 1;
  if( strcmp( fname, "table.dat") != 0) {
    cout << "Error:" << endl;
    cout << "Table1() must be instantiated with file \"table.dat\"" << endl;
    cout <<  " for Table1()->test to work." << endl;
    return 0;
  }
  // test at points
  for( int i = 0; i < nx1; i++) {
    double x = x1[i];
    double y = x * 1000.0;
    double yi = interp( x);
    if( fabs( yi - y) > 1e-6) {
      passed = 0;
      cout << "failed! " << x << " " << yi << endl;
    }
  }

  // test between points
  for( int i = 0; i < n; i++) {
    double p1 = rand0( -1., 5.);
    double pp1 = limit( p1, 0.0, 4.0);
    double y = pp1 * 1000.0;
    double yi = interp( p1);
    if( fabs( yi - y) > 1e-6) {
      passed = 0;
      cout << "failed! " << p1 << " " << yi << endl;
    }
  }
  return passed;
}
#endif

double Table1::interp( double xi) {
/* One-dimensional linear interpolation. */
  if( !tabRead) {
	#ifdef SIXDOF
    cout << "Error - " << tabname << " table not read\n";
	#endif
    return 0.0;
  }
	int il, im;
	double d;
	double *x;

	binsearch( xi, x1, nx1, &il, &im, &d);
	return d * ( y[im] - y[il]) + y[il];
}

#ifdef SIXDOF
/*
void Table1::read_db( char *tabname, DataBox *db, bool echo) {
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
      x1 = db->new_dim( nx1);
      y = db->new_dim( nx1);
      if( !y) {
        cout << "DataBox:  no free space" << endl;
        exit( 1);
      }
      vs = f->str_split( vlines[++i]);
      x1name = vs[0];
      yname = vs[1];
      for( int j = 0; j < nx1; j++) {
        vs = f->str_split( vlines[++i]);
        x1[j] = atof( vs[0].c_str());
        y[j] = atof( vs[1].c_str());
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
*/

void Table1::read( const char *tabname, bool echo) {
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
      x1 = new double[nx1];
      y = new double[nx1];
      vs = f->str_split( vlines[++i]);
      x1name = vs[0];
      yname = vs[1];
      for( int j = 0; j < nx1; j++) {
        vs = f->str_split( vlines[++i]);
        x1[j] = atof( vs[0].c_str());
        y[j] = atof( vs[1].c_str());
      }
      tabRead = 1;
      break;
    }
  }
    //  cout << "Table1 tabname: " << tabname << endl;
  if( !tabnameFound) {
    cout << "Error - " << tabname << " not found." << endl;
    return;
  }
  if( echo) {
    cout << *this;
  }
}

ostream &operator<<( ostream &stream, Table1 t) {
  
  if(out->displayInput == 1) {
    stream << t.tabname << endl;
    stream << t.nx1 << endl;
    stream << t.x1name << " " << t.yname << endl;
    for( int j = 0; j < t.nx1; j++) {
      stream << t.x1[j] << " " << t.y[j] << endl;
    }
  }
  return stream;
}
#endif
}
