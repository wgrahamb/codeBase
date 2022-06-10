// v130802
#ifndef TABLE1_H
#define TABLE1_H
#include "table.h"
#ifdef SIXDOF
#include "strtok.h"
#endif


namespace tframes {

class Table1 : public Table {
  public:
    Table1( const char *fname);
	#ifdef SIXDOF
    void read( const char *tabname, bool echo);
    //void read_db( char *tabname, DataBox *db, bool echo);
	#endif
    double interp( double xi);
	#ifdef SIXDOF
    int test( int n);
	#endif
    double operator()( double x) { return interp( x);};
  private:
	#ifdef SIXDOF
    string x1name, yname;
	#endif
    double *y;
    int nx1;
    double *x1;

	#ifdef SIXDOF
    friend ostream &operator<<( ostream &stream, Table1);
	#endif
};

#ifdef SIXDOF
ostream &operator<<( ostream &stream, Table1);
#endif
}
#endif
