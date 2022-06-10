//----------------------------------------------------------------//
// File: Util.h
// 
// This class is used to fascilate the use of user-defined error
// statistics within the simulation and Monte-Carlo performance
// analysis.
//
// Developer:         Dennis Strickland
//----------------------------------------------------------------//

#ifndef UTIL_H
#define UTIL_H
#include "main.h"

class System;

class MathUtil : public Block { 

public:

  MathUtil(System *sysp);
  ~MathUtil();

  //Monte-Carlo Control Structure
  typedef struct {
    long seed;
    bool restartFlag;
    double mu;
    double xll;
    double xuu;
    int  uclamp;
  } mc_str;
  //
  mc_str mctl;

  void   genSeeds(int numSeeds);
  void   readSeeds();
  vector<long> vecSeeds;

  double unif(bool zero_for_nominal=false);
  double gaus(double xsigma, bool zero_for_nominal=false);
  double unif_r(string descr, bool disable=false,
                bool report_zero=false);
  double gaus_r(double xsigma, string descr, bool disable=false,
                bool report_zero=false);
  double rnfst(long *ix);
  void   setInitSeed(int runNum);
  //
  double atan2_0( double y, double x);
  double normal( double sig);
  double rss( double x, double y);
  double limit( double x, double xmin, double xmax);
  double to_db( double x);
  double from_db( double x);
  double signum ( double x);
  void pop_unif_r();
  void pop_gaus_r();

private:
  System    *sys;

};

#endif

