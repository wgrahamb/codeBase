//-----------------------------------------------------------//
// File: Output.h
// 
// This class controls the screen/file output of the simulation.
//                                                           
// Developer: Ray Sells                            
// Other Developers: Dennis Strickland
//-----------------------------------------------------------//

#ifndef OUTPUT_H
#define OUTPUT_H

#include <fstream>
#include "block.h"

class System;

struct Entry {
  string name;
  double *value;
  float *valueff;
  int   *valuei;
  bool  *valueb;
  int type;
};

class Output : public Block {
  public:
    Output( string infile, System *sysp );
    //~Output();
    void add( string name, double *value);
    void add( string name, float *value);
    void add( string name, int *value);
    void add( string name, bool *value);
    int displayInput;
    int record, preTime_rec, runFile_rec;
    double pt_console, ptHz, pt;
    vector<Entry> vEntry;
    FILE *fp_MC;
    // FILE *fp_HDR;
    int  firstFlag;
    string outfile;

    int kt;
    double tick;

    void closeFiles();

    void reAssignPtr(System *sysp);
    void init();
    void update();
    void rpt();
    string infile;
    double dt;
  protected:


  private:
    System    *sys;

};

#endif

