//----------------------------------------------------------------//
// File: PreFlight.h
//
// This class is used to execute pre-flight initialization and
// command launch.
//
// Developer: Dennis Strickland
//----------------------------------------------------------------//

#ifndef PREFLIGHT_H
#define PREFLIGHT_H
#include "main.h"

class Output;
class System;
//class MathUtil;

class PreFlight : public Block{

  public:
    int kt;
    double tick;

    PreFlight( Output *outp, System *sysp);

    int power_Flag, init_Flag, preLnch_Flag, ready_Flag, fire_Flag, lnch_Flag;
    double t_fire;
    void init();
    void update();
    void rpt();
  private:
    Output   *out;
    System   *sys;
    //MathUtil *mutil;


  protected:


};

#endif
