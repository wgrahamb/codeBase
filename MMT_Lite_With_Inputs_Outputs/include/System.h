//----------------------------------------------------------------//
// File: System.h
// 
// This class is used to fascilate the use of system level simulation 
// variables such as time and mode.
//
// Developer: Dennis Strickland
//----------------------------------------------------------------//

#ifndef SYSTEM_H
#define SYSTEM_H
#include "main.h"

class Output;
class MathUtil;
class NavProc;

class System : public Block{

  public:
    int kt, flg1, brk_Flag;
    int hwil_fire, launch;
    bool prelaunch;
    double t1000, xd, pretime;
    double initTime, preLnchTime, readyTime, fireDelay;
    double last_sim_dt, last_t;
    double t_sys, t_free, t_bo;
    double tick, tlnch, tlnch_abs, sys_dt, dtHz, tmax;
    double t_break, t_flight;
    long  iseed;
    int   rtHwilFlag;
    int   CasFidelityFlag;
    int   FinCommandDelay;
    bool  TraceEvents;
    bool  TraceTiming;
    double TraceTimingStart;
    double TraceTimingDuration;

    System( string infile, Output *outp, MathUtil *mutilp, NavProc *navprocp);

    double phi0, tht0, psi0;
    double latg0, lon0, altg0, azRef0, rng0;

    int errLnch_Flag;
    double phiLnch_sig, thtLnch_sig, psiLnch_sig;

    int errLoc_Flag;
    double latLoc_sig, lonLoc_sig, altLoc_sig;

    int mode;

    void reAssignPtr(MathUtil *mutilp, NavProc *navprocp);
    void traceEvent(const char* msg, double time=-1.0);
    void traceTiming(const char* msg, double time=-1.0);
    bool isTracing();
      void init();
      void update();
      void rpt();
    protected:


    private:
      Output   *out;
      MathUtil *mutil;
      NavProc *navproc;
};

#endif
