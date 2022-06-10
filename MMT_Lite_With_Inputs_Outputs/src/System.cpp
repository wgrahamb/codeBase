//----------------------------------------------------------------//
// File: System.cpp
// 
// This class is used to fascilate the use of system level simulation 
// variables such as time and mode.
//
// Developer: Dennis Strickland
//----------------------------------------------------------------//

#include "System.h"
#include "Output.h"
#include "Util.h"
#include "SimConstants.h"
#include "NavProc.h"

System::System( string infile, Output *outp, MathUtil *mutilp, NavProc *navprocp)
{

	cout << "SYSTEM CONSTRUCTED" << endl;

	// Determine the real-time HWIL mode
	rtHwilFlag = -1;
#ifdef RT_HWIL
	rtHwilFlag = 1;
	cout << "rtHwil: " << rtHwilFlag << endl;
#endif

	//Local access
	out   = outp;
	mutil = mutilp;
	navproc = navprocp;

	//Input
	Filer *ff = new Filer( "./input/"+infile);
	if (rtHwilFlag == -1)
	{
		pretime = ff->getDouble("pretime");
	}
	else
	{
		pretime = -999999;
	}

	initTime = ff->getDouble("initTime");
	preLnchTime = ff->getDouble("preLnchTime");
	readyTime = ff->getDouble("readyTime");
	fireDelay = ff->getDouble("fireDelay");
	dtHz    = ff->getDouble("dtHz");
	tmax    = ff->getDouble("tmax");
	iseed   = ff->getLong("iseed");

	CasFidelityFlag = ff->getInt("HighFidelityCAS");
	FinCommandDelay = 0;

	if (rtHwilFlag == -1) {
			TraceEvents = ff->getInt("TraceEvents");
	} else {
			TraceEvents = true;
	}
	TraceTiming = ff->getInt("TraceTiming");
	TraceTimingStart = ff->getDouble("TraceTimingStart");
	TraceTimingDuration = ff->getDouble("TraceTimingDuration");

	//Scenario Info
	phi0 = ff->getDouble( "phi0");
	tht0 = ff->getDouble( "tht0");
	psi0 = ff->getDouble( "psi0");
	//
	// latg0 now comes from navproc
	// lon0  now comes from navproc
	// altg0 now comes from navproc
	//
	// azRef0 now comes from navproc
	// rng0   now comes from navproc
	//
	errLnch_Flag = ff->getInt( "errLnch_Flag");
	phiLnch_sig  = ff->getDouble( "phiLnch_sig");
	thtLnch_sig  = ff->getDouble( "thtLnch_sig");
	psiLnch_sig  = ff->getDouble( "psiLnch_sig");
	//
	errLoc_Flag  = ff->getInt( "errLoc_Flag");
	latLoc_sig   = ff->getDouble( "latLoc_sig");
	lonLoc_sig   = ff->getDouble( "lonLoc_sig");
	altLoc_sig   = ff->getDouble( "altLoc_sig");
	delete ff;

	//Integration Rate of Simulation
	sys_dt = 1.0/dtHz;

	//Default Launch Time
	tlnch    = 0.0;
	tlnch_abs = 0.0;

	out->add("tof", &t_flight);

}

void System::init() {

	kt   = 0;
	tick = 0.0;
	flg1 = -1;
	hwil_fire = 0;
	launch = 0;
	prelaunch = false;
	brk_Flag = -1;

	// Base initial launcher and position positions on values from navproc
	// to avoid duplication.
	latg0  = navproc->orig_Lat;
	lon0   = navproc->orig_Lon;
	altg0  = navproc->orig_Altg;
	azRef0 = navproc->tgt_Az;
	rng0   = navproc->tgt_Rng;

	last_t = 0.0;
	last_sim_dt = 0.0;
	//System Time
	t_sys = State::t + pretime;
	//
	t_free  = -9.0e6;
	//
	t_bo    =  9.0e6;
	//
	t_break = -9.0e6;

	//Default Launch Time
	tlnch = 0.0;
	tlnch_abs = -pretime;
	
	//Default Flight Time
	t_flight = 0.0;

	//Power-On Mode
	mode = MODE_POWER_ON;

}

void System::update() {
	
	if( State::sample())
	{
		tick = ( double)kt++;
	}
	//
	if( State::sample( State::EVENT, 0.0)) {}

	last_sim_dt = State::t - last_t;
	last_t = State::t;

	// Time used by Simulation
	// (In HWIL, there may be a discontinuity in t_sys just before launch)
	if (!launch)
	{
		t_sys = State::t + pretime;
	}
	else
	{
		t_sys = State::t - tlnch_abs;
	}

	//Time used by Flight Software
	if(brk_Flag == 1)
	{
		t_flight = t_sys - t_break;
	}

}

void System::rpt()
{
	if (State::sample( out->pt_console))
	{
		if (out->displayInput == 1)
		{
			printf( "t=== %8.3f\n", t_sys);
		}
	}
}

//-------------------------------------------------//

void System::reAssignPtr(MathUtil *mutilp, NavProc *navprocp)
{
	//Reassign Class ptr since input class is not declared yet.
	mutil = mutilp;
	navproc = navprocp;
}

void System::traceEvent(const char* msg, double time)
{
	if (time < 0.0) {
			time = State::t;
	}
	if (TraceEvents)
	{
		if (TraceTiming)
		{
			printf("EVENT: %10.6f [%14.6f] %s\n", time,time - TraceTimingStart,msg);
		}
		else
		{
			if (rtHwilFlag == 1 && t_sys < 0.0)
			{
				printf("EVENT: %10.6f ---------------- %s\n", time, msg);
			}
			else
			{
				printf("EVENT: %10.6f [%14.6f] %s\n", time,time - tlnch_abs,msg);
			}
		}
	}
}

void System::traceTiming(const char* msg, double time)
{
	if (time < 0.0)
	{
			time = State::t;
	}

	if (TraceTiming && time >= TraceTimingStart && time <= (TraceTimingStart + TraceTimingDuration))
	{
			printf("TRACE: %10.6f [%14.6f] %s\n",time,time - TraceTimingStart,msg);
	}
}

bool System::isTracing()
{
	return (TraceTiming && State::t >= TraceTimingStart && State::t <= (TraceTimingStart + TraceTimingDuration));
}
