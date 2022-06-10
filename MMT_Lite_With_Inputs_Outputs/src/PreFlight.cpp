//----------------------------------------------------------------//
// File: PreFlight.cpp
// 
// This class is used to execute pre-flight initialization and 
// command launch.
//
// Developer: Dennis Strickland
//----------------------------------------------------------------//

#include "PreFlight.h"
#include "System.h"
#include "Output.h"
#include "SimConstants.h"


PreFlight::PreFlight( Output *outp, System *sysp) {
	cout << "PREFLIGHT CONSTRUCTED" << endl;

	//Local access
	out   = outp;
	sys   = sysp;

	//Input
	//Filer *ff = new Filer( "./input/"+infile);

	//Output
	// out->add("PreFlight::fire_Flag", &fire_Flag);
}

void PreFlight::init() {

	//Initialize Power-On Flag
	power_Flag = -1;

	//Initialize Initialization Flag
	init_Flag = -1;

	//Initialize Pre-Launch Flag
	preLnch_Flag = -1;

	//Initialize Ready Flag
	ready_Flag = -1;

	//Initialize Fire Flag
	fire_Flag = -1;

	//Initialize Launch Flag
	lnch_Flag = -1;

	t_fire = -1.0;

}

void PreFlight::update() {

	if( State::sample()) {
		tick = ( double)kt++;
	}
	//
	if( State::sample( State::EVENT, 0.0)) {
	}

		//Set Power-On Mode
		//
		// do stuff...(check power levels, etc.)
		//
		if( (sys->t_sys <= 0.0) && (power_Flag == -1) ) {
			sys->mode = MODE_POWER_ON;
			power_Flag = 1;
			sys->traceEvent("Power On");
		}

		//Set Initialization Mode
		//
		// do stuff...(load PEEPS, etc.)
		//
		if( (sys->t_sys >= sys->initTime) && (init_Flag == -1) ) {
			sys->mode = MODE_INITIALIZE;
			init_Flag = 1;
		}

		//Set Pre-Launch Mode
		//
		// do stuff... (transfer align, init Navigator filter, etc.)
		//
		if( (sys->t_sys >= sys->preLnchTime) && (preLnch_Flag == -1) ) {
			sys->mode = MODE_PRELAUNCH;
			preLnch_Flag = 1;
			sys->prelaunch = 1;
		}

		//Set Ready-for-Launch Mode
		//
		// Stop static transfer alignment updates
		//
		if( (sys->t_sys >= sys->readyTime) && (ready_Flag == -1) ) {
			sys->mode = MODE_READY;
			ready_Flag = 1;
		}

		//Set Fire Mode
		if (fire_Flag == -1) {
			bool do_fire = false;
			if (sys->rtHwilFlag == -1) {
					if (sys->t_sys >= (0 - sys->fireDelay)) {
							do_fire = true;
				}
			} else {
				if (sys->hwil_fire) {
						do_fire = true;
				}
			}
			if (do_fire) {
					fire_Flag = 1;
					t_fire = State::t;
					sys->traceEvent("Fire Command Received");
			}
		}

		//Set Launch Mode
		//
		// do stuff...
		//
		if (lnch_Flag == -1) {
			bool do_launch = false;
			if (sys->rtHwilFlag == -1)
			{
				if (sys->t_sys >= 0.0)
				{
					do_launch = true;
				}
			}
			else
			{
				if (fire_Flag == 1 && State::t >= t_fire + sys->fireDelay)
				{
					do_launch = true;
				}
			}
			if (do_launch)
			{
				sys->mode  = MODE_LAUNCH;
				sys->tlnch = sys->t_sys;
				sys->tlnch_abs = State::t;
				lnch_Flag = 1;
				sys->launch = true;
				sys->t_sys = 0.0;
			}
		}

		// Fix mode for HWIL (required due to missing prelaunch mode transitions in HWIL)
		if ((lnch_Flag == 1) && (sys->mode < MODE_LAUNCH))
		{
				sys->mode  = MODE_LAUNCH;
		}
}

void PreFlight::rpt() {
	if( State::sample( out->pt_console)) {
	//if( sample()) {
		if(out->displayInput == 1) {
			printf( "t=== %8.3f\n", sys->t_sys);
		}
	}
}

