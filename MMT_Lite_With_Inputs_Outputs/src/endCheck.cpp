#include "iostream"
#include "iomanip"
#include "math.h"
#include "algorithm"

#include "endCheck.h"
#include "System.h"
#include "Output.h"
#include "NavProc.h"

endCheck::endCheck(System *sysp, Output *outp, NavProc *navprocp)
{
	cout << "END CHECK CONSTRUCTED" << endl;
	sys = sysp;
	out = outp;
	navproc = navprocp;
}

void endCheck::init()
{
	lethality = "FLYING";
}

void endCheck::update()
{
	
	relativeLocalPosToIntercept = navproc->pt - navproc->pm_nav;
	forwardLeftUpRelativeMissileToInterceptPos = navproc->mDCM_nav * relativeLocalPosToIntercept;
	missDistance = forwardLeftUpRelativeMissileToInterceptPos.mag();

	if (navproc->pm_nav.z > 1.0)
	{
		lethality = "GROUND COLLISION";
		Sim::stop = -1;
	}
	else if (missDistance < 5.0)
	{
		lethality = "SUCCESSFUL INTERCEPT";
		Sim::stop = -1;
	}
	else if (forwardLeftUpRelativeMissileToInterceptPos.x < 0.0)
	{
		lethality = "POINT OF CLOSEST APPROACH PASSED";
		Sim::stop = -1;
	}
	else if (isnan(navproc->pm_nav.x))
	{
		lethality = "NOT A NUMBER";
		Sim::stop = -1;
	}
	else if (sys->t_flight > sys->tmax)
	{
		lethality = "MAX TIME EXCEEDED";
		Sim::stop = -1;
	}

}
