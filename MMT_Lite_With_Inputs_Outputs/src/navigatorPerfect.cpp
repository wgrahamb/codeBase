#include "iostream"
#include "iomanip"

#include "navigatorPerfect.h"
#include "System.h"
#include "Output.h"

navigatorPerfect::navigatorPerfect(
	string inFile,
	Output *outp,
	System *sysp
)
{

	cout << "TRUE NAVIGATION CONSTRUCTED" << endl;

	out = outp;
	sys = sysp;

	string file = "input/" + inFile;
	Filer ff(file);
	ff.setLine0("navigatorPerfect");
  	sdt = ff.getDouble("sdt");

}

void navigatorPerfect::init()
{

	tnav = 0.0;
	nav_solution_available = false;
	pm = Vec(0.0, 0.0, 0.0);
	vm = Vec(0.0, 0.0, 0.0);
	double phi0 = sys->phi0 * dtr;
	double tht0 = sys->tht0 * dtr;
	double psi0 = sys->psi0 * dtr;
	euler = euler(phi0, tht0, psi0);
	nav_valid = 1;

}

void navigatorPerfect::handleInput(NavigationState const &navigationState)
{

	if (State::sample(sdt))
	{
		nav_solution_available = true;
		tnav = navigationState.missileTimeOfFlight_;
		pm.x = navigationState.missileLTFPosition_[0];
		pm.y = navigationState.missileLTFPosition_[1];
		pm.z = navigationState.missileLTFPosition_[2];
		vm.x = navigationState.missileLTFVelocity_[0];
		vm.y = navigationState.missileLTFVelocity_[1];
		vm.z = navigationState.missileLTFVelocity_[2];
		euler.x = navigationState.missileLTFEulerAngles_[0];
		euler.y = navigationState.missileLTFEulerAngles_[1];
		euler.z = navigationState.missileLTFEulerAngles_[2];
		sf_b.x = navigationState.missileBodyAcceleration_[0];
		sf_b.y = navigationState.missileBodyAcceleration_[1];
		sf_b.z = navigationState.missileBodyAcceleration_[2];
		omegaB.x = navigationState.missileBodyRate_[0];
		omegaB.y = navigationState.missileBodyRate_[1];
		omegaB.z = navigationState.missileBodyRate_[2];
	}

}

void navigatorPerfect::update(){}