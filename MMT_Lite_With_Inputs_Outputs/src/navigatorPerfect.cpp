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

void navigatorPerfect::update(Vec pm_in, Vec vm_in, Vec euler_in, Vec bodySpecificForce_in, Vec omega_in, double tof)
{
	if (State::sample(sdt))
	{
		{
			nav_solution_available = true;
			tnav = tof;
			pm = pm_in;
			vm = vm_in;
			euler = euler_in;
			sf_b = bodySpecificForce_in;
			omegaB = omega_in;
		}
	}
}