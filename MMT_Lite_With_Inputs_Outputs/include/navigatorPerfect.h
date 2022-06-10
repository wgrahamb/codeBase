#ifndef PERFECTNAVIGATOR_H
#define PERFECTNAVIGATOR_H

#include "main.h"

class Output;
class System;

class navigatorPerfect : public Block
{
	public:

	// CONSTRUCTOR
	navigatorPerfect(
		string inFile,
		Output *outp,
		System *sysp
	);

	// FUNCTIONS
	void init();
	void update(Vec pm_in, Vec vm_in, Vec euler_in, Vec bodySpecificForce_in, Vec omega_in, double tof);

	// POINTERS
	Output *out;
	System *sys;

	// INPUT
	double sdt;

	// OUTPUT
	int nav_valid;
	bool nav_solution_available;
	Vec pm, vm, euler, sf_b, omegaB;
	double tnav;

};

#endif