#ifndef ACTUATORS_H
#define ACTUATORS_H

#include "main.h"
#include "queue"
#include "vector"

class Output;
class System;

class actuators : public Block
{

	public:
		
		// CONSTRUCTOR
		actuators(string inFile, Output *outp, System *sysp);
		int fidelityFlag;

		// FUNCTIONS
		void init();
		void update(
			double rollFinCommandDegrees,
			double pitchFinCommandDegrees,
			double yawFinCommandDegrees,
			double angleOfAttack,
			double mach,
			double dynamicPressure,
			double phi
		);
		double integrate(double dy_new, double dy, double y, double intStep);
		double signum(double x);
		double uniform();

		// POINTERS
		Output *out;
		System *sys;

		// VARIABLES
		double defl1, defl2, defl3, defl4;
		double del1c, del2c, del3c, del4c;

		// SECOND ORDER ACTUATOR VARIABLES
		double fn, zeta, del_inc;
		double u1_1, u1_2,  u1_3, u1_4;
		double x1_1, x1d_1, x2_1, x2d_1;
		double x1_2, x1d_2, x2_2, x2d_2;
		double x1_3, x1d_3, x2_3, x2d_3;
		double x1_4, x1d_4, x2_4, x2d_4;
		//
		double wn_1, zeta_1;
		double wn_2, zeta_2;
		double wn_3, zeta_3;
		double wn_4, zeta_4;

};

#endif