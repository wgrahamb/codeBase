#ifndef ENDCHECK_H
#define ENDCHECK_H

#include "main.h"

class System;
class Output;
class NavProc;

class endCheck : public Block
{

	public:

		// CONSTRUCTOR
		endCheck(System *sysp, Output *outp, NavProc *navprocp);

		// FUNCTIONS
		void init();
		void update();

		// POINTERS
		System *sys;
		Output *out;
		NavProc *navproc;

		// VARIABLES
		double lastTime;
		string lethality;
		Vecff relativeLocalPosToIntercept, forwardLeftUpRelativeMissileToInterceptPos;
		double missDistance;

};

#endif