// Standard.
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <math.h>
#include <vector>
#include <map>
#include <algorithm>

// Namespace.
using namespace std;

// Utility.
#include "util.h"

// Header.
#include "secondOrderActuator.h"

secondOrderActuator::secondOrderActuator()
{

	time = 0.0;

	deflectionLimit = 28.0;
	deflectionRateLimit = 250;
	wn = 120;
	zeta = 0.7;

	deflection = 0.0;
	deflectionDerivative = 0.0;
	deflectionDot = 0.0;
	deflectionDotDerivative = 0.0;

	// cout << "Second Order Actuator Loaded.\n";

}

double secondOrderActuator::update(double finCommand, double timeStep)
{

	double temp;

	double deflectionDerivativeNew = deflectionDot;
	temp = signum(deflectionDerivativeNew);
	if (fabs(deflectionDerivativeNew) > deflectionRateLimit)
	{

		deflectionDerivativeNew = deflectionRateLimit * temp;

	}

	deflection = trapezoidIntegrate(deflectionDerivativeNew, deflectionDerivative, deflection, timeStep);
	temp = signum(deflection);
	if (fabs(deflection) > deflectionLimit)
	{

		deflection = deflectionLimit * temp;

	}
	deflectionDerivative = deflectionDerivativeNew;

	double edx = finCommand - deflection;
	double deflectionDotDerivativeNew =  wn * wn * edx - 2 * zeta * wn * deflectionDerivative;
	deflectionDot = trapezoidIntegrate(deflectionDotDerivativeNew, deflectionDotDerivative, deflectionDot, timeStep);
	deflectionDotDerivative = deflectionDotDerivativeNew;

	time += timeStep;

	return deflection;

}