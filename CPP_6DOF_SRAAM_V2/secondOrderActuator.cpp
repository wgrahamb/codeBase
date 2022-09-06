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

secondOrderActuator::secondOrderActuator(string logFilePath)
{

	logFile.open(logFilePath);
	logFile << "time command deflection\n";

	time = 0.0;

	deflectionLimit = 10.0;
	deflectionRateLimit = 25.0;
	wn = 57.2958 * 0.1;
	zeta = 0.8;

	deflection = 0.0;
	deflectionDerivative = 0.0;
	deflectionDot = 0.0;
	deflectionDotDerivative = 0.0;

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
	logFile << setprecision(10) << fixed << time << " " << finCommand << " " << deflection << endl;

	return deflection;

}