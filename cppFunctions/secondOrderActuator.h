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

#ifndef SECONDORDERACTUATOR_H
#define SECONDORDERACTUATOR_H

class secondOrderActuator
{

	public:

	// Methods.
	secondOrderActuator(string logFilePath);
	double update(double finCommand, double timeStep);

	// Log file.
	ofstream logFile;

	// Base variables.
	double time; // Seconds.
	double timeStep; // Seconds.

	// Class variables.
	double deflectionLimit; // Degrees.
	double deflectionRateLimit; // Degrees per second.
	double wn; // Degrees per second.
	double zeta; // Non dimensional.

	// State.
	double deflection; // Degrees.
	double deflectionDerivative; // Degrees per second.
	double deflectionDot; // Degrees per second.
	double deflectionDotDerivative; // Degrees per second squared.

};

#endif