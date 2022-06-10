#include "iostream"
#include "iomanip"
#include "fstream"
#include "math.h"
#include "algorithm"
#include "map"
#include "random"
#include "chrono"

#include "util.h"

using namespace std;

// SIM CONTROL
auto wallClockStart = chrono::high_resolution_clock::now(); // Seconds.
bool go = true; // For loop control.
const double CONSTANT_TIME_STEP = 0.001; // Set time step.
double variableTimeStep = CONSTANT_TIME_STEP; // Necessary for some integration methods.
double maxTime = 400.0;

// TARGET
double pip[3] = {4000.0, 0.0, 3000.0}; // Meters.

// NAVIGATION STATE
NAV_STATE navigationState; // Initialized in header.

// FOR RK4 INTEGRATION
int PASS = 0;
double P1[3], V1[3], A1[3], E1[3], EDOT1[3], W1[3], WDOT1[3];
double P2[3], V2[3], A2[3], E2[3], EDOT2[3], W2[3], WDOT2[3];
double P3[3], V3[3], A3[3], E3[3], EDOT3[3], W3[3], WDOT3[3];
double P4[3], V4[3], A4[3], E4[3], EDOT4[3], W4[3], WDOT4[3];

void engine()
{

}

void rk4Integrate(

	double missileENUAccelerationX,
	double missileENUAccelerationY,
	double missileENUAccelerationZ,
	double missileBodyRateDotX,
	double missileBodyRateDotY,
	double missileBodyRateDotZ

)
{

	navigationState.missileENUAcceleration[0] = missileENUAccelerationX;
	navigationState.missileENUAcceleration[1] = missileENUAccelerationY;
	navigationState.missileENUAcceleration[2] = missileENUAccelerationZ;
	navigationState.missileBodyRateDot[0] = missileBodyRateDotX;
	navigationState.missileBodyRateDot[1] = missileBodyRateDotY;
	navigationState.missileBodyRateDot[2] = missileBodyRateDotZ;

	if (PASS == 0)
	{

		P1[0] = navigationState.missileENUPosition[0];
		P1[1] = navigationState.missileENUPosition[1];
		P1[2] = navigationState.missileENUPosition[2];

		V1[0] = navigationState.missileENUVelocity[0];
		V1[1] = navigationState.missileENUVelocity[1];
		V1[2] = navigationState.missileENUVelocity[2];

		E1[0] = navigationState.missileENUEulerAngles[0];
		E1[1] = navigationState.missileENUEulerAngles[1];
		E1[2] = navigationState.missileENUEulerAngles[2];

		W1[0] = navigationState.missileBodyRate[0];
		W1[1] = navigationState.missileBodyRate[1];
		W1[2] = navigationState.missileBodyRate[2];

		navigationState.missileENUPosition[0] = P1[0];
		navigationState.missileENUPosition[1] = P1[1];
		navigationState.missileENUPosition[2] = P1[2];

		navigationState.missileENUVelocity[0] = V1[0];
		navigationState.missileENUVelocity[1] = V1[1];
		navigationState.missileENUVelocity[2] = V1[2];

		navigationState.missileENUEulerAngles[0] = E1[0];
		navigationState.missileENUEulerAngles[1] = E1[1];
		navigationState.missileENUEulerAngles[2] = E1[2];

		navigationState.missileBodyRate[0] = W1[0];
		navigationState.missileBodyRate[1] = W1[1];
		navigationState.missileBodyRate[2] = W1[2];

		PASS += 1;
		variableTimeStep = CONSTANT_TIME_STEP / 2.0;
		navigationState.missileTimeOfFlight += variableTimeStep;

	}

	else if (PASS == 1)
	{

		A1[0] = navigationState.missileENUAcceleration[0];
		A1[1] = navigationState.missileENUAcceleration[1];
		A1[2] = navigationState.missileENUAcceleration[2];

		WDOT1[0] = navigationState.missileBodyRateDot[0];
		WDOT1[1] = navigationState.missileBodyRateDot[1];
		WDOT1[2] = navigationState.missileBodyRateDot[2];

		EDOT1[0] = navigationState.missileBodyRate[0] + (navigationState.missileBodyRate[1] * sin(navigationState.missileENUEulerAngles[0]) + navigationState.missileBodyRate[2] * cos(navigationState.missileENUEulerAngles[0])) * tan(navigationState.missileENUEulerAngles[1]);
		EDOT1[1] = navigationState.missileBodyRate[1] * cos(navigationState.missileENUEulerAngles[0]) - navigationState.missileBodyRate[2] * sin(navigationState.missileENUEulerAngles[0]);
		EDOT1[2] = -1 * (navigationState.missileBodyRate[1] * sin(navigationState.missileENUEulerAngles[0]) + navigationState.missileBodyRate[2] * cos(navigationState.missileENUEulerAngles[0])) / cos(navigationState.missileENUEulerAngles[1]);

		P2[0] = P1[0] + V1[0] * variableTimeStep;
		P2[1] = P1[1] + V1[1] * variableTimeStep;
		P2[2] = P1[2] + V1[2] * variableTimeStep;

		V2[0] = V1[0] + A1[0] * variableTimeStep;
		V2[1] = V1[1] + A1[1] * variableTimeStep;
		V2[2] = V1[2] + A1[2] * variableTimeStep;

		E2[0] = E1[0] + EDOT1[0] * variableTimeStep;
		E2[1] = E1[1] + EDOT1[1] * variableTimeStep;
		E2[2] = E1[2] + EDOT1[2] * variableTimeStep;

		W2[0] = W1[0] + WDOT1[0] * variableTimeStep;
		W2[1] = W1[1] + WDOT1[1] * variableTimeStep;
		W2[2] = W1[2] + WDOT1[2] * variableTimeStep;

		navigationState.missileENUPosition[0] = P2[0];
		navigationState.missileENUPosition[1] = P2[1];
		navigationState.missileENUPosition[2] = P2[2];

		navigationState.missileENUVelocity[0] = V2[0];
		navigationState.missileENUVelocity[1] = V2[1];
		navigationState.missileENUVelocity[2] = V2[2];

		navigationState.missileENUEulerAngles[0] = E2[0];
		navigationState.missileENUEulerAngles[1] = E2[1];
		navigationState.missileENUEulerAngles[2] = E2[2];

		navigationState.missileBodyRate[0] = W2[0];
		navigationState.missileBodyRate[1] = W2[1];
		navigationState.missileBodyRate[2] = W2[2];

		PASS += 1;
		variableTimeStep = CONSTANT_TIME_STEP / 2.0;

	}

	else if (PASS == 2)
	{

		A2[0] = navigationState.missileENUAcceleration[0];
		A2[1] = navigationState.missileENUAcceleration[1];
		A2[2] = navigationState.missileENUAcceleration[2];

		WDOT2[0] = navigationState.missileBodyRateDot[0];
		WDOT2[1] = navigationState.missileBodyRateDot[1];
		WDOT2[2] = navigationState.missileBodyRateDot[2];

		EDOT2[0] = navigationState.missileBodyRate[0] + (navigationState.missileBodyRate[1] * sin(navigationState.missileENUEulerAngles[0]) + navigationState.missileBodyRate[2] * cos(navigationState.missileENUEulerAngles[0])) * tan(navigationState.missileENUEulerAngles[1]);
		EDOT2[1] = navigationState.missileBodyRate[1] * cos(navigationState.missileENUEulerAngles[0]) - navigationState.missileBodyRate[2] * sin(navigationState.missileENUEulerAngles[0]);
		EDOT2[2] = -1 * (navigationState.missileBodyRate[1] * sin(navigationState.missileENUEulerAngles[0]) + navigationState.missileBodyRate[2] * cos(navigationState.missileENUEulerAngles[0])) / cos(navigationState.missileENUEulerAngles[1]);

		P3[0] = P1[0] + V2[0] * variableTimeStep;
		P3[1] = P1[1] + V2[1] * variableTimeStep;
		P3[2] = P1[2] + V2[2] * variableTimeStep;

		V3[0] = V1[0] + A2[0] * variableTimeStep;
		V3[1] = V1[1] + A2[1] * variableTimeStep;
		V3[2] = V1[2] + A2[2] * variableTimeStep;

		E3[0] = E1[0] + EDOT2[0] * variableTimeStep;
		E3[1] = E1[1] + EDOT2[1] * variableTimeStep;
		E3[2] = E1[2] + EDOT2[2] * variableTimeStep;

		W3[0] = W1[0] + WDOT2[0] * variableTimeStep;
		W3[1] = W1[1] + WDOT2[1] * variableTimeStep;
		W3[2] = W1[2] + WDOT2[2] * variableTimeStep;

		navigationState.missileENUPosition[0] = P3[0];
		navigationState.missileENUPosition[1] = P3[1];
		navigationState.missileENUPosition[2] = P3[2];

		navigationState.missileENUVelocity[0] = V3[0];
		navigationState.missileENUVelocity[1] = V3[1];
		navigationState.missileENUVelocity[2] = V3[2];

		navigationState.missileENUEulerAngles[0] = E3[0];
		navigationState.missileENUEulerAngles[1] = E3[1];
		navigationState.missileENUEulerAngles[2] = E3[2];

		navigationState.missileBodyRate[0] = W2[0];
		navigationState.missileBodyRate[1] = W2[1];
		navigationState.missileBodyRate[2] = W2[2];

		PASS += 1;
		navigationState.missileTimeOfFlight += variableTimeStep; // Now caught up to CONSTANT_TIME_STEP.
		variableTimeStep = CONSTANT_TIME_STEP;

	}

	else if (PASS == 3)
	{

		A3[0] = navigationState.missileENUAcceleration[0];
		A3[1] = navigationState.missileENUAcceleration[1];
		A3[2] = navigationState.missileENUAcceleration[2];

		WDOT3[0] = navigationState.missileBodyRateDot[0];
		WDOT3[1] = navigationState.missileBodyRateDot[1];
		WDOT3[2] = navigationState.missileBodyRateDot[2];

		EDOT3[0] = navigationState.missileBodyRate[0] + (navigationState.missileBodyRate[1] * sin(navigationState.missileENUEulerAngles[0]) + navigationState.missileBodyRate[2] * cos(navigationState.missileENUEulerAngles[0])) * tan(navigationState.missileENUEulerAngles[1]);
		EDOT3[1] = navigationState.missileBodyRate[1] * cos(navigationState.missileENUEulerAngles[0]) - navigationState.missileBodyRate[2] * sin(navigationState.missileENUEulerAngles[0]);
		EDOT3[2] = -1 * (navigationState.missileBodyRate[1] * sin(navigationState.missileENUEulerAngles[0]) + navigationState.missileBodyRate[2] * cos(navigationState.missileENUEulerAngles[0])) / cos(navigationState.missileENUEulerAngles[1]);

		P4[0] = P1[0] + V3[0] * variableTimeStep;
		P4[1] = P1[1] + V3[1] * variableTimeStep;
		P4[2] = P1[2] + V3[2] * variableTimeStep;

		V4[0] = V1[0] + A3[0] * variableTimeStep;
		V4[1] = V1[1] + A3[1] * variableTimeStep;
		V4[2] = V1[2] + A3[2] * variableTimeStep;

		E4[0] = E1[0] + EDOT3[0] * variableTimeStep;
		E4[1] = E1[1] + EDOT3[1] * variableTimeStep;
		E4[2] = E1[2] + EDOT3[2] * variableTimeStep;

		W4[0] = W1[0] + WDOT3[0] * variableTimeStep;
		W4[1] = W1[1] + WDOT3[1] * variableTimeStep;
		W4[2] = W1[2] + WDOT3[2] * variableTimeStep;

		navigationState.missileENUPosition[0] = P4[0];
		navigationState.missileENUPosition[1] = P4[1];
		navigationState.missileENUPosition[2] = P4[2];

		navigationState.missileENUVelocity[0] = V4[0];
		navigationState.missileENUVelocity[1] = V4[1];
		navigationState.missileENUVelocity[2] = V4[2];

		navigationState.missileENUEulerAngles[0] = E4[0];
		navigationState.missileENUEulerAngles[1] = E4[1];
		navigationState.missileENUEulerAngles[2] = E4[2];

		navigationState.missileBodyRate[0] = W4[0];
		navigationState.missileBodyRate[1] = W4[1];
		navigationState.missileBodyRate[2] = W4[2];

		PASS += 1;
		variableTimeStep = CONSTANT_TIME_STEP / 6.0;

	}

	else if (PASS == 4)
	{

		A4[0] = navigationState.missileENUAcceleration[0];
		A4[1] = navigationState.missileENUAcceleration[1];
		A4[2] = navigationState.missileENUAcceleration[2];

		WDOT4[0] = navigationState.missileBodyRateDot[0];
		WDOT4[1] = navigationState.missileBodyRateDot[1];
		WDOT4[2] = navigationState.missileBodyRateDot[2];

		EDOT4[0] = navigationState.missileBodyRate[0] + (navigationState.missileBodyRate[1] * sin(navigationState.missileENUEulerAngles[0]) + navigationState.missileBodyRate[2] * cos(navigationState.missileENUEulerAngles[0])) * tan(navigationState.missileENUEulerAngles[1]);
		EDOT4[1] = navigationState.missileBodyRate[1] * cos(navigationState.missileENUEulerAngles[0]) - navigationState.missileBodyRate[2] * sin(navigationState.missileENUEulerAngles[0]);
		EDOT4[2] = -1 * (navigationState.missileBodyRate[1] * sin(navigationState.missileENUEulerAngles[0]) + navigationState.missileBodyRate[2] * cos(navigationState.missileENUEulerAngles[0])) / cos(navigationState.missileENUEulerAngles[1]);

		navigationState.missileENUPosition[0] = P1[0] + (V1[0] + 2 * V2[0] + 2 * V3[0] + V4[0]) * variableTimeStep;
		navigationState.missileENUPosition[1] = P1[1] + (V1[1] + 2 * V2[1] + 2 * V3[1] + V4[1]) * variableTimeStep;
		navigationState.missileENUPosition[2] = P1[2] + (V1[2] + 2 * V2[2] + 2 * V3[2] + V4[2]) * variableTimeStep;

		navigationState.missileENUVelocity[0] = V1[0] + (A1[0] + 2 * A2[0] + 2 * A3[0] + A4[0]) * variableTimeStep;
		navigationState.missileENUVelocity[1] = V1[1] + (A1[1] + 2 * A2[1] + 2 * A3[1] + A4[1]) * variableTimeStep;
		navigationState.missileENUVelocity[2] = V1[2] + (A1[2] + 2 * A2[2] + 2 * A3[2] + A4[2]) * variableTimeStep;

		navigationState.missileENUEulerAngles[0] = E1[0] + (EDOT1[0] + 2 * EDOT2[0] + 2 * EDOT3[0] + EDOT4[0]) * variableTimeStep;
		navigationState.missileENUEulerAngles[1] = E1[1] + (EDOT1[1] + 2 * EDOT2[1] + 2 * EDOT3[1] + EDOT4[1]) * variableTimeStep;
		navigationState.missileENUEulerAngles[2] = E1[2] + (EDOT1[2] + 2 * EDOT2[2] + 2 * EDOT3[2] + EDOT4[2]) * variableTimeStep;

		navigationState.missileBodyRate[0] = W1[0] + (WDOT1[0] + 2 * WDOT2[0] + 2 * WDOT3[0] + WDOT4[0]) * variableTimeStep;
		navigationState.missileBodyRate[1] = W1[1] + (WDOT1[1] + 2 * WDOT2[1] + 2 * WDOT3[1] + WDOT4[1]) * variableTimeStep;
		navigationState.missileBodyRate[2] = W1[2] + (WDOT1[2] + 2 * WDOT2[2] + 2 * WDOT3[2] + WDOT4[2]) * variableTimeStep;

		// Reset variables.
		PASS = 0;
		variableTimeStep = CONSTANT_TIME_STEP;

		P1[0] = 0.0;
		P1[1] = 0.0;
		P1[2] = 0.0;
		V1[0] = 0.0;
		V1[1] = 0.0;
		V1[2] = 0.0;
		A1[0] = 0.0;
		A1[1] = 0.0;
		A1[2] = 0.0;
		E1[0] = 0.0;
		E1[1] = 0.0;
		E1[2] = 0.0;
		EDOT1[0] = 0.0;
		EDOT1[1] = 0.0;
		EDOT1[2] = 0.0;
		W1[0] = 0.0;
		W1[1] = 0.0;
		W1[2] = 0.0;
		WDOT1[0] = 0.0;
		WDOT1[1] = 0.0;
		WDOT1[2] = 0.0;

		P2[0] = 0.0;
		P2[1] = 0.0;
		P2[2] = 0.0;
		V2[0] = 0.0;
		V2[1] = 0.0;
		V2[2] = 0.0;
		A2[0] = 0.0;
		A2[1] = 0.0;
		A2[2] = 0.0;
		E2[0] = 0.0;
		E2[1] = 0.0;
		E2[2] = 0.0;
		EDOT2[0] = 0.0;
		EDOT2[1] = 0.0;
		EDOT2[2] = 0.0;
		W2[0] = 0.0;
		W2[1] = 0.0;
		W2[2] = 0.0;
		WDOT2[0] = 0.0;
		WDOT2[1] = 0.0;
		WDOT2[2] = 0.0;

		P3[0] = 0.0;
		P3[1] = 0.0;
		P3[2] = 0.0;
		V3[0] = 0.0;
		V3[1] = 0.0;
		V3[2] = 0.0;
		A3[0] = 0.0;
		A3[1] = 0.0;
		A3[2] = 0.0;
		E3[0] = 0.0;
		E3[1] = 0.0;
		E3[2] = 0.0;
		EDOT3[0] = 0.0;
		EDOT3[1] = 0.0;
		EDOT3[2] = 0.0;
		W3[0] = 0.0;
		W3[1] = 0.0;
		W3[2] = 0.0;
		WDOT3[0] = 0.0;
		WDOT3[1] = 0.0;
		WDOT3[2] = 0.0;

		P4[0] = 0.0;
		P4[1] = 0.0;
		P4[2] = 0.0;
		V4[0] = 0.0;
		V4[1] = 0.0;
		V4[2] = 0.0;
		A4[0] = 0.0;
		A4[1] = 0.0;
		A4[2] = 0.0;
		E4[0] = 0.0;
		E4[1] = 0.0;
		E4[2] = 0.0;
		EDOT4[0] = 0.0;
		EDOT4[1] = 0.0;
		EDOT4[2] = 0.0;
		W4[0] = 0.0;
		W4[1] = 0.0;
		W4[2] = 0.0;
		WDOT4[0] = 0.0;
		WDOT4[1] = 0.0;
		WDOT4[2] = 0.0;

	}

}

int main()
{

	cout << "HOWDY" << endl;
	return 0;

}