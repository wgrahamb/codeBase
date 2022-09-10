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
#include <memory>

// Namespace.
using namespace std;

// Components.
#include "secondOrderActuator.h"

#ifndef MISSILEMODEL_H
#define MISSILEMODEL_H

/* Missile constants. */
const double REFERENCE_AREA = 0.01824; // Meters^2.
const double REFERENCE_DIAMETER = 0.1524; // Meters.
const double THRUST_EXIT_AREA = 0.0125; // Meters^2.
const double ROCKET_BURN_OUT_TIME = 2.421; // Seconds.
const double SEEKER_KF_G = 10.0; // Seeker Kalman filter gain. Per second.
const double SEEKER_KF_ZETA = 0.9; // Seeker Kalman filter damping. Non dimensional.
const double SEEKER_KF_WN = 60.0; // Seeker Kalman filter natural frequency. Radians per second.
const double PROPORTIONAL_GUIDANCE_GAIN = 3.0; // Guidance homing gain. Non dimensional.
const double MAXIMUM_ACCELERATION = 450.0; // Roughly 45 Gs. Meters per s^2.
const double RATE_CONTROL_ZETA = 0.6; // Damping of constant rate control. Non dimensional.
const double ROLL_CONTROL_WN = 20.0; // Natural frequency of roll closed loop complex pole. Radians per second.
const double ROLL_CONTROL_ZETA = 0.9; // Damping of roll closed loop complex pole. Non dimensional.
const double FIN_RATE_LIMIT_RADIANS = 10.472; // Radians per second.
const double ROLL_ANGLE_COMMAND = 0.0; // Radians.
const double ALPHA_PRIME_MAX = 40.0; // Degrees.
const double SEA_LEVEL_PRESSURE = 101325; // Pascals.
const double LAUNCH_CENTER_OF_GRAVITY_FROM_NOSE = 1.5357; // Meters.

/* This struct fully represents a missile. Can be deep copied without need of defined copy constructor because there are no pointers. */
struct Missile
{

	/* Variables */

	// Target.
	double pip[3]; // Predicted Intercept Point. Meters.

	// Missile overhead.
	bool BALLISTIC = false;
	bool LAUNCHED = false;
	double TIME_STEP = (1.0 / 600.0);
	double HALF_TIME_STEP = TIME_STEP * 0.5;

	// Missile state.
	double timeOfFlight = 0.0; // Seconds.
	double missileENUToFLUMatrix[3][3]; // Non dimensional.
	double ENUPosition[3]; // Meters.
	double range = 0.0; // Meters.
	double ENUVelocity[3]; // Meters per second.
	double FLUVelocity[3]; // Meters per second.
	double speed; // Meters per second.
	double ENUAcceleration[3]; // Meters per second^2.
	double FLUAcceleration[3]; // Meters per second^2.
	double alphaRadians = 0.0; // Radians.
	double betaRadians = 0.0; // Radians.
	double alphaDegrees = 0.0; // Degrees.
	double betaDegrees = 0.0; // Degrees.
	double ENUEulerAngles[3]; // Radians.
	double ENUEulerDot[3] = {0.0, 0.0, 0.0}; // Radians per second.
	double bodyRate[3] = {0.0, 0.0, 0.0}; // Radians per second.
	double bodyRateDot[3] = {0.0, 0.0, 0.0}; // Radians per second^2.

	// Atmosphere.
	double grav = 0.0; // Meters per second^2.
	double FLUGravity[3] = {0.0, 0.0, 0.0}; // Meters per second^2.
	double pressure = 0.0; // Pascals.
	double dynamicPressure = 0.0; // Pascals.
	double machSpeed = 0.0; // Non dimensional.

	// Seeker.
	double seekerPitch; // Radians.
	double seekerYaw; // Radians.
	double seekerENUToFLUMatrix[3][3]; // Non dimensional.
	double seekerPitchError; // Seeker boresight vertical offset from target. Radians.
	double seekerYawError; // Seeker boresight horizontal offset from target. Radians.
	double seekerWLR; // Pointing yaw rate. Radians per second.
	double seekerWLRD = 0.0; // Derivative of pointing yaw rate. Radians per second^2.
	double seekerWLR1 = 0.0; // Yaw sight line spin rate. Radians per second.
	double seekerWLR1D = 0.0; // Derivative of yaw sight line spin rate. Radians per second^2.
	double seekerWLR2 = 0.0; // Second state variable in yawing kalman filter. Radians per second^2.
	double seekerWLR2D = 0.0; // Derivative of second state variable in yawing kalman filter. Radians per second^3.
	double seekerWLQ; // Pointing pitch rate. Radians per second.
	double seekerWLQD = 0.0; // Derivative of pointing pitch rate. Radians per second^2.
	double seekerWLQ1 = 0.0; // Pitch sight line spin rate. Radians per second.
	double seekerWLQ1D = 0.0; // Derivative of pitch sight line spin rate. Radians per second^2.
	double seekerWLQ2 = 0.0; // Second state variable in pitching kalman filter. Radians per second^2.
	double seekerWLQ2D = 0.0; // Derivative of second state variable in pitching kalman filter. Radians per second^3.

	// Guidance.
	bool homing = false;
	double timeToGo = 0.0;
	double FLUMissileToPipRelativePosition[3] = {0.0, 0.0, 0.0}; // Meters.
	double guidanceNormalCommand = 0.0; // Meters per second^2.
	double guidanceSideCommand = 0.0; // Meters per second^2.
	double maneuveringLimit = MAXIMUM_ACCELERATION; // Meters per second^2.

	// Control
	double pitchError = 0.0;
	double pitchErrorDerivative = 0.0;
	double lastYawProportionalError = 0.0; // Radians per second.
	double yawIntegralError = 0.0; // Something.
	double yawProportionalError = 0.0; // Radians per second.
	double yawFinCommand = 0.0; // Radians.
	double lastPitchProportionalError = 0.0; // Radians per second.
	double pitchIntegralError = 0.0; // Something.
	double pitchProportionalError = 0.0; // Radians per second.
	double pitchFinCommand = 0.0; // Radians.
	double lastRollProportionalError = 0.0; // Radians per second.
	double rollProportionalError = 0.0; // Radians per second.
	double rollFinCommand = 0.0; // Radians.

	// Actuators.
	shared_ptr<secondOrderActuator> FIN1 = make_shared<secondOrderActuator>("output/FIN1.txt");
	shared_ptr<secondOrderActuator> FIN2 = make_shared<secondOrderActuator>("output/FIN2.txt");
	shared_ptr<secondOrderActuator> FIN3 = make_shared<secondOrderActuator>("output/FIN3.txt");
	shared_ptr<secondOrderActuator> FIN4 = make_shared<secondOrderActuator>("output/FIN4.txt");
	double FIN1DEFL = 0.0; // Fin deflection. Radians.
	double FIN2DEFL = 0.0; // Fin deflection. Radians.
	double FIN3DEFL = 0.0; // Fin deflection. Radians.
	double FIN4DEFL = 0.0; // Fin deflection. Radians.
	double pitchFinDeflection = 0.0; // Radians.
	double yawFinDeflection = 0.0; // Radians.
	double rollFinDeflection = 0.0; // Radians.

	// Aerodynamic angles and conversions.
	double alphaPrimeRadians = 0.0; // Radians.
	double alphaPrimeDegrees = 0.0; // Degrees.
	double sinPhiPrime = 0.0; // Non dimensional.
	double cosPhiPrime = 0.0; // Non dimensional.
	double pitchAeroBallisticFinDeflectionDegrees = 0.0; // Degrees.
	double yawAeroBallisticFinDeflectionDegrees = 0.0; // Degrees.
	double rollFinDeflectionDegrees = 0.0; // Degrees.
	double totalFinDeflectionDegrees = 0.0; // Degrees.
	double pitchAeroBallisticBodyRateDegrees = 0.0; // Degrees per second.
	double yawAeroBallisticBodyRateDegrees = 0.0; // Degrees per second
	double rollRateDegrees = 0.0; // Degrees per second.
	double sinOfFourTimesPhiPrime = 0.0; // Non dimensional.
	double squaredSinOfTwoTimesPhiPrime = 0.0; // Non dimensional.

	// Table look ups.
	map<string, int> tableNameIndexPairs;
	vector<vector<vector<double>>> tables;

	// Aerodynamics.
	double CA0 = 0.0; // Axial force coefficient. Non dimensional.
	double CAA = 0.0; // Axial force derivative of alpha prime. Per degree.
	double CAD = 0.0; // Axial force derivative of control fin deflection. Per degree^2.
	double CA_POWER_CORRECTION = 0.0; // Power off correction term for axial force coefficient. Non dimensional.
	double CYP = 0.0; // Side force coefficient correction term for when phi is non zero. Non dimensional.
	double CYDR = 0.0; // Side force derivative of elevator. Per degree.
	double CN0 = 0.0; // Normal force coefficient. Non dimensional.
	double CNP = 0.0; // Correction to normal force coefficient term for when phi is non zero. Non dimensional.
	double CNDQ = 0.0; // Normal force derivative of elevator. Per degree.
	double CLLAP = 0.0; // Roll moment derivative for (alpha prime^2) for when phi is non zero. Per degree^2
	double CLLP = 0.0; // Roll moment damping derivative. Degrees.
	double CLLDP = 0.0; // Roll moment derivative of aileron. Per degree.
	double CLM0 = 0.0; // Pitching moment coefficient at launch center of gravity. Non dimensional.
	double CLMP = 0.0; // Correction to pitching moment coefficient for when phi is non zero. Non dimensional.
	double CLMQ = 0.0; // Pitching moment damping derivative. Per degree.
	double CLMDQ = 0.0; // Pitching moment derivative of elevator. Per degree.
	double CLNP = 0.0; // Yaw moment coefficient correction for when phi is non zero. Non dimensional.

	// Mass and motor properties.
	double mass = 0.0; // Kilograms.
	double unadjustedThrust = 0.0; // Newtons.
	double transverseMomentOfInertia = 0.0; // Kilograms * meters^2.
	double axialMomentOfInertia = 0.0; // Kilograms * meters^2.
	double centerOfGravityFromNose = 0.0; // Meters.

	// Propulsion.
	double thrust = 0.0; // Newtons.

	// Aerodynamic coefficients.
	double CX = 0.0; // Non dimensional.
	double CY = 0.0; // Non dimensional.
	double CZ = 0.0; // Non dimensional.
	double CL = 0.0; // Non dimensional.
	double CM = 0.0; // Non dimensional.
	double CN = 0.0; // Non dimensional.

	// Aerodynamic derivatives.
	double CNA = 0.0; // Per degree.
	double CMA = 0.0; // Per degree.
	double CND = 0.0; // Per degree.
	double CMD = 0.0; // Per degree.
	double CMQ = 0.0; // Per degree.
	double CLP = 0.0; // Per degree.
	double CLD = 0.0; // Per degree.
	double staticMargin = 0.0; // Non dimensional.

	// Performance and termination check.
	double missDistance = 0.0; // Meters.
	string lethality;

	// Integration states.
	// P = ENUPosition
	// V = ENUVelocity
	// A = ENUAcceleration
	// E = ENUEulerAngles
	// ED = ENUEulerDot
	// W = BodyRate
	// WD = BodyRateDot.
	int INTEGRATION_METHOD = 2;
	int INTEGRATION_PASS = 0;

	double P0[3] = {0.0, 0.0, 0.0};
	double V0[3] = {0.0, 0.0, 0.0};
	double E0[3] = {0.0, 0.0, 0.0};
	double W0[3] = {0.0, 0.0, 0.0};

	double P1[3] = {0.0, 0.0, 0.0};
	double V1[3] = {0.0, 0.0, 0.0};
	double A1[3] = {0.0, 0.0, 0.0};
	double E1[3] = {0.0, 0.0, 0.0};
	double ED1[3] = {0.0, 0.0, 0.0};
	double W1[3] = {0.0, 0.0, 0.0};
	double WD1[3] = {0.0, 0.0, 0.0};

	double P2[3] = {0.0, 0.0, 0.0};
	double V2[3] = {0.0, 0.0, 0.0};
	double A2[3] = {0.0, 0.0, 0.0};
	double E2[3] = {0.0, 0.0, 0.0};
	double ED2[3] = {0.0, 0.0, 0.0};
	double W2[3] = {0.0, 0.0, 0.0};
	double WD2[3] = {0.0, 0.0, 0.0};

	double P3[3] = {0.0, 0.0, 0.0};
	double V3[3] = {0.0, 0.0, 0.0};
	double A3[3] = {0.0, 0.0, 0.0};
	double E3[3] = {0.0, 0.0, 0.0};
	double ED3[3] = {0.0, 0.0, 0.0};
	double W3[3] = {0.0, 0.0, 0.0};
	double WD3[3] = {0.0, 0.0, 0.0};

	double P4[3] = {0.0, 0.0, 0.0};
	double V4[3] = {0.0, 0.0, 0.0};
	double A4[3] = {0.0, 0.0, 0.0};
	double E4[3] = {0.0, 0.0, 0.0};
	double ED4[3] = {0.0, 0.0, 0.0};
	double W4[3] = {0.0, 0.0, 0.0};
	double WD4[3] = {0.0, 0.0, 0.0};

};

// Functions.
void formatTables (Missile &missile, string dataFile);
Missile clone(const Missile &missile);
void emplace(Missile &missile, double phi, double theta, double psi, double ENUPosition[3]);
void seekerOn(Missile &missile);
void atmosphere(Missile &missile);
void seeker(Missile &missile);
void guidance(Missile &missile);
void control(Missile &missile);
void actuators(Missile &missile);
void aerodynamicAnglesAndConversions(Missile &missile);
void tableLookUps(Missile &missile);
void accelerationLimit(Missile &missile);
void propulsion(Missile &missile);
void aerodynamics(Missile &missile);
void aerodynamicDerivatives(Missile &missile);
void eulerIntegrateStates(Missile &missile);
void rk2IntegrateStates(Missile &missile);
void rk4IntegrateStates(Missile &missile);
void missileMotion(Missile &missile);
void performanceAndTerminationCheck(Missile &missile, double maxTime);
void writeLogFileHeader(ofstream &logFile);
void logData(Missile &missile, ofstream &logFile);
void sixDofFly(Missile &missile, string flyOutID, bool writeData, bool consoleReport, double maxTime);
void threeDofFly(Missile &missile, string flyOutID, bool writeData, bool consoleReport, double maxTime);

#endif