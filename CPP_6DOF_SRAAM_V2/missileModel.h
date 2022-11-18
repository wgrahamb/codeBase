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
const double REFERENCE_AREA                     = 0.01824; // Meters^2.
const double REFERENCE_DIAMETER                 = 0.1524; // Meters.
const double THRUST_EXIT_AREA                   = 0.0125; // Meters^2.
const double ROCKET_BURN_OUT_TIME               = 2.421; // Seconds.
const double SEEKER_KF_G                        = 10.0; // Seeker Kalman filter gain. Per second.
const double SEEKER_KF_ZETA                     = 0.9; // Seeker Kalman filter damping. Non dimensional.
const double SEEKER_KF_WN                       = 60.0; // Seeker Kalman filter natural frequency. Radians per second.
const double PROPORTIONAL_GUIDANCE_GAIN         = 2.75; // Guidance homing gain. Non dimensional.
const double MAXIMUM_ACCELERATION               = 450.0; // Roughly 45 Gs. Meters per s^2.
const double RATE_CONTROL_ZETA                  = 0.6; // Damping of constant rate control. Non dimensional.
const double ROLL_CONTROL_WN                    = 20.0; // Natural frequency of roll closed loop complex pole. Radians per second.
const double ROLL_CONTROL_ZETA                  = 0.9; // Damping of roll closed loop complex pole. Non dimensional.
const double FIN_RATE_LIMIT_RADIANS             = 10.472; // Radians per second.
const double ROLL_ANGLE_COMMAND                 = 0.0; // Radians.
const double ALPHA_PRIME_MAX                    = 40.0; // Degrees.
const double SEA_LEVEL_PRESSURE                 = 101325; // Pascals.
const double LAUNCH_CENTER_OF_GRAVITY_FROM_NOSE = 1.5357; // Meters.

/* This struct fully represents a missile. */
struct Missile
{

	/* Variables */

	// Target.
	double waypoint[3]; // guidance east, north, up target in meters (INIT)

	// Status.
	bool isBallistic    = false; // flag to indicate whether or not the missile is flying ballistically or guided
	bool isLaunched     = false; // flag to indicate whether or not the missile is free to launch
	double timeStep     = (1.0 / 600.0); // missile integration time step
	double halfTimeStep = timeStep * 0.5; // missile half integration time step for runge kutta methods

	// Body.
	double fluVel[3];    // missile velocity in the body frame in m/s (INIT)
	double spcfForce[3]; // missile acceleration in the body frame in m/s^2 (INIT)
	double spd;          // missile speed in m/s (INIT)
	double rate[3]      = {0.0, 0.0, 0.0}; // missile angular velocity in rads/s
	double rateDot[3]   = {0.0, 0.0, 0.0}; // missile angular acceleration in rads/s^2
	double tof          = 0.0; // missile time of flight in seconds
	double rng          = 0.0; // total missile range in meters
	double alphaRadians = 0.0; // vertical angle of attack in rads
	double betaRadians  = 0.0; // horizontal angle of attack in rads
	double alphaDegrees = 0.0; // vertical angle of attack in degrees
	double betaDegrees  = 0.0; // horizontal angle of attack in degrees

	// Local Frame.
	double enuPos[3];      // missile east, north, up position in meters (INIT)
	double enuVel[3];      // missile east, north, up velocity in m/s (INIT)
	double enuAcc[3];      // missile east, north, up acceleration in m/s^2 (INIT)
	double enuToFlu[3][3]; // missile east, north, up to forward, left, up transformation matrix (INIT)
	double enuAttitude[3]; // missile east, north, up attitude in radians (INIT)
	double enuAttitudeDot[3] = {0.0, 0.0, 0.0}; // missile east, north, up attitude rate in rads/s

	// Atmosphere.
	double grav       = 0.0; // gravity magnitude in m/s^2
	double fluGrav[3] = {0.0, 0.0, 0.0}; // gravity in the body frame in m/s^2
	double p          = 0.0; // air pressure in pascals
	double q          = 0.0; // dynamic pressure in pascals
	double mach       = 0.0; // mach speed

	// Seeker.
	double skrTht;            // seeker pitch angle relative to bore in radians (INIT)
	double skrPsi;            // seeker yaw angle relative to bore in radians (INIT)
	double skrEnuToFlu[3][3]; // seeker east, north, up to forward, left, up transformation matrix (INIT)
	double skrThtErr;         // seeker boresight vertical offset from target in radians (INIT)
	double skrPsiErr;         // seeker boresight horizontal offset from target in radians (INIT)
	double skrWlr;            // pointing yaw rate in rads/s (INIT)
	double skrWlq;            // Pointing pitch rate. Radians per second. (INIT)
	double skrWlrDot  = 0.0; // derivative of pointing yaw rate in rads/s^2
	double skrWlr1    = 0.0; // yaw sight line spin rate in rads/s
	double skrWlr1Dot = 0.0; // derivative of yaw sight line spin rate in rads/s^2
	double skrWlr2    = 0.0; // second state variable in yawing kalman filter in rads/s^2
	double skrWlr2Dot = 0.0; // derivative of second state variable in yawing kalman filter in rads/s^3
	double skrWlqDot  = 0.0; // derivative of pointing pitch rate in rads/s^2
	double skrWlq1    = 0.0; // pitch sight line spin rate in rads/s
	double skrWlq1Dot = 0.0; // derivative of pitch sight line spin rate in rads/s^2
	double skrWlq2    = 0.0; // second state variable in pitching kalman filter in rads/s^2
	double skrWlq2Dot = 0.0; // derivative of second state variable in pitching kalman filter in rads/s^3

	// Guidance.
	bool isHoming           = false; // flag to indicate whether or not the missile has entered the homing phase
	double timeToGo         = 0.0; // time to reach waypoint in seconds
	double mslToWaypoint[3] = {0.0, 0.0, 0.0}; // waypoint position relative to missile body in meters
	double normComm         = 0.0; // body normal guidance command in m/s^2
	double sideComm         = 0.0; // body lateral guidance command in m/s^2
	double commLimit        = MAXIMUM_ACCELERATION; // guidance command limit in m/s^2

	// Control
	double yawIntErr        = 0.0; // Something.
	double lastYawPropErr   = 0.0; // Radians per second.
	double yawPropErr       = 0.0; // Radians per second.
	double yawFinComm       = 0.0; // Radians.
	double pitchIntErr      = 0.0; // Something.
	double lastPitchPropErr = 0.0; // Radians per second.
	double pitchPropErr     = 0.0; // Radians per second.
	double pitchFinComm     = 0.0; // Radians.
	double lastRollPropErr  = 0.0; // Radians per second.
	double rollPropErr      = 0.0; // Radians per second.
	double rollFinComm      = 0.0; // Radians.

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
	// P  = ENUPosition
	// V  = ENUVelocity
	// A  = ENUAcceleration
	// E  = ENUEulerAngles
	// ED = ENUEulerDot
	// W  = BodyRate
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

// Public Functions.
void formatTables (Missile &missile, string dataFile);
Missile clone(const Missile &missile);
void emplace(Missile &missile, double phi, double theta, double psi, double ENUPosition[3]);
void seekerOn(Missile &missile);
void sixDofFly(Missile &missile, string flyOutID, bool writeData, bool consoleReport, double maxTime);
void threeDofFly(Missile &missile, string flyOutID, bool writeData, bool consoleReport, double maxTime);

#endif