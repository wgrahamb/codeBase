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

// Utility.
#include "util.h"

// Namespace.
using namespace std;

/* To do */
// Begin on NOVICE algorithms.
// Typedef vector for target trajectory.
// Bisection algorithm pip selection.
// Seeker on mode for missile.
// Each missile should have a pip and target states.
// Structs for NOVICE players.
// Scene generator.
// Scene loader.
// Algorithm to scale line of attack with line of sight.

// Simulation control.
auto wallClockStart = chrono::high_resolution_clock::now(); // Start tracking real time.
const double TIME_STEP = 0.001; // Seconds. Common sense to have a uniform time step if can.
const double HALF_TIME_STEP = TIME_STEP / 2.0; // Seconds. For rk2 and rk4 integration.

//// Missile Model ////

/*
#
# Author - Wilson Graham Beech.
# Reference - Modeling and Simulation of Aerospace Vehicle Dynamics, Second Edition - Peter H. Zipfel.
#
# ENU = East, North, Up Coordinate System.
# FLU = Forward, Left, Up Coordinate System.
#
# Interceptor Orientation.
# Array 0, Axis - Looking down the nozzle of the interceptor.
# Array 1, Side - Looking down the nozzle of the interceptor, this points out the left hand side.
# Array 2, Normal - Looking down the nozzle of the interceptor, this points out the top side.
#
#			Positive normal.
#				|
#				|
#				|
#	Positive side. -------O------- Negative side.
#				|
#				|
#				|
#			Negative normal.
#
# Negative axis is pointing out of the screen directly at you.
# Positive axis is pointing into the screen directly away from you.
#
# Positive alpha indicates nose below free stream velocity.
# Positive beta indicates nose left of free stream velocity.
# Positive roll indicates normal axis clockwisely rotated from twelve o'clock.
#
# Fin orientation, looking down the nozzle of the missile.
#
#		Fin 4	Fin 1
#			X
#		Fin 3	Fin 2
#
*/

/* Missile constants. */
const double REFERENCE_AREA = 0.01824; // Meters^2.
const double REFERENCE_DIAMETER = 0.1524; // Meters.
const double THRUST_EXIT_AREA = 0.0125; // Meters^2.
const double ROCKET_BURN_OUT_TIME = 2.421; // Seconds.
const double SEEKER_KF_G = 10.0; // Seeker Kalman filter gain. Per second.
const double SEEKER_KF_ZETA = 0.9; // Seeker Kalman filter damping. Non dimensional.
const double SEEKER_KF_WN = 60.0; // Seeker Kalman filter natural frequency. Radians per second.
const double PROPORTIONAL_GUIDANCE_GAIN = 3.0; // Guidance homing gain. Non dimensional.
const double LINE_OF_ATTACK_GUIDANCE_GAIN = 1.0; // Guidance midcourse gain. Non dimensional.
const double MAXIMUM_ACCELERATION = 450.0; // Roughly 45 Gs. Meters per s^2.
const double RATE_CONTROL_ZETA = 0.6; // Damping of constant rate control. Non dimensional.
const double ROLL_CONTROL_WN = 20.0; // Natural frequency of roll closed loop complex pole. Radians per second.
const double ROLL_CONTROL_ZETA = 0.9; // Damping of roll closed loop complex pole. Non dimensional.
const double FIN_CONTROL_WN = 100.0; // Natural frequency of roll closed loop complex pole. Radians per second.
const double FIN_CONTROL_ZETA = 0.7; // Damping of roll closed loop complex pole. Non dimensional.
const double FIN_CONTROL_MAX_DEFLECTION_DEGREES = 28.0; // Degrees.
const double FIN_CONTROL_MAX_DEFLECTION_RADIANS = 0.4887; // Radians.
const double FIN_RATE_LIMIT_RADIANS = 10.472; // Radians per second.
const double ROLL_ANGLE_COMMAND = 0.0; // Radians.
const double ALPHA_PRIME_MAX = 40.0; // Degrees.
const double SEA_LEVEL_PRESSURE = 101325; // Pascals.
const double LAUNCH_CENTER_OF_GRAVITY_FROM_NOSE = 1.5357; // Meters.

struct Missile
{

	/* Variables */

	// Target.
	double pip[3]; // Predicted Intercept Point. Meters.

	// Missile state.
	bool launch = true; // Launch command. True for now, will be needed for fire control.
	double timeOfFlight = 0.0; // Seconds.
	double missileENUToFLUMatrix[3][3]; // Non dimensional.
	double ENUPosition[3]; // Meters.
	double range = 0.0; // Meters.
	double ENUVelocity[3]; // Meters per second.
	double FLUVelocity[3]; // Meters per second.
	double speed; // Meters per second.
	double machSpeed = 0.0; // Non dimensional.
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
	double FLUMissileToPipRelativePosition[3] = {0.0, 0.0, 0.0}; // Meters.
	double guidanceNormalCommand = 0.0; // Meters per second^2.
	double guidanceSideCommand = 0.0; // Meters per second^2.
	double maneuveringLimit = MAXIMUM_ACCELERATION; // Meters per second^2.

	// Control
	double yawControlFeedForwardIntegration = 0.0; // Yaw feed forward integration. Meters per second.
	double yawControlFeedForwardDerivative = 0.0; // Yaw feed forward derivative. Meters per second.
	double pitchControlFeedForwardIntegration = 0.0; // Pitch feed forward integration. Meters per second.
	double pitchControlFeedForwardDerivative = 0.0; // Pitch feed forward derivative. Meters per second.
	double pitchFinCommand = 0.0; // Radians.
	double yawFinCommand = 0.0; // Radians.
	double rollFinCommand = 0.0; // Radians.

	// Actuators.
	double FIN1DEFL = 0.0; // Fin deflection. Radians.
	double FIN1DEFL_D = 0.0; // Fin deflection derived. Radians.
	double FIN1DEFL_DOT = 0.0; // Fin rate. Radians per second.
	double FIN1DEFL_DOTDOT = 0.0; // Fin rate derived. Radians per second^2.
	double FIN2DEFL = 0.0; // Fin deflection. Radians.
	double FIN2DEFL_D = 0.0; // Fin deflection derived. Radians.
	double FIN2DEFL_DOT = 0.0; // Fin rate. Radians per second.
	double FIN2DEFL_DOTDOT = 0.0; // Fin rate derived. Radians per second^2.
	double FIN3DEFL = 0.0; // Fin deflection. Radians.
	double FIN3DEFL_D = 0.0; // Fin deflection derived. Radians.
	double FIN3DEFL_DOT = 0.0; // Fin rate. Radians per second.
	double FIN3DEFL_DOTDOT = 0.0; // Fin rate derived. Radians per second^2.
	double FIN4DEFL = 0.0; // Fin deflection. Radians.
	double FIN4DEFL_D = 0.0; // Fin deflection derived. Radians.
	double FIN4DEFL_DOT = 0.0; // Fin rate. Radians per second.
	double FIN4DEFL_DOTDOT = 0.0; // Fin rate derived. Radians per second^2.
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
	double CA0 = 0.0; // Axial force coefficient. Non dimensional.
	double CAA = 0.0; // Axial force derivative of alpha prime. Per degree.
	double CAD = 0.0; // Axial force derivative of control fin deflection. Per degree^2.
	double CAOFF = 0.0; // Power off correction term for axial force coefficient. Non dimensional.
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
	double mass = 0.0; // Kilograms.
	double unadjustedThrust = 0.0; // Newtons.
	double transverseMomentOfInertia = 0.0; // Kilograms * meters^2.
	double axialMomentOfInertia = 0.0; // Kilograms * meters^2.
	double centerOfGravityFromNose = 0.0; // Meters.

	// Propulsion.
	double thrust = 0.0; // Newtons.

	// Aerodynamic integration coefficients.
	double CX = 0.0; // Non dimensional.
	double CY = 0.0; // Non dimensional.
	double CZ = 0.0; // Non dimensional.
	double CL = 0.0; // Non dimensional.
	double CM = 0.0; // Non dimensional.
	double CN = 0.0; // Non dimensional.

	// Aerodynamic feedback coefficients.
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

void lookUpTablesFormat (Missile &missile, string dataFile)
{
	// LOOK UP DATA
	ifstream inFile(dataFile);
	// STRING OF FILE LINE
	string line;
	// TABLE NUMBER
	int tableNoTrack = 0;
	// ROW NUMBER
	int rowNoTrack = 0;
	// VECTOR TO STORE TABLE DIMENSIONS
	vector<vector<int>> dimensions;
	// LOOP
	while(getline(inFile, line))
	{
		// FLAG FOR INDICATION OF LINE CLASSIFICATION >>> 1 = ONE DIMENSIONAL TABLE SIZE; TWO = TWO DIMENSIONAL TABLE SIZE; THREE = TABLE NAME
		int flag = 0;
		// INITIALIZE NAME OF TABLE
		string name;
		// INITIALIZE DIMENSION OF SPECIFIC TABLE
		vector<int> dimension;
		// FIND TABLE NAME
		if (line.substr(0, 4) == "NAME")
		{
			// RE INIT ROW NUMBER TRACKER
			rowNoTrack = 0;
			// STORE NAME OF TABLE
			name = line.substr(5, line.size() - 6);
			// TRACK TABLE NUMBER
			tableNoTrack += 1;
			// MARK FLAG FOR LATER USE
			flag = 3;
		}
		// FIND TABLE DIMENSION
		else if (line.substr(0, 2) == "NX")
		{
			// MARK FLAG FOR LATER USE
			flag = 1;
			// STORE "ROWS" DIMENSION
			int D1 = stoi(line.substr(4, 3));
			// STORE "ROWS" DIMENSIONS IN VECTOR
			dimension.push_back(D1);
			// INITIALIZE "COLUMNS" DIMENSION
			int D2 = 0;
			// CHECK FOR A DETERMINED "COLUMNS" DIMENSION
			for (int i = 3; i < line.size(); i++) {
				// CHECK
				if (line.substr(i, 2) == "NX")
				{
					// MARK FLAG FOR LATER USE
					flag = 2;
					// ADD ONE TO ROWS DIMENSION SINCE THIS IS A TWO DIMENSIONAL TABLE
					dimension[0] += 1;
					// STORE "COLUMNS" DIMENSION
					D2 = stoi(line.substr(i+4, 3)) + 1;
					// STORE "COLUMNS" DIMENSION IN VECTOR
					dimension.push_back(D2);
				}
			}
			// IF NO DETERMINED SECOND DIMENSION
			if (D2 == 0)
			{
				// "COLUMNS" DIMENSION BECOMES TWO
				D2 = 2;
				// STORE "COLUMNS" DIMENSION IN VECTOR
				dimension.push_back(D2);
			}
		}
		// NOTHING FLAGGED, NEXT ITERATION
		if (flag == 0)
		{
			// ONLY CHECK IF A TABLE HAS BEEN INITIALIZED
			if (dimensions.size() > 0)
			{
				// COUNT ROW NUMBER
				rowNoTrack += 1;
				// PARSE LINE THROUGH A STREAM
				istringstream parseLine(line);
				// INITIALIZE COLUMN COUNTER
				int columnCount = 0;
				// LOOP THROUGH ONE ROW, ALL COLUMNS
				do
				{
					// ITERATE COLUMN COUNTER
					columnCount += 1;
					// INITIALIZE DATA POINT
					string dataPoint;
					// GRAB DATA POINT FROM PARSES
					parseLine >> dataPoint;
					// CHECK TO MAKE SURE IT IS NOT WHITESPACE
					if (dataPoint.find_first_not_of(' ') != std::string::npos)
					{
						// CONVERT STRING TO DOUBLE
						double dataPointDouble = stod(dataPoint);
						/////////// FOR THIS SPECIFIC SET OF DATA, CHECK FOR 90. THERE ARE 14 ROWS AND 15 COLUMNS FOR THE TWO DIMENSIONAL TABLES WHICH MEANS THIS IS A SPECIFIC PIECE OF CODE. WOULD HAVE TO BE ALTERED FOR DIFFERING DATA SETS.
						if (dataPointDouble == 90)
						{
							// PLACE IT AT THE FAR RIGHT CORNER
							missile.tables[tableNoTrack - 1][0].back() = dataPointDouble;
						}
						// IF THIS THE FIRST LOOP, THIS IS THE COLUMN IN THE DATA SET THAT DISPLAYS THE "ROWS" VALUES
						else if (columnCount == 1)
						{
							// FOR TWO DIMENSIONAL TABLE
							if (dimensions[tableNoTrack -1][1] != 2)
							{
								// PLACE DATA POINT IN ITS PLACE
								missile.tables[tableNoTrack - 1][rowNoTrack][0] = dataPointDouble;
							}
							// FOR ONE DIMENSIONAL TABLE
							else
							{
								// PLACE DATA POINT IN ITS PLACE
								missile.tables[tableNoTrack - 1][rowNoTrack - 1][0] = dataPointDouble;
							}
						}
						// IF THIS THE SECOND LOOP, THIS IS THE COLUMN IN THE DATA SET THAT DISPLAYS THE "COLUMNS" VALUES, ONLY FOR TWO DIMENSIONAL TABLES
						else if (columnCount == 2 and dimensions[tableNoTrack -1][1] != 2)
						{
							// PLACE DATA POINT IN ITS PLACE
							missile.tables[tableNoTrack - 1][0][rowNoTrack] = dataPointDouble;
						}
						// ELSE FOR ACTUAL DATA POINTS
						else
						{
							// FOR TWO DIMENSIONAL TABLES
							if (dimensions[tableNoTrack -1][1] != 2)
							{
								// PLACE DATA POINT IN ITS PLACE
								missile.tables[tableNoTrack - 1][rowNoTrack][columnCount - 2] = dataPointDouble;
							}
							// FOR ONE DIMENSIONAL TABLES
							else
							{
								// PLACE DATA POINT IN ITS PLACE
								missile.tables[tableNoTrack - 1][rowNoTrack - 1][columnCount - 1] = dataPointDouble;
							}
						}
					}
				} while (parseLine);
			}
		}
		// CREATE A TABLE OF CORRECT SIZE AND STORE IT
		else if (flag == 1 or flag == 2)
		{
			// STORE VECTOR OF DIMENSIONS
			dimensions.push_back(dimension);
			// SEPERATE ROW DIMENSION
			int rows = dimension[0];
			// SEPERATE COLUMN DIMENSION
			int columns = dimension[1];
			// CREATE TABLE
			vector<vector<double>> newTable(rows, vector<double>(columns));
			// TOP LEFT CORNER OF TABLE UNUSED
			newTable[0][0] = 0.0;
			// STORE NEW TABLE IN VECTOR
			missile.tables.push_back(newTable);
		}
		// STORE NAME OF TABLE
		else if (flag == 3)
		{
			// MAP TABLE NAME INDEX PAIR
			missile.tableNameIndexPairs.emplace(name, tableNoTrack - 1);
		}
	}
}

// Works for now.
// Needs to be refactored for fire control.
// This will be done by the launcher, when the scenario is created.
// When a missile packet is copied for flyouts, this does not have to be done.
// Phi, theta, and psi will be input to this function.
// Pip will come from elsewhere, most likely from the launch command.
void initUnLaunchedMissile(Missile &missile)
{

	// Intitialize and open input file.
	std::ifstream inPut;
	inPut.open("input.txt");

	// Declare inputs.
	double phi, theta, psi, tgtE, tgtN, tgtU;

	// Populate inputs from file.
	inPut >> phi >> theta >> psi >> tgtE >> tgtN >> tgtU >> missile.INTEGRATION_METHOD;

	// Set pip.
	// Will be changed for inputs.
	missile.pip[0] = tgtE; // Meters.
	missile.pip[1] = tgtN; // Meters.
	missile.pip[2] = tgtU; // Meters.

	phi *= degToRad;
	theta *= degToRad;
	psi *= degToRad;

	// Missile.
	missile.ENUEulerAngles[0] = phi;
	missile.ENUEulerAngles[1] = theta;
	missile.ENUEulerAngles[2] = psi;
	eulerAnglesToLocalOrientation(phi, -theta, psi, missile.missileENUToFLUMatrix);
	missile.ENUPosition[0] = 0.0;
	missile.ENUPosition[1] = 0.0;
	missile.ENUPosition[2] = 0.0;
	missile.ENUVelocity[0] = missile.missileENUToFLUMatrix[0][0];
	missile.ENUVelocity[1] = missile.missileENUToFLUMatrix[0][1];
	missile.ENUVelocity[2] = missile.missileENUToFLUMatrix[0][2];
	threeByThreeTimesThreeByOne(missile.missileENUToFLUMatrix, missile.ENUVelocity, missile.FLUVelocity);
	missile.ENUAcceleration[0] = 0.0;
	missile.ENUAcceleration[1] = 0.0;
	missile.ENUAcceleration[2] = 0.0;
	missile.FLUAcceleration[0] = 0.0;
	missile.FLUAcceleration[1] = 0.0;
	missile.FLUAcceleration[2] = 0.0;
	magnitude(missile.ENUVelocity, missile.speed);

	// Intialize seeker by pointing it directly at the target.
	double relPos[3];
	subtractTwoVectors(missile.ENUPosition, missile.pip, relPos);
	double relPosU[3];
	unitVec(relPos, relPosU);
	double mslToInterceptU[3];
	threeByThreeTimesThreeByOne(missile.missileENUToFLUMatrix, relPosU, mslToInterceptU);
	double mslToInterceptAz, mslToInterceptEl;
	azAndElFromVector(mslToInterceptAz, mslToInterceptEl, mslToInterceptU);
	missile.seekerPitch = mslToInterceptEl;
	missile.seekerYaw = mslToInterceptAz;
	double seekerAttitudeToLocalTM[3][3];
	eulerAnglesToLocalOrientation(0.0, -missile.seekerPitch, missile.seekerYaw, seekerAttitudeToLocalTM);
	threeByThreeTimesThreeByThree(seekerAttitudeToLocalTM, missile.missileENUToFLUMatrix, missile.seekerENUToFLUMatrix);
	missile.seekerPitchError = 0.0;
	missile.seekerYawError = 0.0;
	missile.seekerWLR = missile.seekerYaw;
	missile.seekerWLQ = missile.seekerPitch;

	// Format data tables.
	lookUpTablesFormat(missile, "shortRangeInterceptorTables.txt");

	// Set missile lethality.
	missile.lethality = "FLYING"; // STATUS

	// Console output.
	cout << "\n" << endl;
	cout << "MODEL INITIATED" << endl;
	cout << "\n" << endl;

}

void atmosphere(Missile &missile)
{

	int index;

	double alt = missile.ENUPosition[2] * mToKm;

	index = missile.tableNameIndexPairs["RHO"];
	double rho = linearInterpolationWithBoundedEnds(missile.tables[index], alt); // Kilograms per meter^3.

	index = missile.tableNameIndexPairs["GRAVITY"];
	missile.grav = linearInterpolationWithBoundedEnds(missile.tables[index], alt); // Meters per second^2.

	double gravLocalVec[3] = {0.0, 0.0, -missile.grav};
	threeByThreeTimesThreeByOne(missile.missileENUToFLUMatrix, gravLocalVec, missile.FLUGravity);

	index = missile.tableNameIndexPairs["PRESSURE"];
	missile.pressure = linearInterpolationWithBoundedEnds(missile.tables[index], alt);

	index = missile.tableNameIndexPairs["SPEED_OF_SOUND"];
	double a = linearInterpolationWithBoundedEnds(missile.tables[index], alt); // Meters per second^2.

	magnitude(missile.ENUVelocity, missile.speed);
	missile.machSpeed = missile.speed / a;
	missile.dynamicPressure = 0.5 * rho * missile.speed * missile.speed;

}

void seeker(Missile &missile)
{

	double wsq = SEEKER_KF_WN * SEEKER_KF_WN;
	double gg = SEEKER_KF_G * wsq;

	// Yaw channel.
	double wlr1d_new = missile.seekerWLR2;
	double wlr1_new = trapezoidIntegrate(wlr1d_new, missile.seekerWLR1D, missile.seekerWLR1, TIME_STEP);
	missile.seekerWLR1 = wlr1_new;
	missile.seekerWLR1D = wlr1d_new;
	double wlr2d_new = gg * missile.seekerYawError - 2 * SEEKER_KF_ZETA * SEEKER_KF_WN * missile.seekerWLR1D - wsq * missile.seekerWLR1;
	double wlr2_new = trapezoidIntegrate(wlr2d_new, missile.seekerWLR2D, missile.seekerWLR2, TIME_STEP);
	missile.seekerWLR2 = wlr2_new;
	missile.seekerWLR2D = wlr2d_new;

	// Yaw control.
	double wlrd_new = missile.seekerWLR1 - missile.bodyRate[2];
	double wlr_new = trapezoidIntegrate(wlrd_new, missile.seekerWLRD, missile.seekerWLR, TIME_STEP);
	missile.seekerWLR = wlr_new;
	missile.seekerWLRD = wlrd_new;
	missile.seekerYaw = missile.seekerWLR;

	// Pitch channel.
	double wlq1d_new = missile.seekerWLQ2;
	double wlq1_new = trapezoidIntegrate(wlq1d_new, missile.seekerWLQ1D, missile.seekerWLQ1, TIME_STEP);
	missile.seekerWLQ1 = wlq1_new;
	missile.seekerWLQ1D = wlq1d_new;
	double wlq2d_new = gg * missile.seekerPitchError - 2 * SEEKER_KF_ZETA * SEEKER_KF_WN * missile.seekerWLQ1D - wsq * missile.seekerWLQ1;
	double wlq2_new = trapezoidIntegrate(wlq2d_new, missile.seekerWLQ2D, missile.seekerWLQ2, TIME_STEP);
	missile.seekerWLQ2 = wlq2_new;
	missile.seekerWLQ2D = wlq2d_new;

	// Pitch control.
	double wlqd_new = missile.seekerWLQ1 - missile.bodyRate[1];
	double wlq_new = trapezoidIntegrate(wlqd_new, missile.seekerWLQD, missile.seekerWLQ, TIME_STEP);
	missile.seekerWLQ = wlq_new;
	missile.seekerWLQD = wlqd_new;
	missile.seekerPitch = missile.seekerWLQ;

	// Convert seeker data to FLU relative position for guidance.
	double localRelPos[3];
	subtractTwoVectors(missile.ENUPosition, missile.pip, localRelPos);
	double seekerAttitudeToLocalTM[3][3];
	eulerAnglesToLocalOrientation(0.0, -missile.seekerPitch, missile.seekerYaw, seekerAttitudeToLocalTM);
	threeByThreeTimesThreeByThree(seekerAttitudeToLocalTM, missile.missileENUToFLUMatrix, missile.seekerENUToFLUMatrix);
	double seekerToInterceptRelPos[3];
	threeByThreeTimesThreeByOne(missile.seekerENUToFLUMatrix, localRelPos, seekerToInterceptRelPos);
	double inducedErr[3] = {1.0, 0.5, 0.2};
	double seekerToInterceptRelPosWithErr[3];
	multiplyTwoVectors(seekerToInterceptRelPos, inducedErr, seekerToInterceptRelPosWithErr);
	azAndElFromVector(missile.seekerYawError, missile.seekerPitchError, seekerToInterceptRelPosWithErr);
	oneByThreeTimesThreeByThree(seekerToInterceptRelPosWithErr, seekerAttitudeToLocalTM, missile.FLUMissileToPipRelativePosition);

}

void guidance(Missile &missile)
{

	double forwardLeftUpMissileToInterceptPositionUnitVector[3];
	unitVec(missile.FLUMissileToPipRelativePosition, forwardLeftUpMissileToInterceptPositionUnitVector);
	double forwardLeftUpMissileToInterceptLineOfSightVel[3];
	vectorProjection(forwardLeftUpMissileToInterceptPositionUnitVector, missile.FLUVelocity, forwardLeftUpMissileToInterceptLineOfSightVel);
	double timeToGo, forwardLeftUpMissileToInterceptPositionMagnitude, forwardLeftUpMissileToInterceptLineOfSightVelMagnitude;
	magnitude(missile.FLUMissileToPipRelativePosition, forwardLeftUpMissileToInterceptPositionMagnitude);
	magnitude(forwardLeftUpMissileToInterceptLineOfSightVel, forwardLeftUpMissileToInterceptLineOfSightVelMagnitude);
	timeToGo = forwardLeftUpMissileToInterceptPositionMagnitude / forwardLeftUpMissileToInterceptLineOfSightVelMagnitude;

	if (timeToGo > -10000) // This means the missile is always using proportional guidance.
	{

		double closingVelocity[3];
		multiplyVectorTimesScalar(-1.0, missile.FLUVelocity, closingVelocity);
		double closingSpeed;
		magnitude(closingVelocity, closingSpeed);
		double TEMP1[3], TEMP2;
		crossProductTwoVectors(missile.FLUMissileToPipRelativePosition, closingVelocity, TEMP1);
		dotProductTwoVectors(missile.FLUMissileToPipRelativePosition, missile.FLUMissileToPipRelativePosition, TEMP2);
		double lineOfSightRate[3];
		divideVectorByScalar(TEMP2, TEMP1, lineOfSightRate);
		double TEMP3, TEMP4[3];
		double proportionalGuidanceGain = 3.0;
		TEMP3 = -1 * proportionalGuidanceGain * closingSpeed;
		multiplyVectorTimesScalar(TEMP3, forwardLeftUpMissileToInterceptPositionUnitVector, TEMP4);
		double COMMAND[3];
		crossProductTwoVectors(TEMP4, lineOfSightRate, COMMAND);
		missile.guidanceNormalCommand = COMMAND[2];
		missile.guidanceSideCommand = COMMAND[1];

	}
	else // Otherwise use trajectory shaping. Need an algorithm for line of attack scheduling. This missile likes pro nav.
	{

		double lineOfAttack[3];
		lineOfAttack[0] = 0.2;
		lineOfAttack[1] = 0.2;
		lineOfAttack[2] = 0.2;
		double forwardLeftUpMissileToInterceptLineOfAttackVel[3];
		vectorProjection(lineOfAttack, missile.FLUVelocity, forwardLeftUpMissileToInterceptLineOfAttackVel);
		double G = 1 - exp(-0.001 * forwardLeftUpMissileToInterceptPositionMagnitude);
		missile.guidanceNormalCommand = LINE_OF_ATTACK_GUIDANCE_GAIN * (forwardLeftUpMissileToInterceptLineOfSightVel[2] + G * forwardLeftUpMissileToInterceptLineOfAttackVel[2]);
		missile.guidanceSideCommand = LINE_OF_ATTACK_GUIDANCE_GAIN * (forwardLeftUpMissileToInterceptLineOfSightVel[1] + G * forwardLeftUpMissileToInterceptLineOfAttackVel[1]);

	}

}

void control(Missile &missile)
{

	if (missile.machSpeed > 0.6)
	{

		// Aerodynamic feedback.
		double DNA = missile.CNA * (missile.dynamicPressure * REFERENCE_AREA / missile.mass);
		double DMA = missile.CMA * (missile.dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / missile.transverseMomentOfInertia);
		double DMD = missile.CMD * (missile.dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / missile.transverseMomentOfInertia);
		double DMQ = missile.CMQ * (REFERENCE_DIAMETER / (2 * missile.speed)) * (missile.dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / missile.transverseMomentOfInertia);
		double DLP = missile.CLP * (REFERENCE_DIAMETER / (2 * missile.speed)) * (missile.dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / missile.axialMomentOfInertia);
		double DLD = missile.CLD * (missile.dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / missile.axialMomentOfInertia);

		// Natural frequency and damping scheduling.
		double WACL = 0.013 * sqrt(missile.dynamicPressure) + 7.1;
		double ZACL = 0.000559 * sqrt(missile.dynamicPressure) + 0.232;
		double PACL = 14;

		// Feedback gain scheduling.
		double GAINFB3 = WACL * WACL * PACL / (DNA * DMD);
		double GAINFB2 = (2 * ZACL * WACL + PACL + DMQ - DNA / missile.speed) / DMD;
		double GAINFB1 = (
			WACL * WACL +
			2 * ZACL * WACL * PACL +
			DMA +
			DMQ * DNA / missile.speed -
			GAINFB2 * DMD * DNA / missile.speed
		) / (DNA * DMD);

		// Roll control.
		double GKP = (2 * ROLL_CONTROL_WN * ROLL_CONTROL_ZETA + DLP) / DLD;
		double GKPHI = ROLL_CONTROL_WN * ROLL_CONTROL_WN / DLD;
		double EPHI = GKPHI * (ROLL_ANGLE_COMMAND - missile.ENUEulerAngles[0]);
		missile.rollFinCommand = EPHI - GKP * missile.bodyRate[0];

		// Pitch control.
		double zzdNew = missile.guidanceNormalCommand - missile.FLUAcceleration[2];
		double zzNew = trapezoidIntegrate(
			zzdNew,
			missile.pitchControlFeedForwardDerivative,
			missile.pitchControlFeedForwardIntegration,
			TIME_STEP
		);
		missile.pitchControlFeedForwardIntegration = zzNew;
		missile.pitchControlFeedForwardDerivative = zzdNew;
		double deflPitch = -1 * GAINFB1 * missile.FLUAcceleration[2] - GAINFB2 * missile.bodyRate[1] + GAINFB3 * missile.pitchControlFeedForwardIntegration;
		if (abs(deflPitch) > FIN_CONTROL_MAX_DEFLECTION_DEGREES)
		{
			if (deflPitch > 0)
			{
				deflPitch = FIN_CONTROL_MAX_DEFLECTION_DEGREES;
			}
			else if (deflPitch < 0)
			{
				deflPitch = -1 * FIN_CONTROL_MAX_DEFLECTION_DEGREES;
			}
		}
		missile.pitchFinCommand = deflPitch * degToRad;

		// Yaw control.
		double yydNew = missile.FLUAcceleration[1] - missile.guidanceSideCommand;
		double yyNew = trapezoidIntegrate(
			yydNew,
			missile.yawControlFeedForwardDerivative,
			missile.yawControlFeedForwardIntegration,
			TIME_STEP
		);
		missile.yawControlFeedForwardIntegration = yyNew;
		missile.yawControlFeedForwardDerivative = yydNew;
		double deflYaw = GAINFB1 * missile.FLUAcceleration[1] - GAINFB2 * missile.bodyRate[2] + GAINFB3 * missile.yawControlFeedForwardIntegration;
		if (abs(deflYaw) > FIN_CONTROL_MAX_DEFLECTION_DEGREES)
		{
			if (deflYaw > 0)
			{
				deflYaw = FIN_CONTROL_MAX_DEFLECTION_DEGREES;
			}
			else if (deflYaw < 0)
			{
				deflYaw = -1 * FIN_CONTROL_MAX_DEFLECTION_DEGREES;
			}
		}
		missile.yawFinCommand = deflYaw * degToRad;

	}

	else if (missile.machSpeed > 0.01)
	{

		// Aerodynamic feedback.
		double DNA = missile.CNA * (missile.dynamicPressure * REFERENCE_AREA / missile.mass);
		double DND = missile.CND * (missile.dynamicPressure * REFERENCE_AREA / missile.mass);
		double DMA = missile.CMA * (missile.dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / missile.transverseMomentOfInertia);
		double DMD = missile.CMD * (missile.dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / missile.transverseMomentOfInertia);
		double DMQ = missile.CMQ * (REFERENCE_DIAMETER / (2 * missile.speed)) * (missile.dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / missile.transverseMomentOfInertia);
		double DLP = missile.CLP * (REFERENCE_DIAMETER / (2 * missile.speed)) * (missile.dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / missile.axialMomentOfInertia);
		double DLD = missile.CLD * (missile.dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / missile.axialMomentOfInertia);

		// Roll control.
		double GKP = (2 * ROLL_CONTROL_WN * ROLL_CONTROL_ZETA + DLP) / DLD;
		double GKPHI = ROLL_CONTROL_WN * ROLL_CONTROL_WN / DLD;
		double EPHI = GKPHI * (ROLL_ANGLE_COMMAND - missile.ENUEulerAngles[0]);
		missile.rollFinCommand = EPHI - GKP * missile.bodyRate[0];

		// Rate control.
		double ZRATE = DNA / missile.speed - DMA * DND / (missile.speed * DMD);
		double AA = DNA / missile.speed - DMQ;
		double BB = -1 * DMA - DMQ * DNA / missile.speed;
		double TEMP1 = AA - 2 * RATE_CONTROL_ZETA * RATE_CONTROL_ZETA * ZRATE;
		double TEMP2 = AA * AA - 4 * RATE_CONTROL_ZETA * RATE_CONTROL_ZETA * BB;
		double RADIX = TEMP1 * TEMP1 - TEMP2;
		double GRATE = (-1 * TEMP1 + sqrt(RADIX)) / (-1 * DMD);

		// Pitch control.
		missile.pitchFinCommand = GRATE * missile.bodyRate[1]; // Radians.

		// Yaw control.
		missile.yawFinCommand = GRATE * missile.bodyRate[2]; // Radians.

	}
	
	else
	{

		missile.rollFinCommand = 0.0;
		missile.pitchFinCommand = 0.0;
		missile.yawFinCommand = 0.0;

	}

}

void actuators(Missile &missile)
{

	// Fin commands.
	double DEL1C = -missile.rollFinCommand + missile.pitchFinCommand - missile.yawFinCommand;
	double DEL2C = -missile.rollFinCommand + missile.pitchFinCommand + missile.yawFinCommand;
	double DEL3C = missile.rollFinCommand + missile.pitchFinCommand - missile.yawFinCommand;
	double DEL4C = missile.rollFinCommand + missile.pitchFinCommand + missile.yawFinCommand;

	int flag;

	// Fin one.
	flag = 0;
	if (abs(missile.FIN1DEFL) > FIN_CONTROL_MAX_DEFLECTION_RADIANS)
	{
		if (missile.FIN1DEFL < 0)
		{
			missile.FIN1DEFL = -1 * FIN_CONTROL_MAX_DEFLECTION_RADIANS;
		}
		else if (missile.FIN1DEFL > 0)
		{
			missile.FIN1DEFL = FIN_CONTROL_MAX_DEFLECTION_RADIANS;
		}
		if ((missile.FIN1DEFL * missile.FIN1DEFL_DOT) > 0)
		{
			missile.FIN1DEFL_DOT = 0;
		}
	}
	if (abs(missile.FIN1DEFL_DOT) > FIN_RATE_LIMIT_RADIANS)
	{
		flag = 1;
		if (missile.FIN1DEFL_DOT < 0)
		{
			missile.FIN1DEFL_DOT = -1 * FIN_RATE_LIMIT_RADIANS;
		}
		else if (missile.FIN1DEFL_DOT > 0)
		{
			missile.FIN1DEFL_DOT = FIN_RATE_LIMIT_RADIANS;
		}
	}
	double DEL1D_NEW = missile.FIN1DEFL_DOT;
	double DEL1_NEW = trapezoidIntegrate(DEL1D_NEW, missile.FIN1DEFL_D, missile.FIN1DEFL, TIME_STEP);
	missile.FIN1DEFL = DEL1_NEW;
	missile.FIN1DEFL_D = DEL1D_NEW;
	double EDX1 = DEL1C - missile.FIN1DEFL;
	double DEL1DOTDOT_NEW = FIN_CONTROL_WN * FIN_CONTROL_WN * EDX1 - 2 * FIN_CONTROL_ZETA * FIN_CONTROL_WN * missile.FIN1DEFL_D;
	double DEL1DOT_NEW = trapezoidIntegrate(DEL1DOTDOT_NEW, missile.FIN1DEFL_DOTDOT, missile.FIN1DEFL_DOT, TIME_STEP);
	missile.FIN1DEFL_DOT = DEL1DOT_NEW;
	missile.FIN1DEFL_DOTDOT = DEL1DOTDOT_NEW;
	if (flag == 1 and (missile.FIN1DEFL_DOT * missile.FIN1DEFL_DOTDOT) > 0)
	{
		missile.FIN1DEFL_DOTDOT = 0.0;
	}

	// Fin two.
	flag = 0;
	if (abs(missile.FIN2DEFL) > FIN_CONTROL_MAX_DEFLECTION_RADIANS)
	{
		if (missile.FIN2DEFL < 0)
		{
			missile.FIN2DEFL = -1 * FIN_CONTROL_MAX_DEFLECTION_RADIANS;
		}
		else if (missile.FIN2DEFL > 0)
		{
			missile.FIN2DEFL = FIN_CONTROL_MAX_DEFLECTION_RADIANS;
		}
		if ((missile.FIN2DEFL * missile.FIN2DEFL_DOT) > 0)
		{
			missile.FIN2DEFL_DOT = 0;
		}
	}
	if (abs(missile.FIN2DEFL_DOT) > FIN_RATE_LIMIT_RADIANS)
	{
		flag = 1;
		if (missile.FIN2DEFL_DOT < 0)
		{
			missile.FIN2DEFL_DOT = -1 * FIN_RATE_LIMIT_RADIANS;
		}
		else if (missile.FIN2DEFL_DOT > 0)
		{
			missile.FIN2DEFL_DOT = FIN_RATE_LIMIT_RADIANS;
		}
	}
	double DEL2D_NEW = missile.FIN2DEFL_DOT;
	double DEL2_NEW = trapezoidIntegrate(DEL2D_NEW, missile.FIN2DEFL_D, missile.FIN2DEFL, TIME_STEP);
	missile.FIN2DEFL = DEL2_NEW;
	missile.FIN2DEFL_D = DEL2D_NEW;
	double EDX2 = DEL2C - missile.FIN2DEFL;
	double DEL2DOTDOT_NEW = FIN_CONTROL_WN * FIN_CONTROL_WN * EDX2 - 2 * FIN_CONTROL_ZETA * FIN_CONTROL_WN * missile.FIN2DEFL_D;
	double DEL2DOT_NEW = trapezoidIntegrate(DEL2DOTDOT_NEW, missile.FIN2DEFL_DOTDOT, missile.FIN2DEFL_DOT, TIME_STEP);
	missile.FIN2DEFL_DOT = DEL2DOT_NEW;
	missile.FIN2DEFL_DOTDOT = DEL2DOTDOT_NEW;
	if (flag == 1 and (missile.FIN2DEFL_DOT * missile.FIN2DEFL_DOTDOT) > 0)
	{
		missile.FIN2DEFL_DOTDOT = 0.0;
	}

	// Fin three.
	flag = 0;
	if (abs(missile.FIN3DEFL) > FIN_CONTROL_MAX_DEFLECTION_RADIANS)
	{
		if (missile.FIN3DEFL < 0)
		{
			missile.FIN3DEFL = -1 * FIN_CONTROL_MAX_DEFLECTION_RADIANS;
		}
		else if (missile.FIN3DEFL > 0)
		{
			missile.FIN3DEFL = FIN_CONTROL_MAX_DEFLECTION_RADIANS;
		}
		if ((missile.FIN3DEFL * missile.FIN3DEFL_DOT) > 0)
		{
			missile.FIN3DEFL_DOT = 0;
		}
	}
	if (abs(missile.FIN3DEFL_DOT) > FIN_RATE_LIMIT_RADIANS)
	{
		flag = 1;
		if (missile.FIN3DEFL_DOT < 0)
		{
			missile.FIN3DEFL_DOT = -1 * FIN_RATE_LIMIT_RADIANS;
		}
		else if (missile.FIN3DEFL_DOT > 0)
		{
			missile.FIN3DEFL_DOT = FIN_RATE_LIMIT_RADIANS;
		}
	}
	double DEL3D_NEW = missile.FIN3DEFL_DOT;
	double DEL3_NEW = trapezoidIntegrate(DEL3D_NEW, missile.FIN3DEFL_D, missile.FIN3DEFL, TIME_STEP);
	missile.FIN3DEFL = DEL3_NEW;
	missile.FIN3DEFL_D = DEL3D_NEW;
	double EDX3 = DEL3C - missile.FIN3DEFL;
	double DEL3DOTDOT_NEW = FIN_CONTROL_WN * FIN_CONTROL_WN * EDX3 - 2 * FIN_CONTROL_ZETA * FIN_CONTROL_WN * missile.FIN3DEFL_D;
	double DEL3DOT_NEW = trapezoidIntegrate(DEL3DOTDOT_NEW, missile.FIN3DEFL_DOTDOT, missile.FIN3DEFL_DOT, TIME_STEP);
	missile.FIN3DEFL_DOT = DEL3DOT_NEW;
	missile.FIN3DEFL_DOTDOT = DEL3DOTDOT_NEW;
	if (flag == 1 and (missile.FIN3DEFL_DOT * missile.FIN3DEFL_DOTDOT) > 0)
	{
		missile.FIN3DEFL_DOTDOT = 0.0;
	}

	// Fin four.
	flag = 0;
	if (abs(missile.FIN4DEFL) > FIN_CONTROL_MAX_DEFLECTION_RADIANS)
	{
		if (missile.FIN4DEFL < 0)
		{
			missile.FIN4DEFL = -1 * FIN_CONTROL_MAX_DEFLECTION_RADIANS;
		}
		else if (missile.FIN4DEFL > 0)
		{
			missile.FIN4DEFL = FIN_CONTROL_MAX_DEFLECTION_RADIANS;
		}
		if ((missile.FIN4DEFL * missile.FIN4DEFL_DOT) > 0)
		{
			missile.FIN4DEFL_DOT = 0;
		}
	}
	if (abs(missile.FIN4DEFL_DOT) > FIN_RATE_LIMIT_RADIANS)
	{
		flag = 1;
		if (missile.FIN4DEFL_DOT < 0)
		{
			missile.FIN4DEFL_DOT = -1 * FIN_RATE_LIMIT_RADIANS;
		}
		else if (missile.FIN4DEFL_DOT > 0)
		{
			missile.FIN4DEFL_DOT = FIN_RATE_LIMIT_RADIANS;
		}
	}
	double DEL4D_NEW = missile.FIN4DEFL_DOT;
	double DEL4_NEW = trapezoidIntegrate(DEL4D_NEW, missile.FIN4DEFL_D, missile.FIN4DEFL, TIME_STEP);
	missile.FIN4DEFL = DEL4_NEW;
	missile.FIN4DEFL_D = DEL4D_NEW;
	double EDX4 = DEL4C - missile.FIN4DEFL;
	double DEL4DOTDOT_NEW = FIN_CONTROL_WN * FIN_CONTROL_WN * EDX4 - 2 * FIN_CONTROL_ZETA * FIN_CONTROL_WN * missile.FIN4DEFL_D;
	double DEL4DOT_NEW = trapezoidIntegrate(DEL4DOTDOT_NEW, missile.FIN4DEFL_DOTDOT, missile.FIN4DEFL_DOT, TIME_STEP);
	missile.FIN4DEFL_DOT = DEL4DOT_NEW;
	missile.FIN4DEFL_DOTDOT = DEL4DOTDOT_NEW;
	if (flag == 1 and (missile.FIN4DEFL_DOT * missile.FIN4DEFL_DOTDOT) > 0)
	{
		missile.FIN4DEFL_DOTDOT = 0.0;
	}

	// Attitude fin deflections.
	missile.rollFinDeflection = (-missile.FIN1DEFL - missile.FIN2DEFL + missile.FIN3DEFL + missile.FIN4DEFL) / 4;
	missile.pitchFinDeflection = (missile.FIN1DEFL + missile.FIN2DEFL + missile.FIN3DEFL + missile.FIN4DEFL) / 4;
	missile.yawFinDeflection = (-missile.FIN1DEFL + missile.FIN2DEFL - missile.FIN3DEFL + missile.FIN4DEFL) / 4;

}

void aerodynamicAnglesAndConversions(Missile &missile)
{

	missile.alphaRadians = -1 * atan2(missile.FLUVelocity[2], missile.FLUVelocity[0]);
	missile.betaRadians = atan2(missile.FLUVelocity[1], missile.FLUVelocity[0]);
	missile.alphaDegrees = missile.alphaRadians * radToDeg;
	missile.betaDegrees = missile.betaRadians * radToDeg;
	missile.alphaPrimeRadians = acos(cos(missile.alphaRadians) * cos(missile.betaRadians));
	missile.alphaPrimeDegrees = radToDeg * missile.alphaPrimeRadians;
	double phiPrime = atan2(tan(missile.betaRadians), sin(missile.alphaRadians));
	missile.sinPhiPrime = sin(phiPrime);
	missile.cosPhiPrime = cos(phiPrime);
	double pitchDeflAeroFrame = missile.pitchFinDeflection * missile.cosPhiPrime - missile.yawFinDeflection * missile.sinPhiPrime;
	missile.pitchAeroBallisticFinDeflectionDegrees = radToDeg * pitchDeflAeroFrame;
	double yawDeflAeroFrame = missile.pitchFinDeflection * missile.sinPhiPrime + missile.yawFinDeflection * missile.cosPhiPrime;
	missile.yawAeroBallisticFinDeflectionDegrees = radToDeg * yawDeflAeroFrame;
	missile.rollFinDeflectionDegrees = radToDeg * missile.rollFinDeflection;
	missile.totalFinDeflectionDegrees = (abs(missile.pitchAeroBallisticFinDeflectionDegrees) + abs(missile.yawAeroBallisticFinDeflectionDegrees)) / 2;
	double pitchRateAeroFrame = missile.bodyRate[1] * missile.cosPhiPrime - missile.bodyRate[2] * missile.sinPhiPrime;
	missile.pitchAeroBallisticBodyRateDegrees = radToDeg * pitchRateAeroFrame;
	double yawRateAeroFrame = missile.bodyRate[1] * missile.sinPhiPrime + missile.bodyRate[2] * missile.cosPhiPrime;
	missile.yawAeroBallisticBodyRateDegrees = radToDeg * yawRateAeroFrame;
	missile.rollRateDegrees = radToDeg * missile.bodyRate[0];
	missile.sinOfFourTimesPhiPrime = sin(4 * phiPrime);
	missile.squaredSinOfTwoTimesPhiPrime = pow((sin(2 * phiPrime)), 2);

}

void tableLookUps(Missile &missile)
{

	int index;

	index = missile.tableNameIndexPairs["CA0"];
	missile.CA0 = linearInterpolationWithBoundedEnds(missile.tables[index], missile.machSpeed);

	index = missile.tableNameIndexPairs["CAA"];
	missile.CAA = linearInterpolationWithBoundedEnds(missile.tables[index], missile.machSpeed);

	index = missile.tableNameIndexPairs["CAD"];
	missile.CAD = linearInterpolationWithBoundedEnds(missile.tables[index], missile.machSpeed);

	index = missile.tableNameIndexPairs["CAOFF"];
	if (missile.timeOfFlight <= ROCKET_BURN_OUT_TIME)
	{
		missile.CAOFF = 0.0;
	}
	else
	{
		missile.CAOFF = linearInterpolationWithBoundedEnds(missile.tables[index], missile.machSpeed);
	}

	index = missile.tableNameIndexPairs["CYP"];
	missile.CYP = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.machSpeed, missile.alphaPrimeDegrees);

	index = missile.tableNameIndexPairs["CYDR"];
	missile.CYDR = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.machSpeed, missile.alphaPrimeDegrees);

	index = missile.tableNameIndexPairs["CN0"];
	missile.CN0 = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.machSpeed, missile.alphaPrimeDegrees);

	index = missile.tableNameIndexPairs["CNP"];
	missile.CNP = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.machSpeed, missile.alphaPrimeDegrees);

	index = missile.tableNameIndexPairs["CNDQ"];
	missile.CNDQ = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.machSpeed, missile.alphaPrimeDegrees);

	index = missile.tableNameIndexPairs["CLLAP"];
	missile.CLLAP = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.machSpeed, missile.alphaPrimeDegrees);

	index = missile.tableNameIndexPairs["CLLP"];
	missile.CLLP = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.machSpeed, missile.alphaPrimeDegrees);

	index = missile.tableNameIndexPairs["CLLDP"];
	missile.CLLDP = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.machSpeed, missile.alphaPrimeDegrees);

	index = missile.tableNameIndexPairs["CLM0"];
	missile.CLM0 = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.machSpeed, missile.alphaPrimeDegrees);

	index = missile.tableNameIndexPairs["CLMP"];
	missile.CLMP = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.machSpeed, missile.alphaPrimeDegrees);

	index = missile.tableNameIndexPairs["CLMQ"];
	missile.CLMQ = linearInterpolationWithBoundedEnds(missile.tables[index], missile.machSpeed);

	index = missile.tableNameIndexPairs["CLMDQ"];
	missile.CLMDQ = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.machSpeed, missile.alphaPrimeDegrees);

	index = missile.tableNameIndexPairs["CLNP"];
	missile.CLNP = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.machSpeed, missile.alphaPrimeDegrees);

	index = missile.tableNameIndexPairs["MASS"];
	missile.mass = linearInterpolationWithBoundedEnds(missile.tables[index], missile.timeOfFlight);

	index = missile.tableNameIndexPairs["THRUST"];
	missile.unadjustedThrust = linearInterpolationWithBoundedEnds(missile.tables[index], missile.timeOfFlight);

	index = missile.tableNameIndexPairs["TMOI"];
	missile.transverseMomentOfInertia = linearInterpolationWithBoundedEnds(missile.tables[index], missile.timeOfFlight);

	index = missile.tableNameIndexPairs["AMOI"];
	missile.axialMomentOfInertia = linearInterpolationWithBoundedEnds(missile.tables[index], missile.timeOfFlight);

	index = missile.tableNameIndexPairs["CG"];
	missile.centerOfGravityFromNose = linearInterpolationWithBoundedEnds(missile.tables[index], missile.timeOfFlight);

}

void accelerationLimit(Missile &missile)
{

	int index;

	double currentAccelerationEstimate = missile.CN0 * missile.dynamicPressure * REFERENCE_AREA / missile.mass;

	index = missile.tableNameIndexPairs["CN0"];
	double CN0MAX = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.machSpeed, ALPHA_PRIME_MAX);

	double maximumAccelerationEstimate = CN0MAX * missile.dynamicPressure * REFERENCE_AREA / missile.mass;
	double availableAccelerationEstimate = maximumAccelerationEstimate - currentAccelerationEstimate;
	if (availableAccelerationEstimate < 0)
	{
		missile.maneuveringLimit = 1;
	}
	else if (availableAccelerationEstimate > MAXIMUM_ACCELERATION)
	{
		missile.maneuveringLimit = MAXIMUM_ACCELERATION;
	}
	else
	{
		missile.maneuveringLimit = availableAccelerationEstimate;
	}

}

void propulsion(Missile &missile)
{

	if (missile.timeOfFlight >= ROCKET_BURN_OUT_TIME)
	{
		missile.thrust = 0.0;
	}
	else
	{
		missile.thrust = missile.unadjustedThrust + (SEA_LEVEL_PRESSURE - missile.pressure) * THRUST_EXIT_AREA;
	}

}

void aerodynamicIntegrationCoefficients(Missile &missile)
{

	missile.CX = missile.CA0 + missile.CAA * missile.alphaPrimeDegrees + missile.CAD * (missile.totalFinDeflectionDegrees * missile.totalFinDeflectionDegrees) + missile.CAOFF;
	double CYAERO = missile.CYP * missile.sinOfFourTimesPhiPrime + missile.CYDR * missile.yawAeroBallisticFinDeflectionDegrees;
	double CZAERO = missile.CN0 + missile.CNP * missile.squaredSinOfTwoTimesPhiPrime + missile.CNDQ * missile.pitchAeroBallisticFinDeflectionDegrees;
	missile.CL = missile.CLLAP * missile.alphaPrimeDegrees * missile.alphaPrimeDegrees * missile.sinOfFourTimesPhiPrime + missile.CLLP * missile.rollRateDegrees * REFERENCE_DIAMETER / (2 * missile.speed) + missile.CLLDP * missile.rollFinDeflectionDegrees;
	double CNAEROREF = missile.CLNP * missile.sinOfFourTimesPhiPrime + missile.CLMQ * missile.yawAeroBallisticBodyRateDegrees * REFERENCE_DIAMETER / (2 * missile.speed) + missile.CLMDQ * missile.yawAeroBallisticFinDeflectionDegrees;
	double CNAERO = CNAEROREF - CYAERO * (LAUNCH_CENTER_OF_GRAVITY_FROM_NOSE - missile.centerOfGravityFromNose) / REFERENCE_DIAMETER;
	double CMAEROREF = missile.CLM0 + missile.CLMP * missile.squaredSinOfTwoTimesPhiPrime + missile.CLMQ * missile.pitchAeroBallisticBodyRateDegrees * REFERENCE_DIAMETER / (2 * missile.speed) + missile.CLMDQ * missile.pitchAeroBallisticFinDeflectionDegrees;
	double CMAERO = CMAEROREF - CZAERO * (LAUNCH_CENTER_OF_GRAVITY_FROM_NOSE - missile.centerOfGravityFromNose) / REFERENCE_DIAMETER;
	missile.CY = CYAERO * missile.cosPhiPrime - CZAERO * missile.sinPhiPrime;
	missile.CZ = CYAERO * missile.sinPhiPrime + CZAERO * missile.cosPhiPrime;
	missile.CN = CMAERO * missile.sinPhiPrime + CNAERO * missile.cosPhiPrime;
	missile.CM = CMAERO * missile.cosPhiPrime + CNAERO * missile.sinPhiPrime;

}

void aerodynamicFeedbackCoefficients(Missile &missile)
{

	int index;

	double alphaPrimeDegLookUp;
	if (missile.alphaPrimeDegrees > (ALPHA_PRIME_MAX - 3))
	{
		alphaPrimeDegLookUp = ALPHA_PRIME_MAX - 3;
	}
	else
	{
		alphaPrimeDegLookUp = missile.alphaPrimeDegrees;
	}
	double alphaPrimeDegMinusThree = alphaPrimeDegLookUp - 3;
	double alphaPrimeDegPlusThree = alphaPrimeDegLookUp + 3;
	index = missile.tableNameIndexPairs["CN0"];
	double CN0MIN = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.machSpeed, alphaPrimeDegMinusThree);
	double CN0MAX = biLinearInterpolationWithBoundedBorders(missile.tables[index],missile.machSpeed, alphaPrimeDegPlusThree);
	missile.CNA = ((CN0MAX - CN0MIN) / (alphaPrimeDegPlusThree - alphaPrimeDegMinusThree)) * radToDeg;
	index = missile.tableNameIndexPairs["CLM0"];
	double CLM0MIN = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.machSpeed, alphaPrimeDegMinusThree);
	double CLM0MAX = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.machSpeed, alphaPrimeDegPlusThree);
	missile.CMA = ((CLM0MAX - CLM0MIN) / (alphaPrimeDegPlusThree - alphaPrimeDegMinusThree) - (missile.CNA / radToDeg) * (LAUNCH_CENTER_OF_GRAVITY_FROM_NOSE - missile.centerOfGravityFromNose) / REFERENCE_DIAMETER) * radToDeg;
	missile.CND = missile.CNDQ * radToDeg;
	missile.CMD = missile.CLMDQ * radToDeg;
	missile.CMQ = missile.CLMQ * radToDeg;
	missile.CLP = missile.CLLP * radToDeg;
	missile.CLD = missile.CLLDP * radToDeg;
	missile.staticMargin = -1 * (missile.CMA * degToRad) / (missile.CNA * degToRad);

}

void eulerIntegrateStates(Missile &missile)
{

	missile.INTEGRATION_PASS = 0;

	setArrayEquivalentToReference(missile.P0, missile.ENUPosition);
	setArrayEquivalentToReference(missile.V0, missile.ENUVelocity);
	setArrayEquivalentToReference(missile.W0, missile.bodyRate);
	setArrayEquivalentToReference(missile.E0, missile.ENUEulerAngles);

	setArrayEquivalentToReference(missile.A1, missile.ENUAcceleration);
	setArrayEquivalentToReference(missile.WD1, missile.bodyRateDot);
	setArrayEquivalentToReference(missile.ED1, missile.ENUEulerDot);

	double deltaPos[3];
	multiplyVectorTimesScalar(TIME_STEP, missile.V0, deltaPos);
	addTwoVectors(missile.P0, deltaPos, missile.P1);

	double distanceTravelled;
	magnitude(deltaPos, distanceTravelled);
	missile.range += distanceTravelled;

	double deltaVel[3];
	multiplyVectorTimesScalar(TIME_STEP, missile.A1, deltaVel);
	addTwoVectors(missile.V0, deltaVel, missile.V1);

	double deltaOmega[3];
	multiplyVectorTimesScalar(TIME_STEP, missile.WD1, deltaOmega);
	addTwoVectors(missile.W0, deltaOmega, missile.W1);

	double deltaEuler[3];
	multiplyVectorTimesScalar(TIME_STEP, missile.ED1, deltaEuler);
	addTwoVectors(missile.E0, deltaEuler, missile.E1);

	setArrayEquivalentToReference(missile.ENUPosition, missile.P1);
	setArrayEquivalentToReference(missile.ENUVelocity, missile.V1);
	setArrayEquivalentToReference(missile.bodyRate, missile.W1);
	setArrayEquivalentToReference(missile.ENUEulerAngles, missile.E1);

	if (missile.launch)
	{
		missile.timeOfFlight += TIME_STEP;
	}

	setArrayEquivalentToZero(missile.P0);
	setArrayEquivalentToZero(missile.V0);
	setArrayEquivalentToZero(missile.W0);
	setArrayEquivalentToZero(missile.E0);

	setArrayEquivalentToZero(missile.A1);
	setArrayEquivalentToZero(missile.WD1);
	setArrayEquivalentToZero(missile.ED1);

	setArrayEquivalentToZero(missile.P1);
	setArrayEquivalentToZero(missile.V1);
	setArrayEquivalentToZero(missile.W1);
	setArrayEquivalentToZero(missile.E1);

}

void rk2IntegrateStates(Missile &missile)
{

	if (missile.INTEGRATION_PASS == 0)
	{

		missile.INTEGRATION_PASS += 1;

		setArrayEquivalentToReference(missile.P0, missile.ENUPosition);
		setArrayEquivalentToReference(missile.V0, missile.ENUVelocity);
		setArrayEquivalentToReference(missile.W0, missile.bodyRate);
		setArrayEquivalentToReference(missile.E0, missile.ENUEulerAngles);

		setArrayEquivalentToReference(missile.A1, missile.ENUAcceleration);
		setArrayEquivalentToReference(missile.WD1, missile.bodyRateDot);
		setArrayEquivalentToReference(missile.ED1, missile.ENUEulerDot);

		double deltaPos[3];
		multiplyVectorTimesScalar(HALF_TIME_STEP, missile.V0, deltaPos);
		addTwoVectors(missile.P0, deltaPos, missile.P1);

		double deltaVel[3];
		multiplyVectorTimesScalar(HALF_TIME_STEP, missile.A1, deltaVel);
		addTwoVectors(missile.V0, deltaVel, missile.V1);

		double deltaOmega[3];
		multiplyVectorTimesScalar(HALF_TIME_STEP, missile.WD1, deltaOmega);
		addTwoVectors(missile.W0, deltaOmega, missile.W1);

		double deltaEuler[3];
		multiplyVectorTimesScalar(HALF_TIME_STEP, missile.ED1, deltaEuler);
		addTwoVectors(missile.E0, deltaEuler, missile.E1);

		setArrayEquivalentToReference(missile.ENUPosition, missile.P1);
		setArrayEquivalentToReference(missile.ENUVelocity, missile.V1);
		setArrayEquivalentToReference(missile.bodyRate, missile.W1);
		setArrayEquivalentToReference(missile.ENUEulerAngles, missile.E1);

		if (missile.launch)
		{
			missile.timeOfFlight += HALF_TIME_STEP;
		}

	}
	else if (missile.INTEGRATION_PASS == 1)
	{

		missile.INTEGRATION_PASS = 0;

		setArrayEquivalentToReference(missile.A2, missile.ENUAcceleration);
		setArrayEquivalentToReference(missile.WD2, missile.bodyRateDot);
		setArrayEquivalentToReference(missile.ED2, missile.ENUEulerDot);

		double deltaPos[3];
		multiplyVectorTimesScalar(TIME_STEP, missile.V1, deltaPos);
		addTwoVectors(missile.P0, deltaPos, missile.P2);

		double distanceTravelled;
		magnitude(deltaPos, distanceTravelled);
		missile.range += distanceTravelled;

		double deltaVel[3];
		multiplyVectorTimesScalar(TIME_STEP, missile.A2, deltaVel);
		addTwoVectors(missile.V0, deltaVel, missile.V2);

		double deltaOmega[3];
		multiplyVectorTimesScalar(TIME_STEP, missile.WD2, deltaOmega);
		addTwoVectors(missile.W0, deltaOmega, missile.W2);

		double deltaEuler[3];
		multiplyVectorTimesScalar(TIME_STEP, missile.ED2, deltaEuler);
		addTwoVectors(missile.E0, deltaEuler, missile.E2);

		setArrayEquivalentToReference(missile.ENUPosition, missile.P2);
		setArrayEquivalentToReference(missile.ENUVelocity, missile.V2);
		setArrayEquivalentToReference(missile.bodyRate, missile.W2);
		setArrayEquivalentToReference(missile.ENUEulerAngles, missile.E2);

		if (missile.launch)
		{
			missile.timeOfFlight += HALF_TIME_STEP;
		}

		setArrayEquivalentToZero(missile.P0);
		setArrayEquivalentToZero(missile.V0);
		setArrayEquivalentToZero(missile.W0);
		setArrayEquivalentToZero(missile.E0);

		setArrayEquivalentToZero(missile.A1);
		setArrayEquivalentToZero(missile.WD1);
		setArrayEquivalentToZero(missile.ED1);

		setArrayEquivalentToZero(missile.P1);
		setArrayEquivalentToZero(missile.V1);
		setArrayEquivalentToZero(missile.W1);
		setArrayEquivalentToZero(missile.E1);

		setArrayEquivalentToZero(missile.A2);
		setArrayEquivalentToZero(missile.WD2);
		setArrayEquivalentToZero(missile.ED2);

		setArrayEquivalentToZero(missile.P2);
		setArrayEquivalentToZero(missile.V2);
		setArrayEquivalentToZero(missile.W2);
		setArrayEquivalentToZero(missile.E2);

	}

}

void rk4IntegrateStates(Missile &missile)
{

	if (missile.INTEGRATION_PASS == 0)
	{

		missile.INTEGRATION_PASS += 1;

		setArrayEquivalentToReference(missile.P0, missile.ENUPosition);
		setArrayEquivalentToReference(missile.V0, missile.ENUVelocity);
		setArrayEquivalentToReference(missile.W0, missile.bodyRate);
		setArrayEquivalentToReference(missile.E0, missile.ENUEulerAngles);

		setArrayEquivalentToReference(missile.A1, missile.ENUAcceleration);
		setArrayEquivalentToReference(missile.WD1, missile.bodyRateDot);
		setArrayEquivalentToReference(missile.ED1, missile.ENUEulerDot);

		double deltaPos[3];
		multiplyVectorTimesScalar(HALF_TIME_STEP, missile.V0, deltaPos);
		addTwoVectors(missile.P0, deltaPos, missile.P1);

		double deltaVel[3];
		multiplyVectorTimesScalar(HALF_TIME_STEP, missile.A1, deltaVel);
		addTwoVectors(missile.V0, deltaVel, missile.V1);

		double deltaOmega[3];
		multiplyVectorTimesScalar(HALF_TIME_STEP, missile.WD1, deltaOmega);
		addTwoVectors(missile.W0, deltaOmega, missile.W1);

		double deltaEuler[3];
		multiplyVectorTimesScalar(HALF_TIME_STEP, missile.ED1, deltaEuler);
		addTwoVectors(missile.E0, deltaEuler, missile.E1);

		setArrayEquivalentToReference(missile.ENUPosition, missile.P1);
		setArrayEquivalentToReference(missile.ENUVelocity, missile.V1);
		setArrayEquivalentToReference(missile.bodyRate, missile.W1);
		setArrayEquivalentToReference(missile.ENUEulerAngles, missile.E1);

		if (missile.launch)
		{
			missile.timeOfFlight += HALF_TIME_STEP;
		}

	}
	else if (missile.INTEGRATION_PASS == 1)
	{

		missile.INTEGRATION_PASS += 1;

		setArrayEquivalentToReference(missile.A2, missile.ENUAcceleration);
		setArrayEquivalentToReference(missile.WD2, missile.bodyRateDot);
		setArrayEquivalentToReference(missile.ED2, missile.ENUEulerDot);

		double deltaPos[3];
		multiplyVectorTimesScalar(HALF_TIME_STEP, missile.V1, deltaPos);
		addTwoVectors(missile.P0, deltaPos, missile.P2);

		double deltaVel[3];
		multiplyVectorTimesScalar(HALF_TIME_STEP, missile.A2, deltaVel);
		addTwoVectors(missile.V0, deltaVel, missile.V2);

		double deltaOmega[3];
		multiplyVectorTimesScalar(HALF_TIME_STEP, missile.WD2, deltaOmega);
		addTwoVectors(missile.W0, deltaOmega, missile.W2);

		double deltaEuler[3];
		multiplyVectorTimesScalar(HALF_TIME_STEP, missile.ED2, deltaEuler);
		addTwoVectors(missile.E0, deltaEuler, missile.E2);

		setArrayEquivalentToReference(missile.ENUPosition, missile.P2);
		setArrayEquivalentToReference(missile.ENUVelocity, missile.V2);
		setArrayEquivalentToReference(missile.bodyRate, missile.W2);
		setArrayEquivalentToReference(missile.ENUEulerAngles, missile.E2);

	}
	else if (missile.INTEGRATION_PASS == 2)
	{

		missile.INTEGRATION_PASS += 1;

		setArrayEquivalentToReference(missile.A3, missile.ENUAcceleration);
		setArrayEquivalentToReference(missile.WD3, missile.bodyRateDot);
		setArrayEquivalentToReference(missile.ED3, missile.ENUEulerDot);

		double deltaPos[3];
		multiplyVectorTimesScalar(TIME_STEP, missile.V2, deltaPos);
		addTwoVectors(missile.P0, deltaPos, missile.P3);

		double deltaVel[3];
		multiplyVectorTimesScalar(TIME_STEP, missile.A3, deltaVel);
		addTwoVectors(missile.V0, deltaVel, missile.V3);

		double deltaOmega[3];
		multiplyVectorTimesScalar(TIME_STEP, missile.WD3, deltaOmega);
		addTwoVectors(missile.W0, deltaOmega, missile.W3);

		double deltaEuler[3];
		multiplyVectorTimesScalar(TIME_STEP, missile.ED3, deltaEuler);
		addTwoVectors(missile.E0, deltaEuler, missile.E3);

		setArrayEquivalentToReference(missile.ENUPosition, missile.P3);
		setArrayEquivalentToReference(missile.ENUVelocity, missile.V3);
		setArrayEquivalentToReference(missile.bodyRate, missile.W3);
		setArrayEquivalentToReference(missile.ENUEulerAngles, missile.E3);

		if (missile.launch)
		{
			missile.timeOfFlight += HALF_TIME_STEP;
		}

	}
	else if (missile.INTEGRATION_PASS == 3)
	{

		missile.INTEGRATION_PASS = 0;

		setArrayEquivalentToReference(missile.A4, missile.ENUAcceleration);
		setArrayEquivalentToReference(missile.WD4, missile.bodyRateDot);
		setArrayEquivalentToReference(missile.ED4, missile.ENUEulerDot);

		double deltaPos[3];
		deltaPos[0] = (missile.V0[0] + missile.V1[0] * 2 + missile.V2[0] * 2 + missile.V3[0]) * (TIME_STEP / 6.0);
		deltaPos[1] = (missile.V0[1] + missile.V1[1] * 2 + missile.V2[1] * 2 + missile.V3[1]) * (TIME_STEP / 6.0);
		deltaPos[2] = (missile.V0[2] + missile.V1[2] * 2 + missile.V2[2] * 2 + missile.V3[2]) * (TIME_STEP / 6.0);
		addTwoVectors(missile.P0, deltaPos, missile.P4);

		double distanceTravelled;
		magnitude(deltaPos, distanceTravelled);
		missile.range += distanceTravelled;

		double deltaVel[3];
		deltaVel[0] = (missile.A1[0] + missile.A2[0] * 2 + missile.A3[0] * 2 + missile.A4[0]) * (TIME_STEP / 6.0);
		deltaVel[1] = (missile.A1[1] + missile.A2[1] * 2 + missile.A3[1] * 2 + missile.A4[1]) * (TIME_STEP / 6.0);
		deltaVel[2] = (missile.A1[2] + missile.A2[2] * 2 + missile.A3[2] * 2 + missile.A4[2]) * (TIME_STEP / 6.0);
		addTwoVectors(missile.V0, deltaVel, missile.V4);

		double deltaOmega[3];
		deltaOmega[0] = (missile.WD1[0] + missile.WD2[0] * 2 + missile.WD3[0] * 2 + missile.WD4[0]) * (TIME_STEP / 6.0);
		deltaOmega[1] = (missile.WD1[1] + missile.WD2[1] * 2 + missile.WD3[1] * 2 + missile.WD4[1]) * (TIME_STEP / 6.0);
		deltaOmega[2] = (missile.WD1[2] + missile.WD2[2] * 2 + missile.WD3[2] * 2 + missile.WD4[2]) * (TIME_STEP / 6.0);
		addTwoVectors(missile.W0, deltaOmega, missile.W4);

		double deltaEuler[3];
		deltaEuler[0] = (missile.ED1[0] + missile.ED2[0] * 2 + missile.ED3[0] * 2 + missile.ED4[0]) * (TIME_STEP / 6.0);
		deltaEuler[1] = (missile.ED1[1] + missile.ED2[1] * 2 + missile.ED3[1] * 2 + missile.ED4[1]) * (TIME_STEP / 6.0);
		deltaEuler[2] = (missile.ED1[2] + missile.ED2[2] * 2 + missile.ED3[2] * 2 + missile.ED4[2]) * (TIME_STEP / 6.0);
		addTwoVectors(missile.E0, deltaEuler, missile.E4);

		setArrayEquivalentToReference(missile.ENUPosition, missile.P4);
		setArrayEquivalentToReference(missile.ENUVelocity, missile.V4);
		setArrayEquivalentToReference(missile.bodyRate, missile.W4);
		setArrayEquivalentToReference(missile.ENUEulerAngles, missile.E4);

		setArrayEquivalentToZero(missile.P0);
		setArrayEquivalentToZero(missile.V0);
		setArrayEquivalentToZero(missile.W0);
		setArrayEquivalentToZero(missile.E0);

		setArrayEquivalentToZero(missile.A1);
		setArrayEquivalentToZero(missile.WD1);
		setArrayEquivalentToZero(missile.ED1);

		setArrayEquivalentToZero(missile.P1);
		setArrayEquivalentToZero(missile.V1);
		setArrayEquivalentToZero(missile.W1);
		setArrayEquivalentToZero(missile.E1);

		setArrayEquivalentToZero(missile.A2);
		setArrayEquivalentToZero(missile.WD2);
		setArrayEquivalentToZero(missile.ED2);

		setArrayEquivalentToZero(missile.P2);
		setArrayEquivalentToZero(missile.V2);
		setArrayEquivalentToZero(missile.W2);
		setArrayEquivalentToZero(missile.E2);

		setArrayEquivalentToZero(missile.A3);
		setArrayEquivalentToZero(missile.WD3);
		setArrayEquivalentToZero(missile.ED3);

		setArrayEquivalentToZero(missile.P3);
		setArrayEquivalentToZero(missile.V3);
		setArrayEquivalentToZero(missile.W3);
		setArrayEquivalentToZero(missile.E3);

		setArrayEquivalentToZero(missile.A4);
		setArrayEquivalentToZero(missile.WD4);
		setArrayEquivalentToZero(missile.ED4);

		setArrayEquivalentToZero(missile.P4);
		setArrayEquivalentToZero(missile.V4);
		setArrayEquivalentToZero(missile.W4);
		setArrayEquivalentToZero(missile.E4);

	}

}

void missileMotion(Missile &missile)
{

	// Forces.
	double axialForce = missile.thrust - missile.CX * missile.dynamicPressure * REFERENCE_AREA + missile.FLUGravity[0] * missile.mass;
	double sideForce = missile.CY * missile.dynamicPressure * REFERENCE_AREA + missile.FLUGravity[1] * missile.mass;
	double normalForce = missile.CZ * missile.dynamicPressure * REFERENCE_AREA + missile.FLUGravity[2] * missile.mass;

	// Moments.
	double rollMoment = missile.CL * missile.dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER;
	double pitchMoment = missile.CM * missile.dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER;
	double yawMoment = missile.CN * missile.dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER;

	// Specific force.
	missile.FLUAcceleration[0] = axialForce / missile.mass - (missile.bodyRate[1] * missile.FLUVelocity[2] - missile.bodyRate[2] * missile.FLUVelocity[1]);
	missile.FLUAcceleration[1] = sideForce / missile.mass - (missile.bodyRate[2] * missile.FLUVelocity[0] - missile.bodyRate[0] * missile.FLUVelocity[2]);
	missile.FLUAcceleration[2] = normalForce / missile.mass - (missile.bodyRate[0] * missile.FLUVelocity[1] - missile.bodyRate[1] * missile.FLUVelocity[0]);

	// Rotate FLU acceleration into ENU acceleration.
	oneByThreeTimesThreeByThree(missile.FLUAcceleration, missile.missileENUToFLUMatrix, missile.ENUAcceleration);

	// Omega dot.
	missile.bodyRateDot[0] = rollMoment / missile.axialMomentOfInertia;
	missile.bodyRateDot[1] = (1 / missile.transverseMomentOfInertia) * ((missile.transverseMomentOfInertia - missile.axialMomentOfInertia) * missile.bodyRate[0] * missile.bodyRate[2] + pitchMoment);
	missile.bodyRateDot[2] = (1 / missile.transverseMomentOfInertia) * ((missile.axialMomentOfInertia - missile.transverseMomentOfInertia) * missile.bodyRate[0] * missile.bodyRate[1] + yawMoment);

	// Euler dot.
	missile.ENUEulerDot[0] = missile.bodyRate[0] + (missile.bodyRate[1] * sin(missile.ENUEulerAngles[0]) + missile.bodyRate[2] * cos(missile.ENUEulerAngles[0])) * tan(missile.ENUEulerAngles[1]);
	missile.ENUEulerDot[1] = missile.bodyRate[1] * cos(missile.ENUEulerAngles[0]) - missile.bodyRate[2] * sin(missile.ENUEulerAngles[0]);
	missile.ENUEulerDot[2] = -1 * (missile.bodyRate[1] * sin(missile.ENUEulerAngles[0]) + missile.bodyRate[2] * cos(missile.ENUEulerAngles[0])) / cos(missile.ENUEulerAngles[1]);

	// Integrate states.
	if (missile.INTEGRATION_METHOD == 0)
	{
		eulerIntegrateStates(missile);
	}
	else if (missile.INTEGRATION_METHOD == 1)
	{
		rk2IntegrateStates(missile);
	}
	else if (missile.INTEGRATION_METHOD == 2)
	{
		rk4IntegrateStates(missile);
	}

	// Adjust local to body direction cosine matrix.
	eulerAnglesToLocalOrientation(
		missile.ENUEulerAngles[0],
		-missile.ENUEulerAngles[1],
		missile.ENUEulerAngles[2],
		missile.missileENUToFLUMatrix
	);

	threeByThreeTimesThreeByOne(missile.missileENUToFLUMatrix, missile.ENUVelocity, missile.FLUVelocity);

}

void performanceAndTerminationCheck(Missile &missile, double maxTime)
{

	magnitude(missile.FLUMissileToPipRelativePosition, missile.missDistance);

	if (missile.ENUPosition[2] < 0)
	{
		missile.lethality = "GROUND_COLLISION";
	}
	else if (missile.missDistance < 2.0)
	{
		missile.lethality = "SUCCESSFUL_INTERCEPT";
	}
	else if (missile.FLUMissileToPipRelativePosition[0] < 0.0)
	{
		missile.lethality = "POINT_OF_CLOSEST_APPROACH_PASSED";
	}
	else if (isnan(missile.ENUPosition[0]))
	{
		missile.lethality = "NOT_A_NUMBER";
	}
	else if (missile.timeOfFlight > maxTime)
	{
		missile.lethality = "MAX_TIME_EXCEEDED";
	}

}

void writeLogFileHeader(ofstream &logFile)
{

	// Logging everything.
	logFile << fixed << setprecision(10) <<
	"tgtE" <<
	" " << "tgtN" <<
	" " << "tgtU" <<
	" " << "tof" <<
	" " << "posE" <<
	" " << "posN" <<
	" " << "posU" <<
	" " << "range" <<
	" " << "velE" <<
	" " << "velN" <<
	" " << "velU" <<
	" " << "u" <<
	" " << "v" <<
	" " << "w" <<
	" " << "speed" <<
	" " << "mach" <<
	" " << "accE" <<
	" " << "accN" <<
	" " << "accU" <<
	" " << "udot" <<
	" " << "vdot" <<
	" " << "wdot" <<
	" " << "ENUToFLU_0_0" <<
	" " << "ENUToFLU_0_1" <<
	" " << "ENUToFLU_0_2" <<
	" " << "ENUToFLU_1_0" <<
	" " << "ENUToFLU_1_1" <<
	" " << "ENUToFLU_1_2" <<
	" " << "ENUToFLU_2_0" <<
	" " << "ENUToFLU_2_1" <<
	" " << "ENUToFLU_2_2" <<
	" " << "alphaRadians" <<
	" " << "betaRadians" <<
	" " << "alphaDegrees" <<
	" " << "betaDegrees" <<
	" " << "phi" <<
	" " << "theta" <<
	" " << "psi" <<
	" " << "phiDot" <<
	" " << "thetaDot" <<
	" " << "psiDot" <<
	" " << "p" <<
	" " << "q" <<
	" " << "r" <<
	" " << "pdot" <<
	" " << "qdot" <<
	" " << "rdot" <<
	" " << "gravity" <<
	" " << "axialGravity" <<
	" " << "sideGravity" <<
	" " << "normalGravity" <<
	" " << "pressure" <<
	" " << "dynamicPressure" <<
	" " << "seekerPitch" <<
	" " << "seekerYaw" <<
	" " << "seekerENUToFLU_0_0" <<
	" " << "seekerENUToFLU_0_1" <<
	" " << "seekerENUToFLU_0_2" <<
	" " << "seekerENUToFLU_1_0" <<
	" " << "seekerENUToFLU_1_1" <<
	" " << "seekerENUToFLU_1_2" <<
	" " << "seekerENUToFLU_2_0" <<
	" " << "seekerENUToFLU_2_1" <<
	" " << "seekerENUToFLU_2_2" <<
	" " << "seekerPitchError" <<
	" " << "seekerYawError" <<
	" " << "seekerWLR" <<
	" " << "seekerWLRD" <<
	" " << "seekerWLR1" <<
	" " << "seekerWLR1D" <<
	" " << "seekerWLR2" <<
	" " << "seekerWLR2D" <<
	" " << "seekerWLQ" <<
	" " << "seekerWLQD" <<
	" " << "seekerWLQ1" <<
	" " << "seekerWLQ1D" <<
	" " << "seekerWLQ2" <<
	" " << "seekerWLQ2D" <<
	" " << "missileToInterceptRelativePositionForward"
	" " << "missileToInterceptRelativePositionLeft"
	" " << "missileToInterceptRelativePositionUp"
	" " << "guidanceNormalCommand" <<
	" " << "guidanceSideCommand" <<
	" " << "accelerationLimit" <<
	" " << "controlYY" <<
	" " << "controlYYD" <<
	" " << "controlZZ" <<
	" " << "controlZZD" <<
	" " << "rollFinCommand" <<
	" " << "pitchFinCommand" <<
	" " << "yawFinCommand" <<
	" " << "rollFinDeflection" <<
	" " << "pitchFinDeflection" <<
	" " << "yawFinDeflection" <<
	" " << "finOneDeflection" <<
	" " << "finOneDeflectionDerived" <<
	" " << "finOneRate" <<
	" " << "finOneRateDerived" <<
	" " << "finTwoDeflection" <<
	" " << "finTwoDeflectionDerived" <<
	" " << "finTwoRate" <<
	" " << "finTwoRateDerived" <<
	" " << "finThreeDeflection" <<
	" " << "finThreeDeflectionDerived" <<
	" " << "finThreeRate" <<
	" " << "finThreeRateDerived" <<
	" " << "finFourDeflection" <<
	" " << "finFourDeflectionDerived" <<
	" " << "finFourRate" <<
	" " << "finFourRateDerived" <<
	" " << "alphaPrimeRadians" <<
	" " << "alphaPrimeDegrees"
	" " << "sinPhiPrime" <<
	" " << "cosPhiPrime" <<
	" " << "rollFinDeflectionDegrees" <<
	" " << "pitchFinDeflectionDegreesAeroBallisticFrame" <<
	" " << "yawFinDeflectionDegreesAeroBallisticFrame" <<
	" " << "totalFinDeflectionDegrees" <<
	" " << "pitchRateDegreesAeroBallisticFrame" <<
	" " << "yawRateDegreesAeroBallisticFrame" <<
	" " << "rollRateDegrees" <<
	" " << "sinOfFourTimesPhiPrime" <<
	" " << "squaredSinOfTwoTimesPhiPrime"
	" " << "CA0" <<
	" " << "CAA" <<
	" " << "CAD" <<
	" " << "CAOFF" <<
	" " << "CYP" <<
	" " << "CYDR" <<
	" " << "CN0" <<
	" " << "CNP" <<
	" " << "CNDQ" <<
	" " << "CLLAP" <<
	" " << "CLLP" <<
	" " << "CLLDP" <<
	" " << "CLM0" <<
	" " << "CLMP" <<
	" " << "CLMQ" <<
	" " << "CLMDQ" <<
	" " << "CLNP" <<
	" " << "mass" <<
	" " << "unadjustedThrust" <<
	" " << "transverseMomentOfInertia" <<
	" " << "axialMomentOfInertia" <<
	" " << "centerOfGravityFromNose" <<
	" " << "thrust" <<
	" " << "CX" <<
	" " << "CY" <<
	" " << "CZ" <<
	" " << "CL" <<
	" " << "CM" <<
	" " << "CN" <<
	" " << "CNA" <<
	" " << "CMA" <<
	" " << "CND" <<
	" " << "CMD" <<
	" " << "CMQ" <<
	" " << "CLP" <<
	" " << "CLD" <<
	" " << "staticMargin" <<
	" " << "missDistance" <<
	" " << "lethality" <<
	" " << "launch" <<
	"\n";

}

void logData(Missile &missile, ofstream &logFile)
{

	logFile << fixed << setprecision(10) <<
	missile.pip[0] << " " <<
	missile.pip[1] << " " <<
	missile.pip[2] << " " <<
	missile.timeOfFlight << " " <<
	missile.ENUPosition[0] << " " <<
	missile.ENUPosition[1] << " " <<
	missile.ENUPosition[2] << " " <<
	missile.range << " " <<
	missile.ENUVelocity[0] << " " <<
	missile.ENUVelocity[1] << " " <<
	missile.ENUVelocity[2] << " " <<
	missile.FLUVelocity[0] << " " <<
	missile.FLUVelocity[1] << " " <<
	missile.FLUVelocity[2] << " " <<
	missile.speed << " " <<
	missile.machSpeed << " " <<
	missile.ENUAcceleration[0] << " " <<
	missile.ENUAcceleration[1] << " " <<
	missile.ENUAcceleration[2] << " " <<
	missile.FLUAcceleration[0] << " " <<
	missile.FLUAcceleration[1] << " " <<
	missile.FLUAcceleration[2] << " " <<
	missile.missileENUToFLUMatrix[0][0] << " " <<
	missile.missileENUToFLUMatrix[0][1] << " " <<
	missile.missileENUToFLUMatrix[0][2] << " " <<
	missile.missileENUToFLUMatrix[1][0] << " " <<
	missile.missileENUToFLUMatrix[1][1] << " " <<
	missile.missileENUToFLUMatrix[1][2] << " " <<
	missile.missileENUToFLUMatrix[2][0] << " " <<
	missile.missileENUToFLUMatrix[2][1] << " " <<
	missile.missileENUToFLUMatrix[2][2] << " " <<
	missile.alphaRadians << " " <<
	missile.betaRadians << " " <<
	missile.alphaDegrees << " " <<
	missile.betaDegrees << " " <<
	missile.ENUEulerAngles[0] << " " <<
	missile.ENUEulerAngles[1] << " " <<
	missile.ENUEulerAngles[2] << " " <<
	missile.ENUEulerDot[0] << " " <<
	missile.ENUEulerDot[1] << " " <<
	missile.ENUEulerDot[2] << " " <<
	missile.bodyRate[0] << " " <<
	missile.bodyRate[1] << " " <<
	missile.bodyRate[2] << " " <<
	missile.bodyRateDot[0] << " " <<
	missile.bodyRateDot[1] << " " <<
	missile.bodyRateDot[2] << " " <<
	missile.grav << " " <<
	missile.FLUGravity[0] << " " <<
	missile.FLUGravity[1] << " " <<
	missile.FLUGravity[2] << " " <<
	missile.pressure << " " <<
	missile.dynamicPressure << " " <<
	missile.seekerPitch << " " <<
	missile.seekerYaw << " " <<
	missile.seekerENUToFLUMatrix[0][0] << " " <<
	missile.seekerENUToFLUMatrix[0][1] << " " <<
	missile.seekerENUToFLUMatrix[0][2] << " " <<
	missile.seekerENUToFLUMatrix[1][0] << " " <<
	missile.seekerENUToFLUMatrix[1][1] << " " <<
	missile.seekerENUToFLUMatrix[1][2] << " " <<
	missile.seekerENUToFLUMatrix[2][0] << " " <<
	missile.seekerENUToFLUMatrix[2][1] << " " <<
	missile.seekerENUToFLUMatrix[2][2] << " " <<
	missile.seekerPitchError << " " <<
	missile.seekerYawError << " " <<
	missile.seekerWLR << " " <<
	missile.seekerWLRD << " " <<
	missile.seekerWLR1 << " " <<
	missile.seekerWLR1D << " " <<
	missile.seekerWLR2 << " " <<
	missile.seekerWLR2D << " " <<
	missile.seekerWLQ << " " <<
	missile.seekerWLQD << " " <<
	missile.seekerWLQ1 << " " <<
	missile.seekerWLQ1D << " " <<
	missile.seekerWLQ2 << " " <<
	missile.seekerWLQ2D << " " <<
	missile.FLUMissileToPipRelativePosition[0] << " " <<
	missile.FLUMissileToPipRelativePosition[1] << " " <<
	missile.FLUMissileToPipRelativePosition[2] << " " <<
	missile.guidanceNormalCommand << " " <<
	missile.guidanceSideCommand << " " <<
	missile.maneuveringLimit << " " <<
	missile.yawControlFeedForwardIntegration << " " <<
	missile.yawControlFeedForwardDerivative << " " <<
	missile.pitchControlFeedForwardIntegration << " " <<
	missile.pitchControlFeedForwardDerivative << " " <<
	missile.rollFinCommand << " " <<
	missile.pitchFinCommand << " " <<
	missile.yawFinCommand << " " <<
	missile.rollFinDeflection << " " <<
	missile.pitchFinDeflection << " " <<
	missile.yawFinDeflection << " " <<
	missile.FIN1DEFL << " " <<
	missile.FIN1DEFL_D << " " <<
	missile.FIN1DEFL_DOT << " " <<
	missile.FIN1DEFL_DOTDOT << " " <<
	missile.FIN2DEFL << " " <<
	missile.FIN2DEFL_D << " " <<
	missile.FIN2DEFL_DOT << " " <<
	missile.FIN2DEFL_DOTDOT << " " <<
	missile.FIN3DEFL << " " <<
	missile.FIN3DEFL_D << " " <<
	missile.FIN3DEFL_DOT << " " <<
	missile.FIN3DEFL_DOTDOT << " " <<
	missile.FIN4DEFL << " " <<
	missile.FIN4DEFL_D << " " <<
	missile.FIN4DEFL_DOT << " " <<
	missile.FIN4DEFL_DOTDOT << " " <<
	missile.alphaPrimeRadians << " " <<
	missile.alphaPrimeDegrees << " " <<
	missile.sinPhiPrime << " " <<
	missile.cosPhiPrime << " " <<
	missile.rollFinDeflectionDegrees << " " <<
	missile.pitchAeroBallisticFinDeflectionDegrees << " " <<
	missile.yawAeroBallisticFinDeflectionDegrees << " " <<
	missile.totalFinDeflectionDegrees << " " <<
	missile.pitchAeroBallisticBodyRateDegrees << " " <<
	missile.yawAeroBallisticBodyRateDegrees << " " <<
	missile.rollRateDegrees << " " <<
	missile.sinOfFourTimesPhiPrime << " " <<
	missile.squaredSinOfTwoTimesPhiPrime << " " <<
	missile.CA0 << " " <<
	missile.CAA << " " <<
	missile.CAD << " " <<
	missile.CAOFF << " " <<
	missile.CYP << " " <<
	missile.CYDR << " " <<
	missile.CN0 << " " <<
	missile.CNP << " " <<
	missile.CNDQ << " " <<
	missile.CLLAP << " " <<
	missile.CLLP << " " <<
	missile.CLLDP << " " <<
	missile.CLM0 << " " <<
	missile.CLMP << " " <<
	missile.CLMQ << " " <<
	missile.CLMDQ << " " <<
	missile.CLNP << " " <<
	missile.mass << " " <<
	missile.unadjustedThrust << " " <<
	missile.transverseMomentOfInertia << " " <<
	missile.axialMomentOfInertia << " " <<
	missile.centerOfGravityFromNose << " " <<
	missile.thrust << " " <<
	missile.CX << " " <<
	missile.CY << " " <<
	missile.CZ << " " <<
	missile.CL << " " <<
	missile.CM << " " <<
	missile.CN << " " <<
	missile.CNA << " " <<
	missile.CMA << " " <<
	missile.CND << " " <<
	missile.CMD << " " <<
	missile.CMQ << " " <<
	missile.CLP << " " <<
	missile.CLD << " " <<
	missile.staticMargin << " " <<
	missile.missDistance << " " <<
	missile.lethality << " " <<
	missile.launch <<
	"\n";

}

void flyout(Missile &missile, string flyOutID, bool writeData, bool consoleReport, double maxTime, int INTEGRATION_METHOD)
{

	// Change missile integration method to the requested flyout integration method.
	missile.INTEGRATION_METHOD = INTEGRATION_METHOD;

	// For console report if requested.
	double lastTime = missile.timeOfFlight;

	// For log file if requested.
	ofstream logFile;

	if (writeData)
	{

		logFile.open(flyOutID + ".txt");
		writeLogFileHeader(logFile);

	}

	if (consoleReport)
	{

		cout << "FLIGHT" << endl;

	}

	while (missile.lethality == "FLYING")
	{

		atmosphere(missile);
		seeker(missile);
		guidance(missile);
		control(missile);
		actuators(missile);
		aerodynamicAnglesAndConversions(missile);
		tableLookUps(missile);
		accelerationLimit(missile);
		propulsion(missile);
		aerodynamicIntegrationCoefficients(missile);
		aerodynamicFeedbackCoefficients(missile);
		missileMotion(missile);
		if (missile.INTEGRATION_PASS == 0)
		{
			performanceAndTerminationCheck(missile, maxTime);

			if (writeData)
			{

				logData(missile, logFile);

			}
			
			if (consoleReport)
			{

				auto print_it = static_cast<int>(round(missile.timeOfFlight * 10000.0)) % 10000;
				if (print_it == 0)
				{
					cout << setprecision(6) << missile.timeOfFlight << " E " << missile.ENUPosition[0] << " N " << missile.ENUPosition[1] << " U " << missile.ENUPosition[2] << " RANGE " << missile.range << " MACH " << missile.machSpeed << endl;
					lastTime = missile.timeOfFlight;
				}

			}
			
		}

	}

	missile.lethality = "FLYING"; // Reset lethality in the case that this missile is copied and needed again.
	cout << "\n"; // For a better console report.

}

int main ()
{

	double lastTime = 0;

	Missile Missile1;
	initUnLaunchedMissile(Missile1);
	flyout(Missile1, "firstFiveSeconds", false, true, 5.0, 0);

	Missile Missile2 = Missile1; // Missile copies will need a specific init at some point, for containment and midcourse reevaluation.
	flyout(Missile2, "log", true, true, 400.0, 0);

	cout << "\n" << endl;
	cout << "MISSION REPORT" << endl;
	cout << setprecision(6) << "FINAL POSITION AT " << Missile2.timeOfFlight << " E " << Missile2.ENUPosition[0] << " N " << Missile2.ENUPosition[1] << " U " << Missile2.ENUPosition[2] << " RANGE " << Missile2.range << " MACH " << Missile2.machSpeed << endl;
	cout << setprecision(6) << "MISS DISTANCE " << Missile2.missDistance << " FORWARD, LEFT, UP, MISS DISTANCE " << Missile2.FLUMissileToPipRelativePosition[0] << " " << Missile2.FLUMissileToPipRelativePosition[1] << " " << Missile2.FLUMissileToPipRelativePosition[2] << endl;
	cout << "SIMULATION RESULT: " << Missile2.lethality << endl;
	auto wallClockEnd = chrono::high_resolution_clock::now();
	auto simRealRunTime = chrono::duration_cast<chrono::milliseconds>(wallClockEnd - wallClockStart);
	cout << "SIMULATION RUN TIME :" << simRealRunTime.count() << " MILLISECONDS" << endl;
	cout << "\n" << endl;
	return 0;

}