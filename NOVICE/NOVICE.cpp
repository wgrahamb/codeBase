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
// Add rk2 and rk4 integration methods.
// Add blended guidance.
// Move all variables to a missile struct, which is passed to all functions.
// Begin on NOVICE algorithms.

/*
# Missile Model:
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

// Simulation control.
auto wallClockStart = chrono::high_resolution_clock::now(); // Start tracking real time.
const double TIME_STEP = 0.001; // Seconds. Common sense to have a uniform time step if can.
const double HALF_TIME_STEP = TIME_STEP / 2.0; // Seconds. For rk2 and rk4 integration.
const double MAX_TIME = 400; // Seconds.

/* Missile constants. */
const double REFERENCE_AREA = 0.01824; // Meters^2.
const double REFERENCE_DIAMETER = 0.1524; // Meters.
const double THRUST_EXIT_AREA = 0.0125; // Meters^2.
const double ROCKET_BURN_OUT_TIME = 2.421; // Seconds.
const double SEEKER_KF_G = 10.0; // Seeker Kalman filter gain. Per second.
const double SEEKER_KF_ZETA = 0.9; // Seeker Kalman filter damping. Non dimensional.
const double SEEKER_KF_WN = 60.0; // Seeker Kalman filter natural frequency. Radians per second.
const double PROPORTIONAL_GUIDANCE_GAIN = 3.0; // Guidance homing gain. Non dimensional.
const double LINE_OF_ATTACK_GUIDANCE_GAIN = 1.5; // Guidance midcourse gain. Non dimensional.
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

/* Variables */
double pip[3]; // Predicted Intercept Point. Meters.

// Missile state.
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
double alpha = 0.0; // Radians.
double beta = 0.0; // Radians.
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
double alphaPrimeDegress = 0.0; // Degrees.
double sinPhiPrime = 0.0; // Non dimensional.
double cosPhiPrime = 0.0; // Non dimensional.
double pitchAeroBallisticFinDeflectionDegrees = 0.0; // Degrees.
double yawAeroBallisticFinDeflectionDegrees = 0.0; // Degrees.
double rollDeflectionDegrees = 0.0; // Degrees.
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

// Log file.
ofstream logFile;

void lookUpTablesFormat (string dataFile)
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
							tables[tableNoTrack - 1][0].back() = dataPointDouble;
						}
						// IF THIS THE FIRST LOOP, THIS IS THE COLUMN IN THE DATA SET THAT DISPLAYS THE "ROWS" VALUES
						else if (columnCount == 1)
						{
							// FOR TWO DIMENSIONAL TABLE
							if (dimensions[tableNoTrack -1][1] != 2)
							{
								// PLACE DATA POINT IN ITS PLACE
								tables[tableNoTrack - 1][rowNoTrack][0] = dataPointDouble;
							}
							// FOR ONE DIMENSIONAL TABLE
							else
							{
								// PLACE DATA POINT IN ITS PLACE
								tables[tableNoTrack - 1][rowNoTrack - 1][0] = dataPointDouble;
							}
						}
						// IF THIS THE SECOND LOOP, THIS IS THE COLUMN IN THE DATA SET THAT DISPLAYS THE "COLUMNS" VALUES, ONLY FOR TWO DIMENSIONAL TABLES
						else if (columnCount == 2 and dimensions[tableNoTrack -1][1] != 2)
						{
							// PLACE DATA POINT IN ITS PLACE
							tables[tableNoTrack - 1][0][rowNoTrack] = dataPointDouble;
						}
						// ELSE FOR ACTUAL DATA POINTS
						else
						{
							// FOR TWO DIMENSIONAL TABLES
							if (dimensions[tableNoTrack -1][1] != 2)
							{
								// PLACE DATA POINT IN ITS PLACE
								tables[tableNoTrack - 1][rowNoTrack][columnCount - 2] = dataPointDouble;
							}
							// FOR ONE DIMENSIONAL TABLES
							else
							{
								// PLACE DATA POINT IN ITS PLACE
								tables[tableNoTrack - 1][rowNoTrack - 1][columnCount - 1] = dataPointDouble;
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
			tables.push_back(newTable);
		}
		// STORE NAME OF TABLE
		else if (flag == 3)
		{
			// MAP TABLE NAME INDEX PAIR
			tableNameIndexPairs.emplace(name, tableNoTrack - 1);
		}
	}
}

// Works for now.
// Needs to be refactored for fire control.
// This will be done by the launcher, when the scenario is created.
// When a missile packet is copied for flyouts, this does not have to be done.
void init()
{

	// Intitialize and open input file.
	std::ifstream inPut;
	inPut.open("input.txt");

	// Declare inputs.
	double phi, theta, psi, tgtE, tgtN, tgtU;

	// Populate inputs from file.
	inPut >> phi >> theta >> psi >> tgtE >> tgtN >> tgtU;

	// Set pip.
	// Will be changed for inputs.
	pip[0] = tgtE; // Meters.
	pip[1] = tgtN; // Meters.
	pip[2] = tgtU; // Meters.

	phi *= degToRad;
	theta *= degToRad;
	psi *= degToRad;

	// Missile.
	ENUEulerAngles[0] = phi;
	ENUEulerAngles[1] = theta;
	ENUEulerAngles[2] = psi;
	eulerAnglesToLocalOrientation(phi, -theta, psi, missileENUToFLUMatrix);
	ENUPosition[0] = 0.0;
	ENUPosition[1] = 0.0;
	ENUPosition[2] = 0.0;
	ENUVelocity[0] = missileENUToFLUMatrix[0][0];
	ENUVelocity[1] = missileENUToFLUMatrix[0][1];
	ENUVelocity[2] = missileENUToFLUMatrix[0][2];
	threeByThreeTimesThreeByOne(missileENUToFLUMatrix, ENUVelocity, FLUVelocity);
	ENUAcceleration[0] = 0.0;
	ENUAcceleration[1] = 0.0;
	ENUAcceleration[2] = 0.0;
	FLUAcceleration[0] = 0.0;
	FLUAcceleration[1] = 0.0;
	FLUAcceleration[2] = 0.0;
	magnitude(ENUVelocity, speed);

	// Intialize seeker by pointing it directly at the target.
	double relPos[3];
	subtractTwoVectors(ENUPosition, pip, relPos);
	double relPosU[3];
	unitVec(relPos, relPosU);
	double mslToInterceptU[3];
	threeByThreeTimesThreeByOne(missileENUToFLUMatrix, relPosU, mslToInterceptU);
	double mslToInterceptAz, mslToInterceptEl;
	azAndElFromVector(mslToInterceptAz, mslToInterceptEl, mslToInterceptU);
	seekerPitch = mslToInterceptEl;
	seekerYaw = mslToInterceptAz;
	double seekerAttitudeToLocalTM[3][3];
	eulerAnglesToLocalOrientation(0.0, -seekerPitch, seekerYaw, seekerAttitudeToLocalTM);
	threeByThreeTimesThreeByThree(seekerAttitudeToLocalTM, missileENUToFLUMatrix, seekerENUToFLUMatrix);
	seekerPitchError = 0.0;
	seekerYawError = 0.0;
	seekerWLR = seekerYaw;
	seekerWLQ = seekerPitch;

	// Format data tables.
	lookUpTablesFormat("shortRangeInterceptorTables.txt");

	// Set missile lethality.
	lethality = "FLYING"; // STATUS

	// Write header for log file.
	logFile.open("log.txt");
	logFile << fixed << setprecision(10) << "tof posE posN posU tgtE tgtN tgtU normComm normAch pitchRate thetaRate pitchDefl alpha theta sideComm sideAch yawRate psiRate yawDefl beta psi rollComm phiRate rollRate rollDefl roll seekPitchErr seekYawErr staticMargin mach" << endl;

	// Console output.
	cout << "\n" << endl;
	cout << "MODEL INITIATED" << endl;
	cout << "\n" << endl;

}

void atmosphere()
{

	int index;

	double alt = ENUPosition[2] * mToKm;

	index = tableNameIndexPairs["RHO"];
	double rho = linearInterpolationWithBoundedEnds(tables[index], alt); // Kilograms per meter^3.

	index = tableNameIndexPairs["GRAVITY"];
	grav = linearInterpolationWithBoundedEnds(tables[index], alt); // Meters per second^2.

	double gravLocalVec[3] = {0.0, 0.0, -grav};
	threeByThreeTimesThreeByOne(missileENUToFLUMatrix, gravLocalVec, FLUGravity);

	index = tableNameIndexPairs["PRESSURE"];
	pressure = linearInterpolationWithBoundedEnds(tables[index], alt);

	index = tableNameIndexPairs["SPEED_OF_SOUND"];
	double a = linearInterpolationWithBoundedEnds(tables[index], alt); // Meters per second^2.

	magnitude(ENUVelocity, speed);
	machSpeed = speed / a;
	dynamicPressure = 0.5 * rho * speed * speed;

}

void seeker()
{

	double wsq = SEEKER_KF_WN * SEEKER_KF_WN;
	double gg = SEEKER_KF_G * wsq;

	// Yaw channel.
	double wlr1d_new = seekerWLR2;
	double wlr1_new = trapezoidIntegrate(wlr1d_new, seekerWLR1D, seekerWLR1, TIME_STEP);
	seekerWLR1 = wlr1_new;
	seekerWLR1D = wlr1d_new;
	double wlr2d_new = gg * seekerYawError - 2 * SEEKER_KF_ZETA * SEEKER_KF_WN * seekerWLR1D - wsq * seekerWLR1;
	double wlr2_new = trapezoidIntegrate(wlr2d_new, seekerWLR2D, seekerWLR2, TIME_STEP);
	seekerWLR2 = wlr2_new;
	seekerWLR2D = wlr2d_new;

	// Yaw control.
	double wlrd_new = seekerWLR1 - bodyRate[2];
	double wlr_new = trapezoidIntegrate(wlrd_new, seekerWLRD, seekerWLR, TIME_STEP);
	seekerWLR = wlr_new;
	seekerWLRD = wlrd_new;
	seekerYaw = seekerWLR;

	// Pitch channel.
	double wlq1d_new = seekerWLQ2;
	double wlq1_new = trapezoidIntegrate(wlq1d_new, seekerWLQ1D, seekerWLQ1, TIME_STEP);
	seekerWLQ1 = wlq1_new;
	seekerWLQ1D = wlq1d_new;
	double wlq2d_new = gg * seekerPitchError - 2 * SEEKER_KF_ZETA * SEEKER_KF_WN * seekerWLQ1D - wsq * seekerWLQ1;
	double wlq2_new = trapezoidIntegrate(wlq2d_new, seekerWLQ2D, seekerWLQ2, TIME_STEP);
	seekerWLQ2 = wlq2_new;
	seekerWLQ2D = wlq2d_new;

	// Pitch control.
	double wlqd_new = seekerWLQ1 - bodyRate[1];
	double wlq_new = trapezoidIntegrate(wlqd_new, seekerWLQD, seekerWLQ, TIME_STEP);
	seekerWLQ = wlq_new;
	seekerWLQD = wlqd_new;
	seekerPitch = seekerWLQ;

	// Convert seeker data to FLU relative position for guidance.
	double localRelPos[3];
	subtractTwoVectors(ENUPosition, pip, localRelPos);
	double seekerAttitudeToLocalTM[3][3];
	eulerAnglesToLocalOrientation(0.0, -seekerPitch, seekerYaw, seekerAttitudeToLocalTM);
	threeByThreeTimesThreeByThree(seekerAttitudeToLocalTM, missileENUToFLUMatrix, seekerENUToFLUMatrix);
	double seekerToInterceptRelPos[3];
	threeByThreeTimesThreeByOne(seekerENUToFLUMatrix, localRelPos, seekerToInterceptRelPos);
	double inducedErr[3] = {1.0, 0.5, 0.2};
	double seekerToInterceptRelPosWithErr[3];
	multiplyTwoVectors(seekerToInterceptRelPos, inducedErr, seekerToInterceptRelPosWithErr);
	azAndElFromVector(seekerYawError, seekerPitchError, seekerToInterceptRelPosWithErr);
	oneByThreeTimesThreeByThree(seekerToInterceptRelPosWithErr, seekerAttitudeToLocalTM, FLUMissileToPipRelativePosition);

}

void guidance()
{

	double forwardLeftUpMslToInterceptPosU[3];
	double forwardLeftUpMslToInterceptMag;
	unitVec(FLUMissileToPipRelativePosition, forwardLeftUpMslToInterceptPosU);
	magnitude(FLUMissileToPipRelativePosition, forwardLeftUpMslToInterceptMag);
	double relVel[3];
	relVel[0] = ENUVelocity[0] * -1;
	relVel[1] = ENUVelocity[1] * -1;
	relVel[2] = ENUVelocity[2] * -1;
	double relVelMag;
	magnitude(relVel, relVelMag);
	double closingVel[3];
	threeByThreeTimesThreeByOne(missileENUToFLUMatrix, relVel, closingVel);
	double TEMP1[3], TEMP2;
	crossProductTwoVectors(FLUMissileToPipRelativePosition, closingVel, TEMP1);
	dotProductTwoVectors(FLUMissileToPipRelativePosition, FLUMissileToPipRelativePosition, TEMP2);
	double lineOfSightRate[3];
	lineOfSightRate[0] = TEMP1[0] / TEMP2;
	lineOfSightRate[1] = TEMP1[1] / TEMP2;
	lineOfSightRate[2] = TEMP1[2] / TEMP2;
	double command[3];
	double TEMP3[3];
	TEMP3[0] = -1 * PROPORTIONAL_GUIDANCE_GAIN * relVelMag * forwardLeftUpMslToInterceptPosU[0];
	TEMP3[1] = -1 * PROPORTIONAL_GUIDANCE_GAIN * relVelMag * forwardLeftUpMslToInterceptPosU[1];
	TEMP3[2] = -1 * PROPORTIONAL_GUIDANCE_GAIN * relVelMag * forwardLeftUpMslToInterceptPosU[2];
	crossProductTwoVectors(TEMP3, lineOfSightRate, command);
	guidanceNormalCommand = command[2];
	guidanceSideCommand = command[1];
	double trigRatio = atan2(guidanceNormalCommand, guidanceSideCommand);
	double accMag = sqrt(guidanceNormalCommand * guidanceNormalCommand + guidanceSideCommand * guidanceSideCommand);
	if (accMag > maneuveringLimit)
	{
		accMag = maneuveringLimit;
	}
	guidanceSideCommand = accMag * cos(trigRatio);
	guidanceNormalCommand = accMag * sin(trigRatio);

}

void control()
{

	if (machSpeed > 0.6)
	{

		// Aerodynamic feedback.
		double DNA = CNA * (dynamicPressure * REFERENCE_AREA / mass);
		double DMA = CMA * (dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / transverseMomentOfInertia);
		double DMD = CMD * (dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / transverseMomentOfInertia);
		double DMQ = CMQ * (REFERENCE_DIAMETER / (2 * speed)) * (dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / transverseMomentOfInertia);
		double DLP = CLP * (REFERENCE_DIAMETER / (2 * speed)) * (dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / axialMomentOfInertia);
		double DLD = CLD * (dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / axialMomentOfInertia);

		// Natural frequency and damping scheduling.
		double WACL = 0.013 * sqrt(dynamicPressure) + 7.1;
		double ZACL = 0.000559 * sqrt(dynamicPressure) + 0.232;
		double PACL = 14;

		// Feedback gain scheduling.
		double GAINFB3 = WACL * WACL * PACL / (DNA * DMD);
		double GAINFB2 = (2 * ZACL * WACL + PACL + DMQ - DNA / speed) / DMD;
		double GAINFB1 = (
			WACL * WACL +
			2 * ZACL * WACL * PACL +
			DMA +
			DMQ * DNA / speed -
			GAINFB2 * DMD * DNA / speed
		) / (DNA * DMD);

		// Roll control.
		double GKP = (2 * ROLL_CONTROL_WN * ROLL_CONTROL_ZETA + DLP) / DLD;
		double GKPHI = ROLL_CONTROL_WN * ROLL_CONTROL_WN / DLD;
		double EPHI = GKPHI * (ROLL_ANGLE_COMMAND - ENUEulerAngles[0]);
		rollFinCommand = EPHI - GKP * bodyRate[0];

		// Pitch control.
		double zzdNew = guidanceNormalCommand - FLUAcceleration[2];
		double zzNew = trapezoidIntegrate(
			zzdNew,
			pitchControlFeedForwardDerivative,
			pitchControlFeedForwardIntegration,
			TIME_STEP
		);
		pitchControlFeedForwardIntegration = zzNew;
		pitchControlFeedForwardDerivative = zzdNew;
		double deflPitch = -1 * GAINFB1 * FLUAcceleration[2] - GAINFB2 * bodyRate[1] + GAINFB3 * pitchControlFeedForwardIntegration;
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
		pitchFinCommand = deflPitch * degToRad;

		// Yaw control.
		double yydNew = FLUAcceleration[1] - guidanceSideCommand;
		double yyNew = trapezoidIntegrate(
			yydNew,
			yawControlFeedForwardDerivative,
			yawControlFeedForwardIntegration,
			TIME_STEP
		);
		yawControlFeedForwardIntegration = yyNew;
		yawControlFeedForwardDerivative = yydNew;
		double deflYaw = GAINFB1 * FLUAcceleration[1] - GAINFB2 * bodyRate[2] + GAINFB3 * yawControlFeedForwardIntegration;
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
		yawFinCommand = deflYaw * degToRad;

	}

	else if (machSpeed > 0.01)
	{

		// Aerodynamic feedback.
		double DNA = CNA * (dynamicPressure * REFERENCE_AREA / mass);
		double DND = CND * (dynamicPressure * REFERENCE_AREA / mass);
		double DMA = CMA * (dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / transverseMomentOfInertia);
		double DMD = CMD * (dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / transverseMomentOfInertia);
		double DMQ = CMQ * (REFERENCE_DIAMETER / (2 * speed)) * (dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / transverseMomentOfInertia);
		double DLP = CLP * (REFERENCE_DIAMETER / (2 * speed)) * (dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / axialMomentOfInertia);
		double DLD = CLD * (dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / axialMomentOfInertia);

		// Roll control.
		double GKP = (2 * ROLL_CONTROL_WN * ROLL_CONTROL_ZETA + DLP) / DLD;
		double GKPHI = ROLL_CONTROL_WN * ROLL_CONTROL_WN / DLD;
		double EPHI = GKPHI * (ROLL_ANGLE_COMMAND - ENUEulerAngles[0]);
		rollFinCommand = EPHI - GKP * bodyRate[0];

		// Rate control.
		double ZRATE = DNA / speed - DMA * DND / (speed * DMD);
		double AA = DNA / speed - DMQ;
		double BB = -1 * DMA - DMQ * DNA / speed;
		double TEMP1 = AA - 2 * RATE_CONTROL_ZETA * RATE_CONTROL_ZETA * ZRATE;
		double TEMP2 = AA * AA - 4 * RATE_CONTROL_ZETA * RATE_CONTROL_ZETA * BB;
		double RADIX = TEMP1 * TEMP1 - TEMP2;
		double GRATE = (-1 * TEMP1 + sqrt(RADIX)) / (-1 * DMD);

		// Pitch control.
		pitchFinCommand = GRATE * bodyRate[1]; // Radians.

		// Yaw control.
		yawFinCommand = GRATE * bodyRate[2]; // Radians.

	}
	
	else
	{

		rollFinCommand = 0.0;
		pitchFinCommand = 0.0;
		yawFinCommand = 0.0;

	}

}

void actuators()
{

	// Fin commands.
	double DEL1C = -rollFinCommand + pitchFinCommand - yawFinCommand;
	double DEL2C = -rollFinCommand + pitchFinCommand + yawFinCommand;
	double DEL3C = rollFinCommand + pitchFinCommand - yawFinCommand;
	double DEL4C = rollFinCommand + pitchFinCommand + yawFinCommand;

	int flag;

	// Fin one.
	flag = 0;
	if (abs(FIN1DEFL) > FIN_CONTROL_MAX_DEFLECTION_RADIANS)
	{
		if (FIN1DEFL < 0)
		{
			FIN1DEFL = -1 * FIN_CONTROL_MAX_DEFLECTION_RADIANS;
		}
		else if (FIN1DEFL > 0)
		{
			FIN1DEFL = FIN_CONTROL_MAX_DEFLECTION_RADIANS;
		}
		if ((FIN1DEFL * FIN1DEFL_DOT) > 0)
		{
			FIN1DEFL_DOT = 0;
		}
	}
	if (abs(FIN1DEFL_DOT) > FIN_RATE_LIMIT_RADIANS)
	{
		flag = 1;
		if (FIN1DEFL_DOT < 0)
		{
			FIN1DEFL_DOT = -1 * FIN_RATE_LIMIT_RADIANS;
		}
		else if (FIN1DEFL_DOT > 0)
		{
			FIN1DEFL_DOT = FIN_RATE_LIMIT_RADIANS;
		}
	}
	double DEL1D_NEW = FIN1DEFL_DOT;
	double DEL1_NEW = trapezoidIntegrate(DEL1D_NEW, FIN1DEFL_D, FIN1DEFL, TIME_STEP);
	FIN1DEFL = DEL1_NEW;
	FIN1DEFL_D = DEL1D_NEW;
	double EDX1 = DEL1C - FIN1DEFL;
	double DEL1DOTDOT_NEW = FIN_CONTROL_WN * FIN_CONTROL_WN * EDX1 - 2 * FIN_CONTROL_ZETA * FIN_CONTROL_WN * FIN1DEFL_D;
	double DEL1DOT_NEW = trapezoidIntegrate(DEL1DOTDOT_NEW, FIN1DEFL_DOTDOT, FIN1DEFL_DOT, TIME_STEP);
	FIN1DEFL_DOT = DEL1DOT_NEW;
	FIN1DEFL_DOTDOT = DEL1DOTDOT_NEW;
	if (flag == 1 and (FIN1DEFL_DOT * FIN1DEFL_DOTDOT) > 0)
	{
		FIN1DEFL_DOTDOT = 0.0;
	}

	// Fin two.
	flag = 0;
	if (abs(FIN2DEFL) > FIN_CONTROL_MAX_DEFLECTION_RADIANS)
	{
		if (FIN2DEFL < 0)
		{
			FIN2DEFL = -1 * FIN_CONTROL_MAX_DEFLECTION_RADIANS;
		}
		else if (FIN2DEFL > 0)
		{
			FIN2DEFL = FIN_CONTROL_MAX_DEFLECTION_RADIANS;
		}
		if ((FIN2DEFL * FIN2DEFL_DOT) > 0)
		{
			FIN2DEFL_DOT = 0;
		}
	}
	if (abs(FIN2DEFL_DOT) > FIN_RATE_LIMIT_RADIANS)
	{
		flag = 1;
		if (FIN2DEFL_DOT < 0)
		{
			FIN2DEFL_DOT = -1 * FIN_RATE_LIMIT_RADIANS;
		}
		else if (FIN2DEFL_DOT > 0)
		{
			FIN2DEFL_DOT = FIN_RATE_LIMIT_RADIANS;
		}
	}
	double DEL2D_NEW = FIN2DEFL_DOT;
	double DEL2_NEW = trapezoidIntegrate(DEL2D_NEW, FIN2DEFL_D, FIN2DEFL, TIME_STEP);
	FIN2DEFL = DEL2_NEW;
	FIN2DEFL_D = DEL2D_NEW;
	double EDX2 = DEL2C - FIN2DEFL;
	double DEL2DOTDOT_NEW = FIN_CONTROL_WN * FIN_CONTROL_WN * EDX2 - 2 * FIN_CONTROL_ZETA * FIN_CONTROL_WN * FIN2DEFL_D;
	double DEL2DOT_NEW = trapezoidIntegrate(DEL2DOTDOT_NEW, FIN2DEFL_DOTDOT, FIN2DEFL_DOT, TIME_STEP);
	FIN2DEFL_DOT = DEL2DOT_NEW;
	FIN2DEFL_DOTDOT = DEL2DOTDOT_NEW;
	if (flag == 1 and (FIN2DEFL_DOT * FIN2DEFL_DOTDOT) > 0)
	{
		FIN2DEFL_DOTDOT = 0.0;
	}

	// Fin three.
	flag = 0;
	if (abs(FIN3DEFL) > FIN_CONTROL_MAX_DEFLECTION_RADIANS)
	{
		if (FIN3DEFL < 0)
		{
			FIN3DEFL = -1 * FIN_CONTROL_MAX_DEFLECTION_RADIANS;
		}
		else if (FIN3DEFL > 0)
		{
			FIN3DEFL = FIN_CONTROL_MAX_DEFLECTION_RADIANS;
		}
		if ((FIN3DEFL * FIN3DEFL_DOT) > 0)
		{
			FIN3DEFL_DOT = 0;
		}
	}
	if (abs(FIN3DEFL_DOT) > FIN_RATE_LIMIT_RADIANS)
	{
		flag = 1;
		if (FIN3DEFL_DOT < 0)
		{
			FIN3DEFL_DOT = -1 * FIN_RATE_LIMIT_RADIANS;
		}
		else if (FIN3DEFL_DOT > 0)
		{
			FIN3DEFL_DOT = FIN_RATE_LIMIT_RADIANS;
		}
	}
	double DEL3D_NEW = FIN3DEFL_DOT;
	double DEL3_NEW = trapezoidIntegrate(DEL3D_NEW, FIN3DEFL_D, FIN3DEFL, TIME_STEP);
	FIN3DEFL = DEL3_NEW;
	FIN3DEFL_D = DEL3D_NEW;
	double EDX3 = DEL3C - FIN3DEFL;
	double DEL3DOTDOT_NEW = FIN_CONTROL_WN * FIN_CONTROL_WN * EDX3 - 2 * FIN_CONTROL_ZETA * FIN_CONTROL_WN * FIN3DEFL_D;
	double DEL3DOT_NEW = trapezoidIntegrate(DEL3DOTDOT_NEW, FIN3DEFL_DOTDOT, FIN3DEFL_DOT, TIME_STEP);
	FIN3DEFL_DOT = DEL3DOT_NEW;
	FIN3DEFL_DOTDOT = DEL3DOTDOT_NEW;
	if (flag == 1 and (FIN3DEFL_DOT * FIN3DEFL_DOTDOT) > 0)
	{
		FIN3DEFL_DOTDOT = 0.0;
	}

	// Fin four.
	flag = 0;
	if (abs(FIN4DEFL) > FIN_CONTROL_MAX_DEFLECTION_RADIANS)
	{
		if (FIN4DEFL < 0)
		{
			FIN4DEFL = -1 * FIN_CONTROL_MAX_DEFLECTION_RADIANS;
		}
		else if (FIN4DEFL > 0)
		{
			FIN4DEFL = FIN_CONTROL_MAX_DEFLECTION_RADIANS;
		}
		if ((FIN4DEFL * FIN4DEFL_DOT) > 0)
		{
			FIN4DEFL_DOT = 0;
		}
	}
	if (abs(FIN4DEFL_DOT) > FIN_RATE_LIMIT_RADIANS)
	{
		flag = 1;
		if (FIN4DEFL_DOT < 0)
		{
			FIN4DEFL_DOT = -1 * FIN_RATE_LIMIT_RADIANS;
		}
		else if (FIN4DEFL_DOT > 0)
		{
			FIN4DEFL_DOT = FIN_RATE_LIMIT_RADIANS;
		}
	}
	double DEL4D_NEW = FIN4DEFL_DOT;
	double DEL4_NEW = trapezoidIntegrate(DEL4D_NEW, FIN4DEFL_D, FIN4DEFL, TIME_STEP);
	FIN4DEFL = DEL4_NEW;
	FIN4DEFL_D = DEL4D_NEW;
	double EDX4 = DEL4C - FIN4DEFL;
	double DEL4DOTDOT_NEW = FIN_CONTROL_WN * FIN_CONTROL_WN * EDX4 - 2 * FIN_CONTROL_ZETA * FIN_CONTROL_WN * FIN4DEFL_D;
	double DEL4DOT_NEW = trapezoidIntegrate(DEL4DOTDOT_NEW, FIN4DEFL_DOTDOT, FIN4DEFL_DOT, TIME_STEP);
	FIN4DEFL_DOT = DEL4DOT_NEW;
	FIN4DEFL_DOTDOT = DEL4DOTDOT_NEW;
	if (flag == 1 and (FIN4DEFL_DOT * FIN4DEFL_DOTDOT) > 0)
	{
		FIN4DEFL_DOTDOT = 0.0;
	}

	// Attitude fin deflections.
	rollFinDeflection = (-FIN1DEFL - FIN2DEFL + FIN3DEFL + FIN4DEFL) / 4;
	pitchFinDeflection = (FIN1DEFL + FIN2DEFL + FIN3DEFL + FIN4DEFL) / 4;
	yawFinDeflection = (-FIN1DEFL + FIN2DEFL - FIN3DEFL + FIN4DEFL) / 4;

}

void aerodynamicAnglesAndConversions()
{

	alpha = -1 * atan2(FLUVelocity[2], FLUVelocity[0]);
	beta = atan2(FLUVelocity[1], FLUVelocity[0]);
	double alphaPrime = acos(cos(alpha) * cos(beta));
	alphaPrimeDegress = radToDeg * alphaPrime;
	double phiPrime = atan2(tan(beta), sin(alpha));
	sinPhiPrime = sin(phiPrime);
	cosPhiPrime = cos(phiPrime);
	double pitchDeflAeroFrame = pitchFinDeflection * cosPhiPrime - yawFinDeflection * sinPhiPrime;
	pitchAeroBallisticFinDeflectionDegrees = radToDeg * pitchDeflAeroFrame;
	double yawDeflAeroFrame = pitchFinDeflection * sinPhiPrime + yawFinDeflection * cosPhiPrime;
	yawAeroBallisticFinDeflectionDegrees = radToDeg * yawDeflAeroFrame;
	rollDeflectionDegrees = radToDeg * rollFinDeflection;
	totalFinDeflectionDegrees = (abs(pitchAeroBallisticFinDeflectionDegrees) + abs(yawAeroBallisticFinDeflectionDegrees)) / 2;
	double pitchRateAeroFrame = bodyRate[1] * cosPhiPrime - bodyRate[2] * sinPhiPrime;
	pitchAeroBallisticBodyRateDegrees = radToDeg * pitchRateAeroFrame;
	double yawRateAeroFrame = bodyRate[1] * sinPhiPrime + bodyRate[2] * cosPhiPrime;
	yawAeroBallisticBodyRateDegrees = radToDeg * yawRateAeroFrame;
	rollRateDegrees = radToDeg * bodyRate[0];
	sinOfFourTimesPhiPrime = sin(4 * phiPrime);
	squaredSinOfTwoTimesPhiPrime = pow((sin(2 * phiPrime)), 2);

}

void tableLookUps()
{

	int index;

	index = tableNameIndexPairs["CA0"];
	CA0 = linearInterpolationWithBoundedEnds(tables[index], machSpeed);

	index = tableNameIndexPairs["CAA"];
	CAA = linearInterpolationWithBoundedEnds(tables[index], machSpeed);

	index = tableNameIndexPairs["CAD"];
	CAD = linearInterpolationWithBoundedEnds(tables[index], machSpeed);

	index = tableNameIndexPairs["CAOFF"];
	if (timeOfFlight <= ROCKET_BURN_OUT_TIME)
	{
		CAOFF = 0.0;
	}
	else
	{
		CAOFF = linearInterpolationWithBoundedEnds(tables[index], machSpeed);
	}

	index = tableNameIndexPairs["CYP"];
	CYP = biLinearInterpolationWithBoundedBorders(tables[index], machSpeed, alphaPrimeDegress);

	index = tableNameIndexPairs["CYDR"];
	CYDR = biLinearInterpolationWithBoundedBorders(tables[index], machSpeed, alphaPrimeDegress);

	index = tableNameIndexPairs["CN0"];
	CN0 = biLinearInterpolationWithBoundedBorders(tables[index], machSpeed, alphaPrimeDegress);

	index = tableNameIndexPairs["CNP"];
	CNP = biLinearInterpolationWithBoundedBorders(tables[index], machSpeed, alphaPrimeDegress);

	index = tableNameIndexPairs["CNDQ"];
	CNDQ = biLinearInterpolationWithBoundedBorders(tables[index], machSpeed, alphaPrimeDegress);

	index = tableNameIndexPairs["CLLAP"];
	CLLAP = biLinearInterpolationWithBoundedBorders(tables[index], machSpeed, alphaPrimeDegress);

	index = tableNameIndexPairs["CLLP"];
	CLLP = biLinearInterpolationWithBoundedBorders(tables[index], machSpeed, alphaPrimeDegress);

	index = tableNameIndexPairs["CLLDP"];
	CLLDP = biLinearInterpolationWithBoundedBorders(tables[index], machSpeed, alphaPrimeDegress);

	index = tableNameIndexPairs["CLM0"];
	CLM0 = biLinearInterpolationWithBoundedBorders(tables[index], machSpeed, alphaPrimeDegress);

	index = tableNameIndexPairs["CLMP"];
	CLMP = biLinearInterpolationWithBoundedBorders(tables[index], machSpeed, alphaPrimeDegress);

	index = tableNameIndexPairs["CLMQ"];
	CLMQ = linearInterpolationWithBoundedEnds(tables[index], machSpeed);

	index = tableNameIndexPairs["CLMDQ"];
	CLMDQ = biLinearInterpolationWithBoundedBorders(tables[index], machSpeed, alphaPrimeDegress);

	index = tableNameIndexPairs["CLNP"];
	CLNP = biLinearInterpolationWithBoundedBorders(tables[index], machSpeed, alphaPrimeDegress);

	index = tableNameIndexPairs["MASS"];
	mass = linearInterpolationWithBoundedEnds(tables[index], timeOfFlight);

	index = tableNameIndexPairs["THRUST"];
	unadjustedThrust = linearInterpolationWithBoundedEnds(tables[index], timeOfFlight);

	index = tableNameIndexPairs["TMOI"];
	transverseMomentOfInertia = linearInterpolationWithBoundedEnds(tables[index], timeOfFlight);

	index = tableNameIndexPairs["AMOI"];
	axialMomentOfInertia = linearInterpolationWithBoundedEnds(tables[index], timeOfFlight);

	index = tableNameIndexPairs["CG"];
	centerOfGravityFromNose = linearInterpolationWithBoundedEnds(tables[index], timeOfFlight);

}

void accelerationLimit()
{

	int index;

	double currentAccelerationEstimate = CN0 * dynamicPressure * REFERENCE_AREA / mass;

	index = tableNameIndexPairs["CN0"];
	double CN0MAX = biLinearInterpolationWithBoundedBorders(tables[index], machSpeed, ALPHA_PRIME_MAX);

	double maximumAccelerationEstimate = CN0MAX * dynamicPressure * REFERENCE_AREA / mass;
	double availableAccelerationEstimate = maximumAccelerationEstimate - currentAccelerationEstimate;
	if (availableAccelerationEstimate < 0)
	{
		maneuveringLimit = 1;
	}
	else if (availableAccelerationEstimate > MAXIMUM_ACCELERATION)
	{
		maneuveringLimit = MAXIMUM_ACCELERATION;
	}
	else
	{
		maneuveringLimit = availableAccelerationEstimate;
	}

}

void propulsion()
{

	if (timeOfFlight >= ROCKET_BURN_OUT_TIME)
	{
		thrust = 0.0;
	}
	else
	{
		thrust = unadjustedThrust + (SEA_LEVEL_PRESSURE - pressure) * THRUST_EXIT_AREA;
	}

}

void aerodynamicIntegrationCoefficients()
{

	CX = CA0 + CAA * alphaPrimeDegress + CAD * (totalFinDeflectionDegrees * totalFinDeflectionDegrees) + CAOFF;
	double CYAERO = CYP * sinOfFourTimesPhiPrime + CYDR * yawAeroBallisticFinDeflectionDegrees;
	double CZAERO = CN0 + CNP * squaredSinOfTwoTimesPhiPrime + CNDQ * pitchAeroBallisticFinDeflectionDegrees;
	CL = CLLAP * pow(alphaPrimeDegress, 2) * sinOfFourTimesPhiPrime + CLLP * rollRateDegrees * REFERENCE_DIAMETER / (2 * speed) + CLLDP * rollDeflectionDegrees;
	double CNAEROREF = CLNP * sinOfFourTimesPhiPrime + CLMQ * yawAeroBallisticBodyRateDegrees * REFERENCE_DIAMETER / (2 * speed) + CLMDQ * yawAeroBallisticFinDeflectionDegrees;
	double CNAERO = CNAEROREF - CYAERO * (LAUNCH_CENTER_OF_GRAVITY_FROM_NOSE - centerOfGravityFromNose) / REFERENCE_DIAMETER;
	double CMAEROREF = CLM0 + CLMP * squaredSinOfTwoTimesPhiPrime + CLMQ * pitchAeroBallisticBodyRateDegrees * REFERENCE_DIAMETER / (2 * speed) + CLMDQ * pitchAeroBallisticFinDeflectionDegrees;
	double CMAERO = CMAEROREF - CZAERO * (LAUNCH_CENTER_OF_GRAVITY_FROM_NOSE - centerOfGravityFromNose) / REFERENCE_DIAMETER;
	CY = CYAERO * cosPhiPrime - CZAERO * sinPhiPrime;
	CZ = CYAERO * sinPhiPrime + CZAERO * cosPhiPrime;
	CN = CMAERO * sinPhiPrime + CNAERO * cosPhiPrime;
	CM = CMAERO * cosPhiPrime + CNAERO * sinPhiPrime;

}

void aerodynamicFeedbackCoefficients()
{

	int index;

	double alphaPrimeDegLookUp;
	if (alphaPrimeDegress > (ALPHA_PRIME_MAX - 3))
	{
		alphaPrimeDegLookUp = ALPHA_PRIME_MAX - 3;
	}
	else
	{
		alphaPrimeDegLookUp = alphaPrimeDegress;
	}
	double alphaPrimeDegMinusThree = alphaPrimeDegLookUp - 3;
	double alphaPrimeDegPlusThree = alphaPrimeDegLookUp + 3;
	index = tableNameIndexPairs["CN0"];
	double CN0MIN = biLinearInterpolationWithBoundedBorders(tables[index], machSpeed, alphaPrimeDegMinusThree);
	double CN0MAX = biLinearInterpolationWithBoundedBorders(tables[index],machSpeed, alphaPrimeDegPlusThree);
	CNA = ((CN0MAX - CN0MIN) / (alphaPrimeDegPlusThree - alphaPrimeDegMinusThree)) * radToDeg;
	index = tableNameIndexPairs["CLM0"];
	double CLM0MIN = biLinearInterpolationWithBoundedBorders(tables[index], machSpeed, alphaPrimeDegMinusThree);
	double CLM0MAX = biLinearInterpolationWithBoundedBorders(tables[index], machSpeed, alphaPrimeDegPlusThree);
	CMA = ((CLM0MAX - CLM0MIN) / (alphaPrimeDegPlusThree - alphaPrimeDegMinusThree) - (CNA / radToDeg) * (LAUNCH_CENTER_OF_GRAVITY_FROM_NOSE - centerOfGravityFromNose) / REFERENCE_DIAMETER) * radToDeg;
	CND = CNDQ * radToDeg;
	CMD = CLMDQ * radToDeg;
	CMQ = CLMQ * radToDeg;
	CLP = CLLP * radToDeg;
	CLD = CLLDP * radToDeg;
	staticMargin = -1 * (CMA * degToRad) / (CNA * degToRad);

}

void eulerIntegrateStates()
{

	timeOfFlight += TIME_STEP;

	double axialForce = thrust - CX * dynamicPressure * REFERENCE_AREA + FLUGravity[0] * mass;
	double sideForce = CY * dynamicPressure * REFERENCE_AREA + FLUGravity[1] * mass;
	double normalForce = CZ * dynamicPressure * REFERENCE_AREA + FLUGravity[2] * mass;

	double rollMoment = CL * dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER;
	double pitchMoment = CM * dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER;
	double yawMoment = CN * dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER;

	FLUAcceleration[0] = axialForce / mass - (bodyRate[1] * FLUVelocity[2] - bodyRate[2] * FLUVelocity[1]);
	FLUAcceleration[1] = sideForce / mass - (bodyRate[2] * FLUVelocity[0] - bodyRate[0] * FLUVelocity[2]);
	FLUAcceleration[2] = normalForce / mass - (bodyRate[0] * FLUVelocity[1] - bodyRate[1] * FLUVelocity[0]);

	oneByThreeTimesThreeByThree(FLUAcceleration, missileENUToFLUMatrix, ENUAcceleration);

	bodyRateDot[0] = rollMoment / axialMomentOfInertia;
	bodyRateDot[1] = (1 / transverseMomentOfInertia) * ((transverseMomentOfInertia - axialMomentOfInertia) * bodyRate[0] * bodyRate[2] + pitchMoment);
	bodyRateDot[2] = (1 / transverseMomentOfInertia) * ((axialMomentOfInertia - transverseMomentOfInertia) * bodyRate[0] * bodyRate[1] + yawMoment);

	ENUEulerDot[0] = bodyRate[0] + (bodyRate[1] * sin(ENUEulerAngles[0]) + bodyRate[2] * cos(ENUEulerAngles[0])) * tan(ENUEulerAngles[1]);
	ENUEulerDot[1] = bodyRate[1] * cos(ENUEulerAngles[0]) - bodyRate[2] * sin(ENUEulerAngles[0]);
	ENUEulerDot[2] = -1 * (bodyRate[1] * sin(ENUEulerAngles[0]) + bodyRate[2] * cos(ENUEulerAngles[0])) / cos(ENUEulerAngles[1]);

	double deltaPos[3];
	multiplyVectorTimesScalar(TIME_STEP, ENUVelocity, deltaPos);
	double newMslPos[3];
	addTwoVectors(ENUPosition, deltaPos, newMslPos);
	ENUPosition[0] = newMslPos[0];
	ENUPosition[1] = newMslPos[1];
	ENUPosition[2] = newMslPos[2];

	double distanceTravelled;
	magnitude(deltaPos, distanceTravelled);
	range += distanceTravelled;

	double deltaVel[3];
	multiplyVectorTimesScalar(TIME_STEP, ENUAcceleration, deltaVel);
	double newMslVel[3];
	addTwoVectors(ENUVelocity, deltaVel, newMslVel);
	ENUVelocity[0] = newMslVel[0];
	ENUVelocity[1] = newMslVel[1];
	ENUVelocity[2] = newMslVel[2];

	threeByThreeTimesThreeByOne(missileENUToFLUMatrix, ENUVelocity, FLUVelocity);

	double deltaOmega[3];
	multiplyVectorTimesScalar(TIME_STEP, bodyRateDot, deltaOmega);
	double newMslBodyRate[3];
	addTwoVectors(bodyRate, deltaOmega, newMslBodyRate);
	bodyRate[0] = newMslBodyRate[0];
	bodyRate[1] = newMslBodyRate[1];
	bodyRate[2] = newMslBodyRate[2];

	double deltaEuler[3];
	multiplyVectorTimesScalar(TIME_STEP, ENUEulerDot, deltaEuler);
	double newMslEuler[3];
	addTwoVectors(ENUEulerAngles, deltaEuler, newMslEuler);
	ENUEulerAngles[0] = newMslEuler[0];
	ENUEulerAngles[1] = newMslEuler[1];
	ENUEulerAngles[2] = newMslEuler[2];

	eulerAnglesToLocalOrientation(
		ENUEulerAngles[0],
		-ENUEulerAngles[1],
		ENUEulerAngles[2],
		missileENUToFLUMatrix
	);

}

void performanceAndTerminationCheck()
{

	magnitude(FLUMissileToPipRelativePosition, missDistance);

	if (ENUPosition[2] < 0)
	{
		lethality = "GROUND COLLISION";
	}
	else if (missDistance < 2.0)
	{
		lethality = "SUCCESSFUL INTERCEPT";
	}
	else if (FLUMissileToPipRelativePosition[0] < 0.0)
	{
		lethality = "POINT OF CLOSEST APPROACH PASSED";
	}
	else if (isnan(ENUPosition[0]))
	{
		lethality = "NOT A NUMBER";
	}
	else if (timeOfFlight > MAX_TIME)
	{
		lethality = "MAX TIME EXCEEDED";
	}

}

void logData()
{

	logFile << fixed << setprecision(10) <<
	timeOfFlight << " " <<
	ENUPosition[0] << " " <<
	ENUPosition[1] << " " <<
	ENUPosition[2] << " " <<
	pip[0] << " " <<
	pip[1] << " " <<
	pip[2] << " " <<
	guidanceNormalCommand / grav << " " <<
	FLUAcceleration[2] / grav << " " <<
	bodyRate[1] * radToDeg << " " <<
	ENUEulerDot[1] * radToDeg << " " <<
	pitchFinCommand * radToDeg << " " <<
	alpha * radToDeg << " " <<
	ENUEulerAngles[1] * radToDeg << " " <<
	guidanceSideCommand / grav << " " <<
	FLUAcceleration[1] / grav << " " <<
	bodyRate[2] * radToDeg << " " <<
	ENUEulerDot[2] * radToDeg << " " <<
	yawFinCommand * radToDeg << " " <<
	beta * radToDeg << " " <<
	ENUEulerAngles[2] * radToDeg << " " <<
	ROLL_ANGLE_COMMAND << " " <<
	ENUEulerDot[0] * radToDeg << " " <<
	bodyRate[0] * radToDeg << " " <<
	rollDeflectionDegrees << " " <<
	ENUEulerAngles[0] * radToDeg << " " <<
	seekerPitchError << " " <<
	seekerYawError << " " <<
	staticMargin << " " <<
	machSpeed << "\n";

}

void fly()
{

	atmosphere();
	seeker();
	guidance();
	control();
	actuators();
	aerodynamicAnglesAndConversions();
	tableLookUps();
	accelerationLimit();
	propulsion();
	aerodynamicIntegrationCoefficients();
	aerodynamicFeedbackCoefficients();
	eulerIntegrateStates();
	performanceAndTerminationCheck();
	logData();

}

int main ()
{

	init();
	double lastTime = 0;
	cout << "FLIGHT" << endl;
	while (lethality == "FLYING")
	{
		fly();
		auto print_it = static_cast<int>(round(timeOfFlight * 10000.0)) % 10000;
		if (print_it == 0)
		{
			cout << setprecision(6) << timeOfFlight << " E " << ENUPosition[0] << " N " << ENUPosition[1] << " U " << ENUPosition[2] << " RANGE " << range << " MACH " << machSpeed << endl;
			lastTime = timeOfFlight;
		}
	}
	cout << "\n" << endl;
	cout << "MISSION REPORT" << endl;
	cout << setprecision(6) << "FINAL POSITION AT " << timeOfFlight << " E " << ENUPosition[0] << " N " << ENUPosition[1] << " U " << ENUPosition[2] << " RANGE " << range << " MACH " << machSpeed << endl;
	cout << setprecision(6) << "MISS DISTANCE " << missDistance << " FORWARD, LEFT, UP, MISS DISTANCE " << FLUMissileToPipRelativePosition[0] << " " << FLUMissileToPipRelativePosition[1] << " " << FLUMissileToPipRelativePosition[2] << endl;
	logFile << "\n";
	logFile << setprecision(6) << "MISS DISTANCE " << missDistance << " FORWARD, LEFT, UP, MISS DISTANCE " << FLUMissileToPipRelativePosition[0] << " " << FLUMissileToPipRelativePosition[1] << " " << FLUMissileToPipRelativePosition[2] << endl;
	cout << "SIMULATION RESULT: " << lethality << endl;
	auto wallClockEnd = chrono::high_resolution_clock::now();
	auto simRealRunTime = chrono::duration_cast<chrono::milliseconds>(wallClockEnd - wallClockStart);
	cout << "SIMULATION RUN TIME :" << simRealRunTime.count() << " MILLISECONDS" << endl;
	cout << "\n" << endl;
	return 0;

}