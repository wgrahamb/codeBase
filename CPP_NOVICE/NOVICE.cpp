
// Standard library.
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <math.h>
#include <vector>
#include <map>
#include <algorithm>

// Utility functions.
#include "util.h"

// Namespace.
using namespace std;

// Simulation control.
const double timeStep = 1.0 / 1000.0;

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

/* Missile constants. */
const double REFERENCE_AREA = 0.01824; // M^2.
const double REFERENCE_DIAMETER = 0.1524; // Meters.
const double THRUST_EXIT_AREA = 0.0125; // M^2.
const double ROCKET_BURN_OUT_TIME = 2.421; // Seconds.
const double SEEKER_KF_G = 10.0; // Seeker Kalman filter gain. Per second.
const double SEEKER_KF_ZETA = 0.9; // Seeker Kalman filter damping. Non dimensional.
const double SEEKER_KF_WN = 60.0; // Seeker Kalman filter natural frequency. Radians per second.
const double PROPORTIONAL_GUIDANCE_GAIN = 3.0; // Guidance homing gain.
const double LINE_OF_ATTACK_GUIDANCE_GAIN = 1.5; // Guidance midcourse gain.
const double MAXIMUM_ACCELERATION = 450.0; // Meters per s^2. Roughly 45 Gs.
const double RATE_CONTROL_ZETA = 0.6; // Damping of constant rate control. Non dimensional.
const double ROLL_CONTROL_WN = 20.0; // Natural frequency of roll closed loop complex pole. Radians per second.
const double ROLL_CONTROL_ZETA = 0.9; // Damping of roll closed loop complex pole. Non dimensional.
const double FIN_CONTROL_WN = 100.0; // Natural frequency of roll closed loop complex pole. Radians per second.
const double FIN_CONTROL_ZETA = 0.7; // Damping of roll closed loop complex pole. Non dimensional.
const double FIN_RATE_LIMIT_RADIANS = 10.472; // Radians per second.
const double FIN_CONTROL_MAX_DEFLECTION_DEGREES = 28.0; // Degrees.
const double FIN_CONTROL_MAX_DEFLECTION_RADIANS = 0.4887; // Degrees.
const double ROLL_ANGLE_COMMAND = 0.0; // Degrees or radians.
const double TOTAL_ANGLE_OF_ATTACK_MAX = 40.0; // Degrees.
const double SEA_LEVEL_PRESSURE = 101325; // Pascals.
const double LAUNCH_CENTER_OF_GRAVITY_FROM_NOSE = 1.5357; // Meters.

// Missile structure.
struct MissilePacket
{

	/* Missile variables. */
	// Missile packet.
	double pip[3] = {0.0, 0.0, 0.0}; // Predicted intercept point. Meters.
	double timeOfFlight = 0.0; // Seconds.
	double ENUPosition[3] = {0.0, 0.0, 0.0}; // Meters.
	double range = 0.0; // Meters.
	double ENUVelocity[3] = {0.0, 0.0, 0.0}; // Meters per second.
	double FLUVelocity[3] = {0.0, 0.0, 0.0}; // Meters per second.
	double speed = 0.0; // Meters per second.
	double machSpeed = 0.0; // Non dimensional.
	double ENUAcceleration[3] = {0.0, 0.0, 0.0}; // Meters per s^2.
	double FLUAcceleration[3] = {0.0, 0.0, 0.0}; // Meters per s^2.
	double ENUToFLUMatrix[3][3]; // Non dimensional.
	double alpha = 0.0; // Radians.
	double beta = 0.0; // Radians.
	double ENUEulerAngles[3] = {0.0, 0.0, 0.0}; // Radians.
	double ENUEulerDot[3] = {0.0, 0.0, 0.0}; // Radians per second.
	double bodyRate[3] = {0.0, 0.0, 0.0}; // Radians per second.
	double bodyRateDot[3] = {0.0, 0.0, 0.0}; // Radians per s^2.

	// Environment.
	double gravity = 0.0; // Meters per s^2.
	double FLUGravity[3] = {0.0, 0.0, 0.0}; // Meters per s^2.
	double pressure = 0.0; // Pascals.
	double dynamicPressure = 0.0; // Pascals.

	// Seeker.
	double seekerPitch = 0.0; // Radians.
	double seekerYaw = 0.0; // Radians.
	double seekerENUToFLUMatrix[3][3]; // Non dimensional.
	double seekerPitchError = 0.0; // Radians off target.
	double seekerYawError = 0.0; // Radians off target.
	double seekerWLR = 0.0; // Pointing yaw rate. Radians per second.
	double seekerWLRD = 0.0; // Derivative of pointing yaw rate. Radians per s^2.
	double seekerWLR1 = 0.0; // Yaw sight line spin rate. Radians per second.
	double seekerWLR1D = 0.0; // Derivative of yaw sight line spin rate. Radians per s^2.
	double seekerWLR2 = 0.0; // Second state variable in Kalman filter, yaw. Radians per s^2.
	double seekerWLR2D = 0.0; // Derivative of second state variable in Kalman filter, yaw. Radians per s^3.
	double seekerWLQ = 0.0; // Pointing pitch rate. Radians per second.
	double seekerWLQD = 0.0; // Derivative of pointing pitch rate. Radians per s^2.
	double seekerWLQ1 = 0.0; // pitch sight line spin rate. Radians per second.
	double seekerWLQ1D = 0.0; // Derivative of pitch sight line spin rate. Radians per s^2.
	double seekerWLQ2 = 0.0; // Second state variable in Kalman filter, pitch. Radians per s^2.
	double seekerWLQ2D = 0.0; // Derivative of second state variable in Kalman filter, pitch. Radians per s^3.

	// Guidance.
	double missileToInterceptFLURelativePosition[3] = {0.0, 0.0, 0.0}; // Meters.
	double missileGuidanceNormalCommand = 0.0; // Meters per s^2.
	double missileGuidanceSideCommand = 0.0; // Meters per s^2.
	double accelerationLimit = 0.0; // Meters per s^2.

	// Control.
	double controlYY = 0.0; // Yaw feed forward integration. Meters per second.
	double controlYYD = 0.0; // Yaw feed forward derivative. Meters per second.
	double missileControlZZ = 0.0; // Pitch feed forward integration. Meters per second.
	double missileControlZZD = 0.0; // Pitch feed forward derivative. Meters per second.
	double missileRollFinCommand = 0.0; // Radians.
	double missilePitchFinCommand = 0.0; // Radians.
	double yawFinCommand = 0.0; // Radians.

	// Actuators.
	double rollFinDeflectionRadians = 0.0; // Radians.
	double pitchFinDeflection = 0.0; // Radians.
	double yawFinDeflection = 0.0; // Radians.
	double DEL1 = 0.0; // Radians.
	double DEL1D = 0.0; // Fin one position derived. Radians.
	double DEL1DOT = 0.0; // Fin one rate. Radians per second.
	double DEL1DOTDOT = 0.0; // Fin one rate derived. Radians per s^2.
	double DEL2 = 0.0; // Radians.
	double DEL2D = 0.0; // Fin two position derived. Radians.
	double DEL2DOT = 0.0; // Fin two rate. Radians per second.
	double DEL2DOTDOT = 0.0; // Fin two rate derived. Radians per s^2.
	double DEL3 = 0.0; // Radians.
	double DEL3D = 0.0; // Fin three position derived. Radians.
	double DEL3DOT = 0.0; // Fin three rate. Radians per second.
	double DEL3DOTDOT = 0.0; // Fin three rate derived. Radians per s^2.
	double DEL4 = 0.0; // Radians.
	double DEL4D = 0.0; // Fin four position derived. Radians.
	double DEL4DOT = 0.0; // Fin four rate. Radians per second.
	double DEL4DOTDOT = 0.0; // Fin four rate derived. Radians per s^2.

	// Aero ballistic angles.
	double alphaPrimeDegrees = 0.0; // Total angle of attack. Degrees.
	double sinPhiPrime = 0.0; // Non dimensional.
	double cosPhiPrime = 0.0; // Non dimensional.
	double rollFinDeflectionDegrees = 0.0; // Degrees.
	double pitchDeflectionAeroBallisticFrameDegrees = 0.0; // Degrees.
	double yawDeflectionAeroBallisticFrameDegrees = 0.0; // Degrees.
	double totalFinDeflectionDegrees = 0.0; // Degrees.
	double pitchRateAeroBallisticFrameDegrees = 0.0; // Degrees per second.
	double yawRateAeroBallisticFrameDegrees = 0.0; // Degrees per second.
	double rollRateDegrees = 0.0; // Degrees per second.
	double sinOfFourTimesPhiPrime = 0.0; // Non dimensional.
	double squaredSinOfTwoTimesPhiPrime = 0.0; // Non dimensional.

	// Data look up.
	map<string, int> tableNameIndexPairs;
	vector<vector<vector<double>>> tables;
	double CA0 = 0.0; // Non dimensional.
	double CAA = 0.0; // Per degree.
	double CAD = 0.0; // Per degree^2.
	double CAOFF = 0.0; // Non dimensional.
	double CYP = 0.0; // Non dimensional.
	double CYDR = 0.0; // Per degree.
	double CN0 = 0.0; // Non dimensional.
	double CNP = 0.0; // Non dimensional.
	double CNDQ = 0.0; // Per degree.
	double CLLAP = 0.0; // Per degree^2.
	double CLLP = 0.0; // Per degree.
	double CLLDP = 0.0; // Per degree.
	double CLM0 = 0.0; // Non dimensional.
	double CLMP = 0.0; // Non dimensional.
	double CLMQ = 0.0; // Per degree.
	double CLMDQ = 0.0; // Per degree.
	double CLNP = 0.0; // Non dimensional
	double mass = 0.0; // Kilograms.
	double unadjustedThrust = 0.0; // Newtons.
	double transverseMomentOfInertia = 0.0; // Kilograms * M^2.
	double axialMomentOfInertia = 0.0; // Kilograms * M^2.
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

	// Aerodynamic control feedback values.
	double CNA = 0.0; // Per degree.
	double CMA = 0.0; // Per degree.
	double CND = 0.0; // Per degree.
	double CMD = 0.0; // Per degree.
	double CMQ = 0.0; // Per degree.
	double CLP = 0.0; // Per degree.
	double CLD = 0.0; // Per degree.
	double staticMargin = 0.0; // Non dimensional.

	// Integration.
	// ENUPosition, ENUVelocity, ENUAcceleration, ENUEuler, ENUEulerDot, BodyRate, BodyRateDot.
	int MISSILE_INTEGRATION_METHOD = 0;
	int MISSILE_INTEGRATION_PASS = 0;
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

	// Performance and end check.
	double missDistance = 0.0; // Meters.
	string lethality; // For termination check.

};

void lookUpTablesFormat (MissilePacket &Missile, string dataFile)
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
		if (line.substr(0, 4) == "NAME"){
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
		else if (line.substr(0, 2) == "NX"){
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
				if (line.substr(i, 2) == "NX") {
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
			if (D2 == 0) {
				// "COLUMNS" DIMENSION BECOMES TWO
				D2 = 2;
				// STORE "COLUMNS" DIMENSION IN VECTOR
				dimension.push_back(D2);
			}
		}
		// NOTHING FLAGGED, NEXT ITERATION
		if (flag == 0){
			// ONLY CHECK IF A TABLE HAS BEEN INITIALIZED
			if (dimensions.size() > 0){
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
					if (dataPoint.find_first_not_of(' ') != std::string::npos){
						// CONVERT STRING TO DOUBLE
						double dataPointDouble = stod(dataPoint);
						/////////// FOR THIS SPECIFIC SET OF DATA, CHECK FOR 90. THERE ARE 14 ROWS AND 15 COLUMNS FOR THE TWO DIMENSIONAL TABLES WHICH MEANS THIS IS A SPECIFIC PIECE OF CODE. WOULD HAVE TO BE ALTERED FOR DIFFERING DATA SETS.
						if (dataPointDouble == 90) {
							// PLACE IT AT THE FAR RIGHT CORNER
							Missile.tables[tableNoTrack - 1][0].back() = dataPointDouble;
						}
						// IF THIS THE FIRST LOOP, THIS IS THE COLUMN IN THE DATA SET THAT DISPLAYS THE "ROWS" VALUES
						else if (columnCount == 1) {
							// FOR TWO DIMENSIONAL TABLE
							if (dimensions[tableNoTrack -1][1] != 2){
								// PLACE DATA POINT IN ITS PLACE
								Missile.tables[tableNoTrack - 1][rowNoTrack][0] = dataPointDouble;
							}
							// FOR ONE DIMENSIONAL TABLE
							else {
								// PLACE DATA POINT IN ITS PLACE
								Missile.tables[tableNoTrack - 1][rowNoTrack - 1][0] = dataPointDouble;
							}
						}
						// IF THIS THE SECOND LOOP, THIS IS THE COLUMN IN THE DATA SET THAT DISPLAYS THE "COLUMNS" VALUES, ONLY FOR TWO DIMENSIONAL TABLES
						else if (columnCount == 2 and dimensions[tableNoTrack -1][1] != 2) {
							// PLACE DATA POINT IN ITS PLACE
							Missile.tables[tableNoTrack - 1][0][rowNoTrack] = dataPointDouble;
						}
						// ELSE FOR ACTUAL DATA POINTS
						else {
							// FOR TWO DIMENSIONAL TABLES
							if (dimensions[tableNoTrack -1][1] != 2) {
								// PLACE DATA POINT IN ITS PLACE
								Missile.tables[tableNoTrack - 1][rowNoTrack][columnCount - 2] = dataPointDouble;
							}
							// FOR ONE DIMENSIONAL TABLES
							else {
								// PLACE DATA POINT IN ITS PLACE
								Missile.tables[tableNoTrack - 1][rowNoTrack - 1][columnCount - 1] = dataPointDouble;
							}
						}
					}
				} while (parseLine);
			}
		}
		// CREATE A TABLE OF CORRECT SIZE AND STORE IT
		else if (flag == 1 or flag == 2){
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
			Missile.tables.push_back(newTable);
		}
		// STORE NAME OF TABLE
		else if (flag == 3){
			// MAP TABLE NAME INDEX PAIR
			Missile.tableNameIndexPairs.emplace(name, tableNoTrack - 1);
		}
	}
}

// Works for now.
// Needs to be refactored for fire control.
// This will be done by the launcher, when the scenario is created.
// When a missile packet is copied, this does not have to be done.
void initializeMissile(MissilePacket &Missile, string inputFile)
{

	// Set time of flight.
	Missile.timeOfFlight = 0.0;

	// Open input file.
	std::ifstream inPut;
	inPut.open("input.txt");

	// Declare inputs.
	double phi, theta, psi, tgtE, tgtN, tgtU;

	// Populate inputs.
	inPut >> phi >> theta >> psi >> tgtE >> tgtN >> tgtU;

	// Populate the predicted intercept point.
	// Needs to be an input for flyouts.
	Missile.pip[0] = tgtE;
	Missile.pip[1] = tgtN;
	Missile.pip[2] = tgtU;

	// Convert the input euler angles to radians.
	phi *= degToRad;
	theta *= degToRad;
	psi *= degToRad;

	// Set missile ENU Euler angles.
	// Will be determined by the launcher.
	Missile.ENUEulerAngles[0] = phi;
	Missile.ENUEulerAngles[1] = theta;
	Missile.ENUEulerAngles[2] = psi;

	// Initialize navigation direction cosine matrix.
	// Will be determined by the launcher.
	eulerAnglesToLocalOrientation(phi, -theta, psi, Missile.ENUToFLUMatrix);

	// Set missile ENU position.
	// Will be determined by launcher.
	Missile.ENUPosition[0] = 0.0;
	Missile.ENUPosition[1] = 0.0;
	Missile.ENUPosition[2] = 0.0;

	// Set missile ENU velocity.
	// Determined by launcher orientation.
	Missile.ENUVelocity[0] = Missile.ENUToFLUMatrix[0][0];
	Missile.ENUVelocity[1] = Missile.ENUToFLUMatrix[0][1];
	Missile.ENUVelocity[2] = Missile.ENUToFLUMatrix[0][2];

	// Initialize missile FLU velocity.
	threeByThreeTimesThreeByOne(
		Missile.ENUToFLUMatrix,
		Missile.ENUVelocity,
		Missile.FLUVelocity
	);

	// Initialize missile ENU and FLU acceleration.
	Missile.ENUAcceleration[0] = 0.0;
	Missile.ENUAcceleration[1] = 0.0;
	Missile.ENUAcceleration[2] = 0.0;
	Missile.FLUAcceleration[0] = 0.0;
	Missile.FLUAcceleration[1] = 0.0;
	Missile.FLUAcceleration[2] = 0.0;

	// Initialize the seeker by pointing it directly at the pip.
	// Fine for now.
	// In final simulation, set so that seeker turns on at start of homing.
	// The missile packet will need a waypoint and target states.
	double missileToInterceptENURelativePosition[3];
	double missileToInterceptENURelativePositionUnit[3];
	double missileToInterceptFLURelativePositionUnit[3];
	double missileToInterceptFLURelativePositionAzimuth;
	double missileToInterceptFLURelativePositionElevation;
	double seekerAttitudeToENUMatrix[3][3];
	subtractTwoVectors(Missile.ENUPosition, Missile.pip, missileToInterceptENURelativePosition);
	unitVec(missileToInterceptENURelativePosition, missileToInterceptENURelativePositionUnit);
	threeByThreeTimesThreeByOne(
		Missile.ENUToFLUMatrix,
		missileToInterceptENURelativePositionUnit,
		missileToInterceptFLURelativePositionUnit
	);
	azAndElFromVector(
		missileToInterceptFLURelativePositionAzimuth,
		missileToInterceptFLURelativePositionElevation,
		missileToInterceptFLURelativePositionUnit
	);
	Missile.seekerPitch = missileToInterceptFLURelativePositionAzimuth;
	Missile.seekerYaw = missileToInterceptFLURelativePositionElevation;
	flightPathAnglesToLocalOrientation(
		missileToInterceptFLURelativePositionAzimuth,
		-missileToInterceptFLURelativePositionElevation,
		seekerAttitudeToENUMatrix
	);
	threeByThreeTimesThreeByThree(
		seekerAttitudeToENUMatrix,
		Missile.ENUToFLUMatrix,
		Missile.seekerENUToFLUMatrix
	);
	Missile.seekerPitchError = 0.0; // Since now pointed directly at target.
	Missile.seekerYawError = 0.0; // Since now pointed directly at target.
	Missile.seekerWLQ = Missile.seekerPitch;
	Missile.seekerWLR = Missile.seekerYaw;

	// Set lethality.
	Missile.lethality = "FLYING";

	// Console report.
	cout << "\n";
	cout << "MODEL INITIATED" << endl;
	cout << "\n";

}

void timeOfFlight(MissilePacket &Missile)
{

	Missile.timeOfFlight += timeStep;

}

void atmosphere(MissilePacket &Missile)
{

	int tableIndex;
	double altitude;
	double rho;
	double missileENUGravity[3];
	double missileSpeedOfSound;

	altitude = Missile.ENUPosition[2];
	tableIndex = Missile.tableNameIndexPairs["RHO"];
	rho = linearInterpolationWithBoundedEnds(Missile.tables[tableIndex], altitude);
	tableIndex = Missile.tableNameIndexPairs["GRAVITY"];
	Missile.gravity = linearInterpolationWithBoundedEnds(Missile.tables[tableIndex], altitude);
	tableIndex = Missile.tableNameIndexPairs["PRESSURE"];
	Missile.pressure = linearInterpolationWithBoundedEnds(Missile.tables[tableIndex], altitude);
	tableIndex = Missile.tableNameIndexPairs["SPEED_OF_SOUND"];
	missileSpeedOfSound = linearInterpolationWithBoundedEnds(Missile.tables[tableIndex], altitude);
	missileENUGravity[0] = 0.0;
	missileENUGravity[1] = 0.0;
	missileENUGravity[2] = -1.0 * Missile.gravity;
	threeByThreeTimesThreeByOne(
		Missile.ENUToFLUMatrix,
		missileENUGravity,
		Missile.FLUGravity
	);
	magnitude(Missile.ENUVelocity, Missile.speed); // METERS PER SECOND
	Missile.machSpeed = Missile.speed / missileSpeedOfSound; // ND
	Missile.dynamicPressure = 0.5 * rho * Missile.speed * Missile.speed; // PASCALS

}

void seeker(MissilePacket &Missile)
{

	double wsq;
	double gg;
	double wlr1d_new;
	double wlr1_new;
	double wlr2d_new;
	double wlr2_new;
	double wlq1d_new;
	double wlq1_new;
	double wlq2d_new;
	double wlq2_new;
	double wlrd_new;
	double wlr_new;
	double wlqd_new;
	double wlq_new;
	double missileToInterceptENURelativePosition[3];
	double seekerAttitudeToENUMatrix[3][3];
	double seekerToInterceptENURelativePosition[3];
	double error[3];
	double seekerToInterceptENURelativePositionWithError[3];

	wsq = SEEKER_KF_WN * SEEKER_KF_WN;
	gg = SEEKER_KF_G * wsq;

	// Seeker yaw.
	wlr1d_new = Missile.seekerWLR2;
	wlr1_new = trapezoidIntegrate(wlr1d_new, Missile.seekerWLR1D, Missile.seekerWLR1, timeStep);
	Missile.seekerWLR1 = wlr1_new;
	Missile.seekerWLR1D = wlr1d_new;
	wlr2d_new = gg * Missile.seekerYawError - 2 * SEEKER_KF_ZETA * SEEKER_KF_WN * Missile.seekerWLR1D - wsq * Missile.seekerWLR1;
	wlr2_new = trapezoidIntegrate(wlr2d_new, Missile.seekerWLR2D, Missile.seekerWLR2, timeStep);
	Missile.seekerWLR2 = wlr2_new;
	Missile.seekerWLR2D = wlr2d_new;

	// Yaw control.
	wlrd_new = Missile.seekerWLR1 - Missile.bodyRate[2];
	wlr_new = trapezoidIntegrate(wlrd_new, Missile.seekerWLRD, Missile.seekerWLR, timeStep);
	Missile.seekerWLR = wlr_new;
	Missile.seekerWLRD = wlrd_new;
	Missile.seekerYaw = Missile.seekerWLR;

	// Seeker pitch.
	wlq1d_new = Missile.seekerWLQ2;
	wlq1_new = trapezoidIntegrate(wlq1d_new, Missile.seekerWLQ1D, Missile.seekerWLQ1, timeStep);
	Missile.seekerWLQ1 = wlq1_new;
	Missile.seekerWLQ1D = wlq1d_new;
	wlq2d_new = gg * Missile.seekerPitchError - 2 * SEEKER_KF_ZETA * SEEKER_KF_WN * Missile.seekerWLQ1D - wsq * Missile.seekerWLQ1;
	wlq2_new = trapezoidIntegrate(wlq2d_new, Missile.seekerWLQ2D, Missile.seekerWLQ2, timeStep);
	Missile.seekerWLQ2 = wlq2_new;
	Missile.seekerWLQ2D = wlq2d_new;

	// Pitch control.
	wlrd_new = Missile.seekerWLQ1 - Missile.bodyRate[1];
	wlq_new = trapezoidIntegrate(wlqd_new, Missile.seekerWLQD, Missile.seekerWLQ, timeStep);
	Missile.seekerWLQ = wlq_new;
	Missile.seekerWLQD = wlqd_new;
	Missile.seekerPitch = Missile.seekerWLQ;

	// Calculate FLU relative position for guidance.
	subtractTwoVectors(Missile.ENUPosition, Missile.pip, missileToInterceptENURelativePosition);
	eulerAnglesToLocalOrientation(
		0.0,
		-Missile.seekerPitch,
		Missile.seekerYaw,
		seekerAttitudeToENUMatrix
	);
	threeByThreeTimesThreeByThree(
		seekerAttitudeToENUMatrix,
		Missile.ENUToFLUMatrix,
		Missile.seekerENUToFLUMatrix
	);
	threeByThreeTimesThreeByOne(
		Missile.seekerENUToFLUMatrix,
		missileToInterceptENURelativePosition,
		seekerToInterceptENURelativePosition
	);
	error[0] = 1.0;
	error[1] = 0.5;
	error[2] = 0.2;
	multiplyTwoVectors(
		seekerToInterceptENURelativePosition,
		error,
		seekerToInterceptENURelativePositionWithError
	);
	azAndElFromVector(
		Missile.seekerYawError,
		Missile.seekerPitchError,
		seekerToInterceptENURelativePositionWithError
	);
	oneByThreeTimesThreeByThree(
		seekerToInterceptENURelativePositionWithError,
		seekerAttitudeToENUMatrix,
		Missile.missileToInterceptFLURelativePosition
	);


}

void guidance(MissilePacket &Missile)
{

	double forwardLeftUpMissileToInterceptPositionUnitVector[3];
	unitVec(Missile.missileToInterceptFLURelativePosition, forwardLeftUpMissileToInterceptPositionUnitVector);
	double forwardLeftUpMissileToInterceptLineOfSightVel[3];
	vectorProjection(forwardLeftUpMissileToInterceptPositionUnitVector, Missile.FLUVelocity, forwardLeftUpMissileToInterceptLineOfSightVel);
	double timeToGo, forwardLeftUpMissileToInterceptPositionMagnitude, forwardLeftUpMissileToInterceptLineOfSightVelMagnitude;
	magnitude(Missile.missileToInterceptFLURelativePosition, forwardLeftUpMissileToInterceptPositionMagnitude);
	magnitude(forwardLeftUpMissileToInterceptLineOfSightVel, forwardLeftUpMissileToInterceptLineOfSightVelMagnitude);
	timeToGo = forwardLeftUpMissileToInterceptPositionMagnitude / forwardLeftUpMissileToInterceptLineOfSightVelMagnitude;
	if (timeToGo < 5)
	{
		double closingVelocity[3];
		multiplyVectorTimesScalar(-1.0, Missile.FLUVelocity, closingVelocity);
		double closingSpeed;
		magnitude(closingVelocity, closingSpeed);
		double TEMP1[3], TEMP2;
		crossProductTwoVectors(Missile.missileToInterceptFLURelativePosition, closingVelocity, TEMP1);
		dotProductTwoVectors(Missile.missileToInterceptFLURelativePosition, Missile.missileToInterceptFLURelativePosition, TEMP2);
		double lineOfSightRate[3];
		divideVectorByScalar(TEMP2, TEMP1, lineOfSightRate);
		double TEMP3, TEMP4[3];
		double proportionalGuidanceGain = 3.0;
		TEMP3 = -1 * PROPORTIONAL_GUIDANCE_GAIN * closingSpeed;
		multiplyVectorTimesScalar(TEMP3, forwardLeftUpMissileToInterceptPositionUnitVector, TEMP4);
		double COMMAND[3];
		crossProductTwoVectors(TEMP4, lineOfSightRate, COMMAND);
		Missile.missileGuidanceNormalCommand = COMMAND[2];
		Missile.missileGuidanceSideCommand = COMMAND[1];
	}
	else
	{
		double lineOfAttack[3];
		lineOfAttack[0] = forwardLeftUpMissileToInterceptPositionUnitVector[0];
		lineOfAttack[1] = forwardLeftUpMissileToInterceptPositionUnitVector[1];
		lineOfAttack[2] = -0.5;
		double forwardLeftUpMissileToInterceptLineOfAttackVel[3];
		vectorProjection(lineOfAttack, Missile.FLUVelocity, forwardLeftUpMissileToInterceptLineOfAttackVel);
		double G = 1 - exp(-0.001 * forwardLeftUpMissileToInterceptPositionMagnitude);
		Missile.missileGuidanceNormalCommand = LINE_OF_ATTACK_GUIDANCE_GAIN * (forwardLeftUpMissileToInterceptLineOfSightVel[2] + G * forwardLeftUpMissileToInterceptLineOfAttackVel[2]);
		Missile.missileGuidanceSideCommand = LINE_OF_ATTACK_GUIDANCE_GAIN * (forwardLeftUpMissileToInterceptLineOfSightVel[1] + G * forwardLeftUpMissileToInterceptLineOfAttackVel[1]);
	}

}

void control(MissilePacket &Missile)
{

	if (Missile.machSpeed > 0.6) // Maneuvering.
	{

		double DNA = Missile.CNA * (Missile.dynamicPressure * REFERENCE_AREA / Missile.mass); // METERS PER SECOND^2
		double DMA = Missile.CMA * (Missile.dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / Missile.transverseMomentOfInertia); // PER SECOND^2
		double DMD = Missile.CMD * (Missile.dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / Missile.transverseMomentOfInertia); // PER SECOND^2
		double DMQ = Missile.CMQ * (REFERENCE_DIAMETER / (2 * Missile.speed)) * (Missile.dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / Missile.transverseMomentOfInertia); // PER SECOND
		double DLP = Missile.CLP * (REFERENCE_DIAMETER / (2 * Missile.speed)) * (Missile.dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / Missile.axialMomentOfInertia); // PER SECOND
		double DLD = Missile.CLD * (Missile.dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / Missile.axialMomentOfInertia); // PER SECOND^2

		double WACL = 0.013 * sqrt(Missile.dynamicPressure) + 7.1;
		double ZACL = 0.000559 * sqrt(Missile.dynamicPressure) + 0.232;
		double PACL = 14;

		// FEEDBACK GAINS
		double GAINFB3 = WACL * WACL * PACL / (DNA * DMD);
		double GAINFB2 = (2 * ZACL * WACL + PACL + DMQ - DNA / Missile.speed) / DMD;
		double GAINFB1 = (
			WACL * WACL + 2 * ZACL * WACL * PACL + DMA + DMQ * DNA / Missile.speed - GAINFB2 * DMD * DNA / Missile.speed
		) / (DNA * DMD);

		// ROLL
		double GKP = (2 * ROLL_CONTROL_WN * ROLL_CONTROL_ZETA + DLP) / DLD;
		double GKPHI = ROLL_CONTROL_WN * ROLL_CONTROL_WN / DLD;
		double EPHI = GKPHI * (ROLL_ANGLE_COMMAND - Missile.ENUEulerAngles[0]);
		Missile.missileRollFinCommand = EPHI - GKP * Missile.bodyRate[0];

		// PITCH
		double zzdNew = Missile.missileGuidanceNormalCommand - Missile.FLUAcceleration[2];
		double zzNew = trapezoidIntegrate(zzdNew, Missile.missileControlZZD, Missile.missileControlZZ, timeStep);
		Missile.missileControlZZ = zzNew;
		Missile.missileControlZZD = zzdNew;
		double deflPitch = -1 * GAINFB1 * Missile.FLUAcceleration[2] - GAINFB2 * Missile.bodyRate[1] + GAINFB3 * Missile.missileControlZZ;
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
		Missile.missilePitchFinCommand = deflPitch * degToRad;

		// YAW
		double yydNew = Missile.FLUAcceleration[1] - Missile.missileGuidanceSideCommand;
		double yyNew = trapezoidIntegrate(yydNew, Missile.controlYYD, Missile.controlYY, timeStep);
		Missile.controlYY = yyNew;
		Missile.controlYYD = yydNew;
		double deflYaw = GAINFB1 * Missile.FLUAcceleration[1] - GAINFB2 * Missile.bodyRate[2] + GAINFB3 * Missile.controlYY;
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
		Missile.yawFinCommand = deflYaw * degToRad;

	}
	else if (Missile.machSpeed > 0.1) // Rate lock.
	{

		double DNA = Missile.CNA * (Missile.dynamicPressure * REFERENCE_AREA / Missile.mass); // METERS PER SECOND^2
		double DND = Missile.CND * (Missile.dynamicPressure * REFERENCE_AREA / Missile.mass); // METERS PER SECOND^2
		double DMA = Missile.CMA * (Missile.dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / Missile.transverseMomentOfInertia); // PER SECOND^2
		double DMD = Missile.CMD * (Missile.dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / Missile.transverseMomentOfInertia); // PER SECOND^2
		double DMQ = Missile.CMQ * (REFERENCE_DIAMETER / (2 * Missile.speed)) * (Missile.dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / Missile.transverseMomentOfInertia); // PER SECOND
		double DLP = Missile.CLP * (REFERENCE_DIAMETER / (2 * Missile.speed)) * (Missile.dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / Missile.axialMomentOfInertia); // PER SECOND
		double DLD = Missile.CLD * (Missile.dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER / Missile.axialMomentOfInertia); // PER SECOND^2

		// ROLL
		double GKP = (2 * ROLL_CONTROL_WN * ROLL_CONTROL_ZETA + DLP) / DLD;
		double GKPHI = ROLL_CONTROL_WN * ROLL_CONTROL_WN / DLD;
		double EPHI = GKPHI * (ROLL_ANGLE_COMMAND - Missile.ENUEulerAngles[0]);
		Missile.missileRollFinCommand = EPHI - GKP * Missile.bodyRate[0];

		// RATE CONTROL
		double ZRATE = DNA / Missile.speed - DMA * DND / (Missile.speed * DMD); // ND
		double AA = DNA / Missile.speed - DMQ; // ND
		double BB = -1 * DMA - DMQ * DNA / Missile.speed; // ND
		double TEMP1 = AA - 2 * RATE_CONTROL_ZETA * RATE_CONTROL_ZETA * ZRATE; // ND
		double TEMP2 = AA * AA - 4 * RATE_CONTROL_ZETA * RATE_CONTROL_ZETA * BB; // ND
		double RADIX = TEMP1 * TEMP1 - TEMP2; // ND
		double GRATE = (-1 * TEMP1 + sqrt(RADIX)) / (-1 * DMD); // ND

		// PITCH
		Missile.missilePitchFinCommand = GRATE * Missile.bodyRate[1]; // RADIANS

		// YAW
		Missile.yawFinCommand = GRATE * Missile.bodyRate[2]; // RADIANS

	}
	else // Free flight.
	{
		Missile.missileRollFinCommand = 0.0;
		Missile.missilePitchFinCommand = 0.0;
		Missile.yawFinCommand = 0.0;
	}

}

void actuators(MissilePacket &Missile)
{

	double DEL1C = -Missile.missileRollFinCommand + Missile.missilePitchFinCommand - Missile.yawFinCommand;
	double DEL2C = -Missile.missileRollFinCommand + Missile.missilePitchFinCommand + Missile.yawFinCommand;
	double DEL3C = Missile.missileRollFinCommand + Missile.missilePitchFinCommand - Missile.yawFinCommand;
	double DEL4C = Missile.missileRollFinCommand + Missile.missilePitchFinCommand + Missile.yawFinCommand;
	
	int flag;

	// FIN ONE
	flag = 0;
	if (abs(Missile.DEL1) > FIN_CONTROL_MAX_DEFLECTION_RADIANS)
	{
		if (Missile.DEL1 < 0)
		{
			Missile.DEL1 = -1 * FIN_CONTROL_MAX_DEFLECTION_RADIANS;
		}
		else if (Missile.DEL1 > 0)
		{
			Missile.DEL1 = FIN_CONTROL_MAX_DEFLECTION_RADIANS;
		}
		if ((Missile.DEL1 * Missile.DEL1DOT) > 0)
		{
			Missile.DEL1DOT = 0;
		}
	}
	if (abs(Missile.DEL1DOT) > FIN_RATE_LIMIT_RADIANS )
	{
		flag = 1;
		if (Missile.DEL1DOT < 0)
		{
			Missile.DEL1DOT = -1 * FIN_RATE_LIMIT_RADIANS ;
		}
		else if (Missile.DEL1DOT > 0)
		{
			Missile.DEL1DOT = FIN_RATE_LIMIT_RADIANS ;
		}
	}
	double DEL1D_NEW = Missile.DEL1DOT;
	double DEL1_NEW = trapezoidIntegrate(DEL1D_NEW, Missile.DEL1D, Missile.DEL1, timeStep);
	Missile.DEL1 = DEL1_NEW;
	Missile.DEL1D = DEL1D_NEW;
	double EDX1 = DEL1C - Missile.DEL1;
	double DEL1DOTDOT_NEW = FIN_CONTROL_WN * FIN_CONTROL_WN * EDX1 - 2 * FIN_CONTROL_ZETA * FIN_CONTROL_WN * Missile.DEL1D;
	double DEL1DOT_NEW = trapezoidIntegrate(DEL1DOTDOT_NEW, Missile.DEL1DOTDOT, Missile.DEL1DOT, timeStep);
	Missile.DEL1DOT = DEL1DOT_NEW;
	Missile.DEL1DOTDOT = DEL1DOTDOT_NEW;
	if (flag == 1 and (Missile.DEL1DOT * Missile.DEL1DOTDOT) > 0)
	{
		Missile.DEL1DOTDOT = 0.0;
	}

	// FIN TWO
	flag = 0;
	if (abs(Missile.DEL2) > FIN_CONTROL_MAX_DEFLECTION_RADIANS)
	{
		if (Missile.DEL2 < 0)
		{
			Missile.DEL2 = -1 * FIN_CONTROL_MAX_DEFLECTION_RADIANS;
		}
		else if (Missile.DEL2 > 0)
		{
			Missile.DEL2 = FIN_CONTROL_MAX_DEFLECTION_RADIANS;
		}
		if ((Missile.DEL2 * Missile.DEL2DOT) > 0)
		{
			Missile.DEL2DOT = 0;
		}
	}
	if (abs(Missile.DEL2DOT) > FIN_RATE_LIMIT_RADIANS )
	{
		flag = 1;
		if (Missile.DEL2DOT < 0)
		{
			Missile.DEL2DOT = -1 * FIN_RATE_LIMIT_RADIANS ;
		}
		else if (Missile.DEL2DOT > 0)
		{
			Missile.DEL2DOT = FIN_RATE_LIMIT_RADIANS ;
		}
	}
	double DEL2D_NEW = Missile.DEL2DOT;
	double DEL2_NEW = trapezoidIntegrate(DEL2D_NEW, Missile.DEL2D, Missile.DEL2, timeStep);
	Missile.DEL2 = DEL2_NEW;
	Missile.DEL2D = DEL2D_NEW;
	double EDX2 = DEL2C - Missile.DEL2;
	double DEL2DOTDOT_NEW = FIN_CONTROL_WN * FIN_CONTROL_WN * EDX2 - 2 * FIN_CONTROL_ZETA * FIN_CONTROL_WN * Missile.DEL2D;
	double DEL2DOT_NEW = trapezoidIntegrate(DEL2DOTDOT_NEW, Missile.DEL2DOTDOT, Missile.DEL2DOT, timeStep);
	Missile.DEL2DOT = DEL2DOT_NEW;
	Missile.DEL2DOTDOT = DEL2DOTDOT_NEW;
	if (flag == 1 and (Missile.DEL2DOT * Missile.DEL2DOTDOT) > 0)
	{
		Missile.DEL2DOTDOT = 0.0;
	}

	// FIN THREE
	flag = 0;
	if (abs(Missile.DEL3) > FIN_CONTROL_MAX_DEFLECTION_RADIANS)
	{
		if (Missile.DEL3 < 0)
		{
			Missile.DEL3 = -1 * FIN_CONTROL_MAX_DEFLECTION_RADIANS;
		}
		else if (Missile.DEL3 > 0)
		{
			Missile.DEL3 = FIN_CONTROL_MAX_DEFLECTION_RADIANS;
		}
		if ((Missile.DEL3 * Missile.DEL3DOT) > 0)
		{
			Missile.DEL3DOT = 0;
		}
	}
	if (abs(Missile.DEL3DOT) > FIN_RATE_LIMIT_RADIANS )
	{
		flag = 1;
		if (Missile.DEL3DOT < 0)
		{
			Missile.DEL3DOT = -1 * FIN_RATE_LIMIT_RADIANS ;
		}
		else if (Missile.DEL3DOT > 0)
		{
			Missile.DEL3DOT = FIN_RATE_LIMIT_RADIANS ;
		}
	}
	double DEL3D_NEW = Missile.DEL3DOT;
	double DEL3_NEW = trapezoidIntegrate(DEL3D_NEW, Missile.DEL3D, Missile.DEL3, timeStep);
	Missile.DEL3 = DEL3_NEW;
	Missile.DEL3D = DEL3D_NEW;
	double EDX3 = DEL3C - Missile.DEL3;
	double DEL3DOTDOT_NEW = FIN_CONTROL_WN * FIN_CONTROL_WN * EDX3 - 2 * FIN_CONTROL_ZETA * FIN_CONTROL_WN * Missile.DEL3D;
	double DEL3DOT_NEW = trapezoidIntegrate(DEL3DOTDOT_NEW, Missile.DEL3DOTDOT, Missile.DEL3DOT, timeStep);
	Missile.DEL3DOT = DEL3DOT_NEW;
	Missile.DEL3DOTDOT = DEL3DOTDOT_NEW;
	if (flag == 1 and (Missile.DEL3DOT * Missile.DEL3DOTDOT) > 0)
	{
		Missile.DEL3DOTDOT = 0.0;
	}

	// FIN FOUR
	flag = 0;
	if (abs(Missile.DEL4) > FIN_CONTROL_MAX_DEFLECTION_RADIANS)
	{
		if (Missile.DEL4 < 0)
		{
			Missile.DEL4 = -1 * FIN_CONTROL_MAX_DEFLECTION_RADIANS;
		}
		else if (Missile.DEL4 > 0)
		{
			Missile.DEL4 = FIN_CONTROL_MAX_DEFLECTION_RADIANS;
		}
		if ((Missile.DEL4 * Missile.DEL4DOT) > 0)
		{
			Missile.DEL4DOT = 0;
		}
	}
	if (abs(Missile.DEL4DOT) > FIN_RATE_LIMIT_RADIANS)
	{
		flag = 1;
		if (Missile.DEL4DOT < 0)
		{
			Missile.DEL4DOT = -1 * FIN_RATE_LIMIT_RADIANS ;
		}
		else if (Missile.DEL4DOT > 0)
		{
			Missile.DEL4DOT = FIN_RATE_LIMIT_RADIANS ;
		}
	}
	double DEL4D_NEW = Missile.DEL4DOT;
	double DEL4_NEW = trapezoidIntegrate(DEL4D_NEW, Missile.DEL4D, Missile.DEL4, timeStep);
	Missile.DEL4 = DEL4_NEW;
	Missile.DEL4D = DEL4D_NEW;
	double EDX4 = DEL4C - Missile.DEL4;
	double DEL4DOTDOT_NEW = FIN_CONTROL_WN * FIN_CONTROL_WN * EDX4 - 2 * FIN_CONTROL_ZETA * FIN_CONTROL_WN * Missile.DEL4D;
	double DEL4DOT_NEW = trapezoidIntegrate(DEL4DOTDOT_NEW, Missile.DEL4DOTDOT, Missile.DEL4DOT, timeStep);
	Missile.DEL4DOT = DEL4DOT_NEW;
	Missile.DEL4DOTDOT = DEL4DOTDOT_NEW;
	if (flag == 1 and (Missile.DEL4DOT * Missile.DEL4DOTDOT) > 0)
	{
		Missile.DEL4DOTDOT = 0.0;
	}

	Missile.rollFinDeflectionRadians = (-Missile.DEL1 - Missile.DEL2 + Missile.DEL3 + Missile.DEL4) / 4;
	Missile.pitchFinDeflection = (Missile.DEL1 + Missile.DEL2 + Missile.DEL3 + Missile.DEL4) / 4;
	Missile.yawFinDeflection = (-Missile.DEL1 + Missile.DEL2 - Missile.DEL3 + Missile.DEL4) / 4;

}

void aeroBallisticAnglesAndConversions(MissilePacket &Missile)
{

	double alphaPrime = acos(cos(Missile.alpha) * cos(Missile.beta));
	Missile.alphaPrimeDegrees = radToDeg * alphaPrime;
	double phiPrime = atan2(tan(Missile.beta), sin(Missile.alpha));
	Missile.sinPhiPrime = sin(phiPrime);
	Missile.cosPhiPrime = cos(phiPrime);
	double pitchDeflAeroFrame = Missile.pitchFinDeflection * Missile.cosPhiPrime - Missile.yawFinDeflection * Missile.sinPhiPrime;
	Missile.pitchDeflectionAeroBallisticFrameDegrees = radToDeg * pitchDeflAeroFrame;
	double yawDeflAeroFrame = Missile.pitchFinDeflection * Missile.sinPhiPrime + Missile.yawFinDeflection * Missile.cosPhiPrime;
	Missile.yawDeflectionAeroBallisticFrameDegrees = radToDeg * yawDeflAeroFrame;
	Missile.rollFinDeflectionDegrees = radToDeg * Missile.rollFinDeflectionRadians;
	Missile.totalFinDeflectionDegrees = (abs(Missile.pitchDeflectionAeroBallisticFrameDegrees) + abs(Missile.yawDeflectionAeroBallisticFrameDegrees)) / 2;
	double pitchRateAeroFrame = Missile.bodyRate[1] * Missile.cosPhiPrime - Missile.bodyRate[2] * Missile.sinPhiPrime;
	Missile.pitchRateAeroBallisticFrameDegrees = radToDeg * pitchRateAeroFrame;
	double yawRateAeroFrame = Missile.bodyRate[1] * missile.sinPhiPrime + Missile.bodyRate[2] * Missile.cosPhiPrime;
	Missile.yawRateAeroBallisticFrameDegrees = radToDeg * yawRateAeroFrame;
	Missile.rollRateDegrees = radToDeg * Missile.bodyRate[0];
	Missile.sinOfFourTimesPhiPrime = sin(4 * phiPrime);
	Missile.squaredSinOfTwoTimesPhiPrime = pow((sin(2 * phiPrime)), 2);

}

void dataLookUp(MissilePacket &Missile)
{

	int index;

	index = Missile.tableNameIndexPairs["CA0"];
	Missile.CA0 = linearInterpolationWithBoundedEnds(Missile.tables[index], Missile.machSpeed);
	index = Missile.tableNameIndexPairs["CAA"];
	Missile.CAA = linearInterpolationWithBoundedEnds(Missile.tables[index], Missile.machSpeed);
	index = Missile.tableNameIndexPairs["CAD"];
	Missile.CAD = linearInterpolationWithBoundedEnds(Missile.tables[index], Missile.machSpeed);
	index = Missile.tableNameIndexPairs["CAOFF"];
	if (Missile.timeOfFlight <= ROCKET_BURN_OUT_TIME)
	{
		Missile.CAOFF = 0.0;
	}
	else
	{
		Missile.CAOFF = linearInterpolationWithBoundedEnds(Missile.tables[index], Missile.machSpeed);
	}
	index = Missile.tableNameIndexPairs["CYP"];
	Missile.CYP = biLinearInterpolationWithBoundedBorders(Missile.tables[index], Missile.machSpeed, Missile.alphaPrimeDegrees);
	Missile.CYDR = biLinearInterpolationWithBoundedBorders("CYDR", mslMach, alphaPrimeDeg);
	Missile.CN0 = biLinearInterpolationWithBoundedBorders("CN0", mslMach, alphaPrimeDeg);
	Missile.CNP = biLinearInterpolationWithBoundedBorders("CNP", mslMach, alphaPrimeDeg);
	Missile.CNDQ = biLinearInterpolationWithBoundedBorders("CNDQ", mslMach, alphaPrimeDeg);
	Missile.CLLAP = biLinearInterpolationWithBoundedBorders("CLLAP", mslMach, alphaPrimeDeg);
	Missile.CLLP = biLinearInterpolationWithBoundedBorders("CLLP", mslMach, alphaPrimeDeg);
	Missile.CLLDP = biLinearInterpolationWithBoundedBorders("CLLDP", mslMach, alphaPrimeDeg);
	Missile.CLM0 = biLinearInterpolationWithBoundedBorders("CLM0", mslMach, alphaPrimeDeg);
	Missile.CLMP = biLinearInterpolationWithBoundedBorders("CLMP", mslMach, alphaPrimeDeg);
	Missile.CLMQ = linearInterpolationWithBoundedEnds("CLMQ", mslMach);
	Missile.CLMDQ = biLinearInterpolationWithBoundedBorders("CLMDQ", mslMach, alphaPrimeDeg);
	Missile.CLNP = biLinearInterpolationWithBoundedBorders("CLNP", mslMach, alphaPrimeDeg);
	Missile.mass = linearInterpolationWithBoundedEnds("MASS", mslTof);
	Missile.unadjustedThrust = linearInterpolationWithBoundedEnds("THRUST", mslTof);
	Missile.transverseMomentOfInertia = linearInterpolationWithBoundedEnds("TMOI", mslTof);
	Missile.axialMomentOfInertia = linearInterpolationWithBoundedEnds("AMOI", mslTof);
	Missile.centerOfGravityFromNose = linearInterpolationWithBoundedEnds("CG", mslTof);

}

int main()
{

	MissilePacket originalMissile;
	auto copiedMissile = originalMissile; // Just a test. Works great. No pointers makes it easy!
	lookUpTablesFormat(originalMissile, "shortRangeInterceptorTables.txt");
	initializeMissile(originalMissile, "input.txt");

	// Functions test.
	timeOfFlight(originalMissile); // Soon to be handled by motion module.
	atmosphere(originalMissile);
	seeker(originalMissile);
	guidance(originalMissile);
	control(originalMissile);
	aeroBallisticAnglesAndConversions(originalMissile);

	cout << "HOWDY WORLD, FROM CPP NOVICE." << endl;
	cout << "\n";
	return 0;

}