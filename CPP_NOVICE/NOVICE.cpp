
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

// Missile structure.
struct MissilePacket
{

	/* Missile constants. */
	double missileReferenceArea = 0.01824; // M^2.
	double missileReferenceDiameter = 0.1524; // Meters.
	double missileNozzleExitArea = 0.0125; // M^2.
	double missileRocketBurnOut = 2.421; // Seconds.
	double missileSeekerG = 10.0; // Seeker Kalman filter gain. Per second.
	double missileSeekerZETA = 0.9; // Seeker Kalman filter damping. Non dimensional.
	double missileSeekerWN = 60.0; // Seeker Kalman filter natural frequency. Radians per second.
	double missileProportionalGuidanceGain = 3.0; // Guidance homing gain.
	double missileLineOfAttackGuidanceGain = 1.5; // Guidance midcourse gain.
	double missileMaximumAccelerationAllowed = 450.0; // Meters per s^2. Roughly 45 Gs.
	double missileConstantRateControlZETA = 0.6; // Damping of constant rate control. Non dimensional.
	double missileRollControlWN = 20.0; // Natural frequency of roll closed loop complex pole. Radians per second.
	double missileRollControlZETA = 0.9; // Damping of roll closed loop complex pole. Non dimensional.
	double missileControlMaxFinDeflection = 28.0; // Degrees.
	double missileRollAngleCommand = 0.0; // Degrees or radians.
	double missileTotalAngleOfAttackMaximum = 40.0; // Degrees.
	double missileSeaLevelPressure = 101325; // Pascals.

	/* Missile variables. */
	// Missile packet.
	double missilePip[3] = {0.0, 0.0, 0.0}; // Predicted intercept point. Meters.
	double missileTimeOfFlight = 0.0; // Seconds.
	double missileENUPosition[3] = {0.0, 0.0, 0.0}; // Meters.
	double missileRange = 0.0; // Meters.
	double missileENUVelocity[3] = {0.0, 0.0, 0.0}; // Meters per second.
	double missileFLUVelocity[3] = {0.0, 0.0, 0.0}; // Meters per second.
	double missileSpeed = 0.0; // Meters per second.
	double missileMachSpeed = 0.0; // Non dimensional.
	double missileENUAcceleration[3] = {0.0, 0.0, 0.0}; // Meters per s^2.
	double missileFLUAcceleration[3] = {0.0, 0.0, 0.0}; // Meters per s^2.
	double missileENUToFLUMatrix[3][3]; // Non dimensional.
	double missileAlpha = 0.0; // Radians.
	double missileBeta = 0.0; // Radians.
	double missileENUEulerAngles[3] = {0.0, 0.0, 0.0}; // Radians.
	double missileENUEulerDot[3] = {0.0, 0.0, 0.0}; // Radians per second.
	double missileRate[3] = {0.0, 0.0, 0.0}; // Radians per second.
	double missileRateDot[3] = {0.0, 0.0, 0.0}; // Radians per s^2.

	// Environment.
	double missileGravity = 0.0; // Meters per s^2.
	double missileFLUGravity[3] = {0.0, 0.0, 0.0}; // Meters per s^2.
	double missilePressure = 0.0; // Pascals.
	double missileDynamicPressure = 0.0; // Pascals.

	// Seeker.
	double missileSeekerPitch = 0.0; // Radians.
	double missileSeekerYaw = 0.0; // Radians.
	double missileSeekerENUToFLUMatrix[3][3]; // Non dimensional.
	double missileSeekerPitchError = 0.0; // Radians off target.
	double missileSeekerYawError = 0.0; // Radians off target.
	double missileSeekerWLR = 0.0; // Pointing yaw rate. Radians per second.
	double missileSeekerWLRD = 0.0; // Derivative of pointing yaw rate. Radians per s^2.
	double missileSeekerWLR1 = 0.0; // Yaw sight line spin rate. Radians per second.
	double missileSeekerWLR1D = 0.0; // Derivative of yaw sight line spin rate. Radians per s^2.
	double missileSeekerWLR2 = 0.0; // Second state variable in Kalman filter, yaw. Radians per s^2.
	double missileSeekerWLR2D = 0.0; // Derivative of second state variable in Kalman filter, yaw. Radians per s^3.
	double missileSeekerWLQ = 0.0; // Pointing pitch rate. Radians per second.
	double missileSeekerWLQD = 0.0; // Derivative of pointing pitch rate. Radians per s^2.
	double missileSeekerWLQ1 = 0.0; // pitch sight line spin rate. Radians per second.
	double missileSeekerWLQ1D = 0.0; // Derivative of pitch sight line spin rate. Radians per s^2.
	double missileSeekerWLQ2 = 0.0; // Second state variable in Kalman filter, pitch. Radians per s^2.
	double missileSeekerWLQ2D = 0.0; // Derivative of second state variable in Kalman filter, pitch. Radians per s^3.

	// Guidance.
	double missileToInterceptFLURelativePosition[3] = {0.0, 0.0, 0.0}; // Meters.
	double missileGuidanceNormalCommand = 0.0; // Meters per s^2.
	double missileGuidanceSideCommand = 0.0; // Meters per s^2.
	double missileMaximumAcceleration = 0.0; // Meters per s^2.

	// Control.
	double missileControlYY = 0.0; // Yaw feed forward integration. Meters per second.
	double missileControlYYD = 0.0; // Yaw feed forward derivative. Meters per second.
	double missileControlZZ = 0.0; // Pitch feed forward integration. Meters per second.
	double missileControlZZD = 0.0; // Pitch feed forward derivative. Meters per second.
	double missileRollFinCommand = 0.0; // Radians.
	double missilePitchFinCommand = 0.0; // Radians.
	double missileYawFinCommand = 0.0; // Radians.

	// Actuators.
	double missileRollFinDeflection = 0.0; // Radians.
	double missilePitchFinDeflection = 0.0; // Radians.
	double missileYawFinDeflection = 0.0; // Radians.
	double missileFinOneDeflection = 0.0; // Radians.
	double missileDEL1D = 0.0; // Fin one position derived. Radians.
	double missileDEL1DOT = 0.0; // Fin one rate. Radians per second.
	double missileDEL1DOTDOT = 0.0; // Fin one rate derived. Radians per s^2.
	double missileFinTwoDeflection = 0.0; // Radians.
	double missileDEL2D = 0.0; // Fin two position derived. Radians.
	double missileDEL2DOT = 0.0; // Fin two rate. Radians per second.
	double missileDEL2DOTDOT = 0.0; // Fin two rate derived. Radians per s^2.
	double missileFinThreeDeflection = 0.0; // Radians.
	double missileDEL3D = 0.0; // Fin three position derived. Radians.
	double missileDEL3DOT = 0.0; // Fin three rate. Radians per second.
	double missileDEL3DOTDOT = 0.0; // Fin three rate derived. Radians per s^2.
	double missileFinFourDeflection = 0.0; // Radians.
	double missileDEL4D = 0.0; // Fin four position derived. Radians.
	double missileDEL4DOT = 0.0; // Fin four rate. Radians per second.
	double missileDEL4DOTDOT = 0.0; // Fin four rate derived. Radians per s^2.

	// Aero ballistic angles.
	double missileAlphaPrimeDegrees = 0.0; // Total angle of attack. Degrees.
	double missileSinPhiPrime = 0.0; // Non dimensional.
	double missileCosPhiPrime = 0.0; // Non dimensional.
	double missilePitchDeflectionAeroBallisticFrameDegrees = 0.0; // Degrees.
	double missileYawDeflectionAeroBallisticFrameDegrees = 0.0; // Degrees.
	double missileRollDeflectionDegrees = 0.0; // Degrees.
	double missileTotalFinDeflectionDegrees = 0.0; // Degrees.
	double missilePitchRateAeroBallisticFrameDegrees = 0.0; // Degrees per second.
	double missileYawRateAeroBallisticFrameDegrees = 0.0; // Degrees per second.
	double missileRollRateDegrees = 0.0; // Degrees per second.
	double missileSinOfFourTimesPhiPrime = 0.0; // Non dimensional.
	double missileSquaredSinOfTwoTimesPhiPrime = 0.0; // Non dimensional.

	// Data look up.
	map<string, int> missileTableNameIndexPairs;
	vector<vector<vector<double>>> missileTables;
	double missileCA0 = 0.0; // Non dimensional.
	double missileCAA = 0.0; // Per degree.
	double missileCAD = 0.0; // Per degree^2.
	double missileCAOFF = 0.0; // Non dimensional.
	double missileCYP = 0.0; // Non dimensional.
	double missileCYDR = 0.0; // Per degree.
	double missileCN0 = 0.0; // Non dimensional.
	double missileCNP = 0.0; // Non dimensional.
	double missileCNDQ = 0.0; // Per degree.
	double missileCLLAP = 0.0; // Per degree^2.
	double missileCLLP = 0.0; // Per degree.
	double missileCLLDP = 0.0; // Per degree.
	double missileCLM0 = 0.0; // Non dimensional.
	double missileCLMP = 0.0; // Non dimensional.
	double missileCLMQ = 0.0; // Per degree.
	double missileCLMDQ = 0.0; // Per degree.
	double missileCLNP = 0.0; // Non dimensional
	double missileMass = 0.0; // Kilograms.
	double missileUnadjustedThrust = 0.0; // Newtons.
	double missileTransverseMomentOfInertia = 0.0; // Kilograms * M^2.
	double missileAxialMomentOfInertia = 0.0; // Kilograms * M^2.
	double missileCenterOfGravityFromNose = 0.0; // Meters.

	// Propulsion.
	double missileThrust = 0.0; // Newtons.

	// Aerodynamic coefficients.
	double missileLaunchCenterOfGravityFromNose = 0.0; // Meters.
	double missileCX = 0.0; // Non dimensional.
	double missileCY = 0.0; // Non dimensional.
	double missileCZ = 0.0; // Non dimensional.
	double missileCL = 0.0; // Non dimensional.
	double missileCM = 0.0; // Non dimensional.
	double missileCN = 0.0; // Non dimensional.

	// Aerodynamic control feedback values.
	double missileCNA = 0.0; // Per degree.
	double missileCMA = 0.0; // Per degree.
	double missileCND = 0.0; // Per degree.
	double missileCMD = 0.0; // Per degree.
	double missileCMQ = 0.0; // Per degree.
	double missileCLP = 0.0; // Per degree.
	double missileCLD = 0.0; // Per degree.
	double missileStaticMargin = 0.0; // Non dimensional.

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
	double missileMissDistance = 0.0; // Meters.
	string missileLethality; // For termination check.

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
							Missile.missileTables[tableNoTrack - 1][0].back() = dataPointDouble;
						}
						// IF THIS THE FIRST LOOP, THIS IS THE COLUMN IN THE DATA SET THAT DISPLAYS THE "ROWS" VALUES
						else if (columnCount == 1) {
							// FOR TWO DIMENSIONAL TABLE
							if (dimensions[tableNoTrack -1][1] != 2){
								// PLACE DATA POINT IN ITS PLACE
								Missile.missileTables[tableNoTrack - 1][rowNoTrack][0] = dataPointDouble;
							}
							// FOR ONE DIMENSIONAL TABLE
							else {
								// PLACE DATA POINT IN ITS PLACE
								Missile.missileTables[tableNoTrack - 1][rowNoTrack - 1][0] = dataPointDouble;
							}
						}
						// IF THIS THE SECOND LOOP, THIS IS THE COLUMN IN THE DATA SET THAT DISPLAYS THE "COLUMNS" VALUES, ONLY FOR TWO DIMENSIONAL TABLES
						else if (columnCount == 2 and dimensions[tableNoTrack -1][1] != 2) {
							// PLACE DATA POINT IN ITS PLACE
							Missile.missileTables[tableNoTrack - 1][0][rowNoTrack] = dataPointDouble;
						}
						// ELSE FOR ACTUAL DATA POINTS
						else {
							// FOR TWO DIMENSIONAL TABLES
							if (dimensions[tableNoTrack -1][1] != 2) {
								// PLACE DATA POINT IN ITS PLACE
								Missile.missileTables[tableNoTrack - 1][rowNoTrack][columnCount - 2] = dataPointDouble;
							}
							// FOR ONE DIMENSIONAL TABLES
							else {
								// PLACE DATA POINT IN ITS PLACE
								Missile.missileTables[tableNoTrack - 1][rowNoTrack - 1][columnCount - 1] = dataPointDouble;
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
			Missile.missileTables.push_back(newTable);
		}
		// STORE NAME OF TABLE
		else if (flag == 3){
			// MAP TABLE NAME INDEX PAIR
			Missile.missileTableNameIndexPairs.emplace(name, tableNoTrack - 1);
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
	Missile.missileTimeOfFlight = 0.0;

	// Open input file.
	std::ifstream inPut;
	inPut.open("input.txt");

	// Declare inputs.
	double phi, theta, psi, tgtE, tgtN, tgtU;

	// Populate inputs.
	inPut >> phi >> theta >> psi >> tgtE >> tgtN >> tgtU;

	// Populate the predicted intercept point.
	// Needs to be an input for flyouts.
	Missile.missilePip[0] = tgtE;
	Missile.missilePip[1] = tgtN;
	Missile.missilePip[2] = tgtU;

	// Convert the input euler angles to radians.
	phi *= degToRad;
	theta *= degToRad;
	psi *= degToRad;

	// Set missile ENU Euler angles.
	// Will be determined by the launcher.
	Missile.missileENUEulerAngles[0] = phi;
	Missile.missileENUEulerAngles[1] = theta;
	Missile.missileENUEulerAngles[2] = psi;

	// Initialize navigation direction cosine matrix.
	// Will be determined by the launcher.
	eulerAnglesToLocalOrientation(phi, -theta, psi, Missile.missileENUToFLUMatrix);

	// Set missile ENU position.
	// Will be determined by launcher.
	Missile.missileENUPosition[0] = 0.0;
	Missile.missileENUPosition[1] = 0.0;
	Missile.missileENUPosition[2] = 0.0;

	// Set missile ENU velocity.
	// Determined by launcher orientation.
	Missile.missileENUVelocity[0] = Missile.missileENUToFLUMatrix[0][0];
	Missile.missileENUVelocity[1] = Missile.missileENUToFLUMatrix[0][1];
	Missile.missileENUVelocity[2] = Missile.missileENUToFLUMatrix[0][2];

	// Initialize missile FLU velocity.
	threeByThreeTimesThreeByOne(
		Missile.missileENUToFLUMatrix,
		Missile.missileENUVelocity,
		Missile.missileFLUVelocity
	);

	// Initialize missile ENU and FLU acceleration.
	Missile.missileENUAcceleration[0] = 0.0;
	Missile.missileENUAcceleration[1] = 0.0;
	Missile.missileENUAcceleration[2] = 0.0;
	Missile.missileFLUAcceleration[0] = 0.0;
	Missile.missileFLUAcceleration[1] = 0.0;
	Missile.missileFLUAcceleration[2] = 0.0;

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
	subtractTwoVectors(Missile.missileENUPosition, Missile.missilePip, missileToInterceptENURelativePosition);
	unitVec(missileToInterceptENURelativePosition, missileToInterceptENURelativePositionUnit);
	threeByThreeTimesThreeByOne(
		Missile.missileENUToFLUMatrix,
		missileToInterceptENURelativePositionUnit,
		missileToInterceptFLURelativePositionUnit
	);
	azAndElFromVector(
		missileToInterceptFLURelativePositionAzimuth,
		missileToInterceptFLURelativePositionElevation,
		missileToInterceptFLURelativePositionUnit
	);
	Missile.missileSeekerPitch = missileToInterceptFLURelativePositionAzimuth;
	Missile.missileSeekerYaw = missileToInterceptFLURelativePositionElevation;
	flightPathAnglesToLocalOrientation(
		missileToInterceptFLURelativePositionAzimuth,
		-missileToInterceptFLURelativePositionElevation,
		seekerAttitudeToENUMatrix
	);
	threeByThreeTimesThreeByThree(
		seekerAttitudeToENUMatrix,
		Missile.missileENUToFLUMatrix,
		Missile.missileSeekerENUToFLUMatrix
	);
	Missile.missileSeekerPitchError = 0.0; // Since now pointed directly at target.
	Missile.missileSeekerYawError = 0.0; // Since now pointed directly at target.
	Missile.missileSeekerWLQ = Missile.missileSeekerPitch;
	Missile.missileSeekerWLR = Missile.missileSeekerYaw;

	// Look up launch center of gravity from nose.
	int tableIndex = Missile.missileTableNameIndexPairs["CG"];
	Missile.missileLaunchCenterOfGravityFromNose = linearInterpolationWithBoundedEnds(
		Missile.missileTables[tableIndex],
		Missile.missileTimeOfFlight
	);

	// Set lethality.
	Missile.missileLethality = "FLYING";

	// Console report.
	cout << "\n";
	cout << "MODEL INITIATED" << endl;
	cout << "\n";

}

void timeOfFlight(MissilePacket &Missile)
{

	Missile.missileTimeOfFlight += timeStep;

}

void atmosphere(MissilePacket &Missile)
{

	int tableIndex;
	double altitude;
	double rho;
	double missileENUGravity[3];
	double missileSpeedOfSound;

	altitude = Missile.missileENUPosition[2];
	tableIndex = Missile.missileTableNameIndexPairs["RHO"];
	rho = linearInterpolationWithBoundedEnds(Missile.missileTables[tableIndex], altitude);
	tableIndex = Missile.missileTableNameIndexPairs["GRAVITY"];
	Missile.missileGravity = linearInterpolationWithBoundedEnds(Missile.missileTables[tableIndex], altitude);
	tableIndex = Missile.missileTableNameIndexPairs["PRESSURE"];
	Missile.missilePressure = linearInterpolationWithBoundedEnds(Missile.missileTables[tableIndex], altitude);
	tableIndex = Missile.missileTableNameIndexPairs["SPEED_OF_SOUND"];
	missileSpeedOfSound = linearInterpolationWithBoundedEnds(Missile.missileTables[tableIndex], altitude);
	missileENUGravity[0] = 0.0;
	missileENUGravity[1] = 0.0;
	missileENUGravity[2] = -1.0 * Missile.missileGravity;
	threeByThreeTimesThreeByOne(
		Missile.missileENUToFLUMatrix,
		missileENUGravity,
		Missile.missileFLUGravity
	);
	magnitude(Missile.missileENUVelocity, Missile.missileSpeed); // METERS PER SECOND
	Missile.missileMachSpeed = Missile.missileSpeed / missileSpeedOfSound; // ND
	Missile.missileDynamicPressure = 0.5 * rho * Missile.missileSpeed * Missile.missileSpeed; // PASCALS

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

	wsq = Missile.missileSeekerWN * Missile.missileSeekerWN;
	gg = Missile.missileSeekerG * wsq;

	// Seeker yaw.
	wlr1d_new = Missile.missileSeekerWLR2;
	wlr1_new = trapezoidIntegrate(wlr1d_new, Missile.missileSeekerWLR1D, Missile.missileSeekerWLR1, timeStep);
	Missile.missileSeekerWLR1 = wlr1_new;
	Missile.missileSeekerWLR1D = wlr1d_new;
	wlr2d_new = gg * Missile.missileSeekerYawError - 2 * Missile.missileSeekerZETA * Missile.missileSeekerWN * Missile.missileSeekerWLR1D - wsq * Missile.missileSeekerWLR1;
	wlr2_new = trapezoidIntegrate(wlr2d_new, Missile.missileSeekerWLR2D, Missile.missileSeekerWLR2, timeStep);
	Missile.missileSeekerWLR2 = wlr2_new;
	Missile.missileSeekerWLR2D = wlr2d_new;

	// Yaw control.
	wlrd_new = Missile.missileSeekerWLR1 - Missile.missileRate[2];
	wlr_new = trapezoidIntegrate(wlrd_new, Missile.missileSeekerWLRD, Missile.missileSeekerWLR, timeStep);
	Missile.missileSeekerWLR = wlr_new;
	Missile.missileSeekerWLRD = wlrd_new;
	Missile.missileSeekerYaw = Missile.missileSeekerWLR;

	// Seeker pitch.
	wlq1d_new = Missile.missileSeekerWLQ2;
	wlq1_new = trapezoidIntegrate(wlq1d_new, Missile.missileSeekerWLQ1D, Missile.missileSeekerWLQ1, timeStep);
	Missile.missileSeekerWLQ1 = wlq1_new;
	Missile.missileSeekerWLQ1D = wlq1d_new;
	wlq2d_new = gg * Missile.missileSeekerPitchError - 2 * Missile.missileSeekerZETA * Missile.missileSeekerWN * Missile.missileSeekerWLQ1D - wsq * Missile.missileSeekerWLQ1;
	wlq2_new = trapezoidIntegrate(wlq2d_new, Missile.missileSeekerWLQ2D, Missile.missileSeekerWLQ2, timeStep);
	Missile.missileSeekerWLQ2 = wlq2_new;
	Missile.missileSeekerWLQ2D = wlq2d_new;

	// Pitch control.
	wlrd_new = Missile.missileSeekerWLQ1 - Missile.missileRate[1];
	wlq_new = trapezoidIntegrate(wlqd_new, Missile.missileSeekerWLQD, Missile.missileSeekerWLQ, timeStep);
	Missile.missileSeekerWLQ = wlq_new;
	Missile.missileSeekerWLQD = wlqd_new;
	Missile.missileSeekerPitch = Missile.missileSeekerWLQ;

	// Calculate FLU relative position for guidance.
	subtractTwoVectors(Missile.missileENUPosition, Missile.missilePip, missileToInterceptENURelativePosition);
	eulerAnglesToLocalOrientation(
		0.0,
		-Missile.missileSeekerPitch,
		Missile.missileSeekerYaw,
		seekerAttitudeToENUMatrix
	);
	threeByThreeTimesThreeByThree(
		seekerAttitudeToENUMatrix,
		Missile.missileENUToFLUMatrix,
		Missile.missileSeekerENUToFLUMatrix
	);
	threeByThreeTimesThreeByOne(
		Missile.missileSeekerENUToFLUMatrix,
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
		Missile.missileSeekerYawError,
		Missile.missileSeekerPitchError,
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
	vectorProjection(forwardLeftUpMissileToInterceptPositionUnitVector, Missile.missileFLUVelocity, forwardLeftUpMissileToInterceptLineOfSightVel);
	double timeToGo, forwardLeftUpMissileToInterceptPositionMagnitude, forwardLeftUpMissileToInterceptLineOfSightVelMagnitude;
	magnitude(Missile.missileToInterceptFLURelativePosition, forwardLeftUpMissileToInterceptPositionMagnitude);
	magnitude(forwardLeftUpMissileToInterceptLineOfSightVel, forwardLeftUpMissileToInterceptLineOfSightVelMagnitude);
	timeToGo = forwardLeftUpMissileToInterceptPositionMagnitude / forwardLeftUpMissileToInterceptLineOfSightVelMagnitude;
	if (timeToGo < 5)
	{
		double closingVelocity[3];
		multiplyVectorTimesScalar(-1.0, Missile.missileFLUVelocity, closingVelocity);
		double closingSpeed;
		magnitude(closingVelocity, closingSpeed);
		double TEMP1[3], TEMP2;
		crossProductTwoVectors(Missile.missileToInterceptFLURelativePosition, closingVelocity, TEMP1);
		dotProductTwoVectors(Missile.missileToInterceptFLURelativePosition, Missile.missileToInterceptFLURelativePosition, TEMP2);
		double lineOfSightRate[3];
		divideVectorByScalar(TEMP2, TEMP1, lineOfSightRate);
		double TEMP3, TEMP4[3];
		double proportionalGuidanceGain = 3.0;
		TEMP3 = -1 * Missile.missileProportionalGuidanceGain * closingSpeed;
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
		vectorProjection(lineOfAttack, Missile.missileFLUVelocity, forwardLeftUpMissileToInterceptLineOfAttackVel);
		double G = 1 - exp(-0.001 * forwardLeftUpMissileToInterceptPositionMagnitude);
		Missile.missileGuidanceNormalCommand = Missile.missileLineOfAttackGuidanceGain * (forwardLeftUpMissileToInterceptLineOfSightVel[2] + G * forwardLeftUpMissileToInterceptLineOfAttackVel[2]);
		Missile.missileGuidanceSideCommand = Missile.missileLineOfAttackGuidanceGain * (forwardLeftUpMissileToInterceptLineOfSightVel[1] + G * forwardLeftUpMissileToInterceptLineOfAttackVel[1]);
	}

}

void control(MissilePacket &Missile)
{

	if (Missile.missileMachSpeed > 0.6) // Maneuvering.
	{

		double DNA = Missile.missileCNA * (Missile.missileDynamicPressure * Missile.missileReferenceArea / Missile.missileMass); // METERS PER SECOND^2
		double DMA = Missile.missileCMA * (Missile.missileDynamicPressure * Missile.missileReferenceArea * Missile.missileReferenceDiameter / Missile.missileTransverseMomentOfInertia); // PER SECOND^2
		double DMD = Missile.missileCMD * (Missile.missileDynamicPressure * Missile.missileReferenceArea * Missile.missileReferenceDiameter / Missile.missileTransverseMomentOfInertia); // PER SECOND^2
		double DMQ = Missile.missileCMQ * (Missile.missileReferenceDiameter / (2 * Missile.missileSpeed)) * (Missile.missileDynamicPressure * Missile.missileReferenceArea * Missile.missileReferenceDiameter / Missile.missileTransverseMomentOfInertia); // PER SECOND
		double DLP = Missile.missileCLP * (Missile.missileReferenceDiameter / (2 * Missile.missileSpeed)) * (Missile.missileDynamicPressure * Missile.missileReferenceArea * Missile.missileReferenceDiameter / Missile.missileAxialMomentOfInertia); // PER SECOND
		double DLD = Missile.missileCLD * (Missile.missileDynamicPressure * Missile.missileReferenceArea * Missile.missileReferenceDiameter / Missile.missileAxialMomentOfInertia); // PER SECOND^2

		double WACL = 0.013 * sqrt(Missile.missileDynamicPressure) + 7.1;
		double ZACL = 0.000559 * sqrt(Missile.missileDynamicPressure) + 0.232;
		double PACL = 14;

		// FEEDBACK GAINS
		double GAINFB3 = WACL * WACL * PACL / (DNA * DMD);
		double GAINFB2 = (2 * ZACL * WACL + PACL + DMQ - DNA / Missile.missileSpeed) / DMD;
		double GAINFB1 = (
			WACL * WACL + 2 * ZACL * WACL * PACL + DMA + DMQ * DNA / Missile.missileSpeed - GAINFB2 * DMD * DNA / Missile.missileSpeed
		) / (DNA * DMD);

		// ROLL
		double GKP = (2 * Missile.missileRollControlWN * Missile.missileRollControlZETA + DLP) / DLD;
		double GKPHI = Missile.missileRollControlWN * Missile.missileRollControlWN / DLD;
		double EPHI = GKPHI * (Missile.missileRollAngleCommand - Missile.missileENUEulerAngles[0]);
		Missile.missileRollFinCommand = EPHI - GKP * Missile.missileRate[0];

		// PITCH
		double zzdNew = Missile.missileGuidanceNormalCommand - Missile.missileFLUAcceleration[2];
		double zzNew = trapezoidIntegrate(zzdNew, Missile.missileControlZZD, Missile.missileControlZZ, timeStep);
		Missile.missileControlZZ = zzNew;
		Missile.missileControlZZD = zzdNew;
		double deflPitch = -1 * GAINFB1 * Missile.missileFLUAcceleration[2] - GAINFB2 * Missile.missileRate[1] + GAINFB3 * Missile.missileControlZZ;
		if (abs(deflPitch) > Missile.missileControlMaxFinDeflection)
		{
			if (deflPitch > 0)
			{
				deflPitch = Missile.missileControlMaxFinDeflection;
			}
			else if (deflPitch < 0)
			{
				deflPitch = -1 * Missile.missileControlMaxFinDeflection;
			}
		}
		Missile.missilePitchFinCommand = deflPitch * degToRad;

		// YAW
		double yydNew = Missile.missileFLUAcceleration[1] - Missile.missileGuidanceSideCommand;
		double yyNew = trapezoidIntegrate(yydNew, Missile.missileControlYYD, Missile.missileControlYY, timeStep);
		Missile.missileControlYY = yyNew;
		Missile.missileControlYYD = yydNew;
		double deflYaw = GAINFB1 * Missile.missileFLUAcceleration[1] - GAINFB2 * Missile.missileRate[2] + GAINFB3 * Missile.missileControlYY;
		if (abs(deflYaw) > Missile.missileControlMaxFinDeflection)
		{
			if (deflYaw > 0)
			{
				deflYaw = Missile.missileControlMaxFinDeflection;
			}
			else if (deflYaw < 0)
			{
				deflYaw = -1 * Missile.missileControlMaxFinDeflection;
			}
		}
		Missile.missileYawFinCommand = deflYaw * degToRad;

	}
	else if (Missile.missileMachSpeed > 0.1) // Rate lock.
	{

		double DNA = Missile.missileCNA * (Missile.missileDynamicPressure * Missile.missileReferenceArea / Missile.missileMass); // METERS PER SECOND^2
		double DND = Missile.missileCND * (Missile.missileDynamicPressure * Missile.missileReferenceArea / Missile.missileMass); // METERS PER SECOND^2
		double DMA = Missile.missileCMA * (Missile.missileDynamicPressure * Missile.missileReferenceArea * Missile.missileReferenceDiameter / Missile.missileTransverseMomentOfInertia); // PER SECOND^2
		double DMD = Missile.missileCMD * (Missile.missileDynamicPressure * Missile.missileReferenceArea * Missile.missileReferenceDiameter / Missile.missileTransverseMomentOfInertia); // PER SECOND^2
		double DMQ = Missile.missileCMQ * (Missile.missileReferenceDiameter / (2 * Missile.missileSpeed)) * (Missile.missileDynamicPressure * Missile.missileReferenceArea * Missile.missileReferenceDiameter / Missile.missileTransverseMomentOfInertia); // PER SECOND
		double DLP = Missile.missileCLP * (Missile.missileReferenceDiameter / (2 * Missile.missileSpeed)) * (Missile.missileDynamicPressure * Missile.missileReferenceArea * Missile.missileReferenceDiameter / Missile.missileAxialMomentOfInertia); // PER SECOND
		double DLD = Missile.missileCLD * (Missile.missileDynamicPressure * Missile.missileReferenceArea * Missile.missileReferenceDiameter / Missile.missileAxialMomentOfInertia); // PER SECOND^2

		// ROLL
		double GKP = (2 * Missile.missileRollControlWN * Missile.missileRollControlZETA + DLP) / DLD;
		double GKPHI = Missile.missileRollControlWN * Missile.missileRollControlWN / DLD;
		double EPHI = GKPHI * (Missile.missileRollAngleCommand - Missile.missileENUEulerAngles[0]);
		Missile.missileRollFinCommand = EPHI - GKP * Missile.missileRate[0];

		// RATE CONTROL
		double ZRATE = DNA / Missile.missileSpeed - DMA * DND / (Missile.missileSpeed * DMD); // ND
		double AA = DNA / Missile.missileSpeed - DMQ; // ND
		double BB = -1 * DMA - DMQ * DNA / Missile.missileSpeed; // ND
		double TEMP1 = AA - 2 * Missile.missileConstantRateControlZETA * Missile.missileConstantRateControlZETA * ZRATE; // ND
		double TEMP2 = AA * AA - 4 * Missile.missileConstantRateControlZETA * Missile.missileConstantRateControlZETA * BB; // ND
		double RADIX = TEMP1 * TEMP1 - TEMP2; // ND
		double GRATE = (-1 * TEMP1 + sqrt(RADIX)) / (-1 * DMD); // ND

		// PITCH
		Missile.missilePitchFinCommand = GRATE * Missile.missileRate[1]; // RADIANS

		// YAW
		Missile.missileYawFinCommand = GRATE * Missile.missileRate[2]; // RADIANS

	}
	else // Free flight.
	{
		Missile.missileRollFinCommand = 0.0;
		Missile.missilePitchFinCommand = 0.0;
		Missile.missileYawFinCommand = 0.0;
	}

}

int main()
{

	MissilePacket originalMissile;
	auto copiedMissile = originalMissile; // Just a test. Works great. No pointers makes it easy!
	lookUpTablesFormat(originalMissile, "shortRangeInterceptorTables.txt");
	initializeMissile(originalMissile, "input.txt");

	// Functions test.
	timeOfFlight(originalMissile);
	atmosphere(originalMissile);
	seeker(originalMissile);
	guidance(originalMissile);
	control(originalMissile);

	cout << "HOWDY WORLD, FROM CPP NOVICE." << endl;
	cout << "\n";
	return 0;

}