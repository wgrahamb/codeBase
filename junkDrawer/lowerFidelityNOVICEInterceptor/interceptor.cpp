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

using namespace std;

#include "util.h"

// Simulation control.
auto wallClockStart = chrono::high_resolution_clock::now(); // Start tracking real time.
const double TIME_STEP = 0.01; // Seconds.
const double MAX_TIME = 200.0;

/* Missile Model */
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

// Missile constants.
const double REFERENCE_AREA = 0.01824; // Meters^2.
const double THRUST_EXIT_AREA = 0.0125; // Meters^2.
const double ROCKET_BURN_OUT_TIME = 2.421; // Seconds.
const double ALPHA_PRIME_MAX = 35.0; // Degrees.
const double SEA_LEVEL_PRESSURE = 101325.0; // Pascals.
const double PROPORTIONAL_GUIDANCE_GAIN = 3.0; // Non dimensional.
const int TRAJECTORY_TYPE_FLAG = 1; // 0 = Ballistic, 1 = Guided.

// Missile.
string lethality;
double missileTimeOfFlight;
double missileAzimuth;
double missileElevation;
double missileENUPosition[3];
double missileENUToFLUMatrix[3][3];
double missileENUVelocity[3];
double missileFLUVelocity[3];
double missileSpeed;
double missileMachSpeed;
double missileENUAcceleration[3];
double missileFLUAcceleration[3];
double missileAlpha;
double missileBeta;
double FLUMissileToInterceptRelativePosition[3];
double normalGuidanceCommand;
double sideGuidanceCommand;
double missDistance;
ofstream logFile;

// Tables.
map<string, int> tableNameIndexPairs;
vector<vector<vector<double>>> tables;

// Target.
double targetENUPosition[3];

// Parses text file with missile model tables. Should only be called once.
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

void init()
{

	ifstream input;
	input.open("input.txt");

	lookUpTablesFormat("shortRangeInterceptorTables.txt");

	input >> missileAzimuth >> missileElevation >> targetENUPosition[0] >> targetENUPosition[1] >> targetENUPosition[2];

	missileAzimuth *= degToRad;
	missileElevation *= degToRad;

	lethality = "FLYING";
	missileTimeOfFlight = 0.0;
	setArrayEquivalentToZero(missileENUPosition);
	flightPathAnglesToLocalOrientation(missileAzimuth, -1.0 * missileElevation, missileENUToFLUMatrix);
	missileENUVelocity[0] = missileENUToFLUMatrix[0][0];
	missileENUVelocity[1] = missileENUToFLUMatrix[0][1];
	missileENUVelocity[2] = missileENUToFLUMatrix[0][2];
	missileFLUVelocity[0] = 1.0;
	missileFLUVelocity[1] = 0.0;
	missileFLUVelocity[2] = 0.0;
	missileSpeed = 1.0;
	missileMachSpeed = 0.0;
	setArrayEquivalentToZero(missileENUAcceleration);
	setArrayEquivalentToZero(missileFLUAcceleration);
	missileAlpha = 0.0;
	missileBeta = 0.0;
	double ENUMissileToInterceptRelativePosition[3];
	subtractTwoVectors(missileENUPosition, targetENUPosition, ENUMissileToInterceptRelativePosition);
	threeByThreeTimesThreeByOne(missileENUToFLUMatrix, ENUMissileToInterceptRelativePosition, FLUMissileToInterceptRelativePosition);
	normalGuidanceCommand = 0.0;
	sideGuidanceCommand = 0.0;
	magnitude(FLUMissileToInterceptRelativePosition, missDistance);
	logFile.open("output/log.txt");
	logFile << fixed << setprecision(10) << "tof posE posN posU tgtE tgtN tgtU alpha beta lethality" << endl;

}

void threeDofFlyout()
{

	cout << "\nFLIGHT\n";
	cout << "\n";
	double lastTime = 0.0;

	while (lethality == "FLYING")
	{

		// Used throughout;
		int index;
		double TEMP;

		// Time of flight.
		missileTimeOfFlight += TIME_STEP;

		// Orientation.
		azAndElFromVector(missileAzimuth, missileElevation, missileENUVelocity);
		flightPathAnglesToLocalOrientation(missileAzimuth, -1.0 * missileElevation, missileENUToFLUMatrix);

		// Atmosphere.
		double altitude = missileENUPosition[2] * mToKm;
		index = tableNameIndexPairs["RHO"];
		double rho = linearInterpolationWithBoundedEnds(tables[index], altitude);
		index = tableNameIndexPairs["GRAVITY"];
		double gravity = linearInterpolationWithBoundedEnds(tables[index], altitude);
		double gravityENU[3] = {0.0, 0.0, -1.0 * gravity};
		double gravityFLU[3];
		threeByThreeTimesThreeByOne(missileENUToFLUMatrix, gravityENU, gravityFLU);
		index = tableNameIndexPairs["PRESSURE"];
		double pressure = linearInterpolationWithBoundedEnds(tables[index], altitude);
		index = tableNameIndexPairs["SPEED_OF_SOUND"];
		double a = linearInterpolationWithBoundedEnds(tables[index], altitude);
		magnitude(missileENUVelocity, missileSpeed);
		missileMachSpeed = missileSpeed / a;
		double dynamicPressure = 0.5 * rho * missileSpeed * missileSpeed;

		// Aero ballistic angles.
		double alphaPrimeRadians = acos(cos(missileAlpha) * cos(missileBeta));
		double alphaPrimeDegrees = alphaPrimeRadians * radToDeg;
		double phiPrimeRadians = atan2_0(tan(missileBeta), sin(missileAlpha));
		double phiPrimeDegrees = phiPrimeRadians * radToDeg;
		double alphaDegrees = missileAlpha * radToDeg;
		double betaDegrees = missileBeta * radToDeg;
		double sinOfFourTimesPhiPrime = sin(4 * phiPrimeRadians);
		TEMP = sin(2 * phiPrimeRadians);
		double squaredSinOfTwoTimesPhiPrime = TEMP * TEMP;
		double cosPhiPrime = cos(phiPrimeRadians);
		double sinPhiPrime = sin(phiPrimeRadians);

		// Mass and thrust look up.
		index = tableNameIndexPairs["MASS"];
		double mass = linearInterpolationWithBoundedEnds(tables[index], missileTimeOfFlight);
		index = tableNameIndexPairs["THRUST"];
		double unAdjustedThrust = linearInterpolationWithBoundedEnds(tables[index], missileTimeOfFlight);

		// Propulsion.
		double thrust;
		if (missileTimeOfFlight <= ROCKET_BURN_OUT_TIME)
		{
			thrust = unAdjustedThrust + (SEA_LEVEL_PRESSURE - pressure) * THRUST_EXIT_AREA;
		}
		else
		{
			thrust = 0.0;
		}

		// Axial coefficient look ups.
		index = tableNameIndexPairs["CA0"];
		double CA0 = linearInterpolationWithBoundedEnds(tables[index], missileMachSpeed);
		index = tableNameIndexPairs["CAA"];
		double CAA = linearInterpolationWithBoundedEnds(tables[index], missileMachSpeed);
		double CAOFF;
		if (missileTimeOfFlight <= ROCKET_BURN_OUT_TIME)
		{
			CAOFF = 0.0;
		}
		else
		{
			index = tableNameIndexPairs["CAOFF"];
			CAOFF = linearInterpolationWithBoundedEnds(tables[index], missileMachSpeed);
		}

		// Side coefficient look ups.
		index = tableNameIndexPairs["CYP"];
		double CYP_Actual = biLinearInterpolationWithBoundedBorders(tables[index], missileMachSpeed, alphaPrimeDegrees);
		double CYP_Max = biLinearInterpolationWithBoundedBorders(tables[index], missileMachSpeed, ALPHA_PRIME_MAX);

		// Normal coefficient look ups.
		index = tableNameIndexPairs["CN0"];
		double CN0_Actual = biLinearInterpolationWithBoundedBorders(tables[index], missileMachSpeed, alphaPrimeDegrees);
		double CN0_Max = biLinearInterpolationWithBoundedBorders(tables[index], missileMachSpeed, ALPHA_PRIME_MAX);
		index = tableNameIndexPairs["CNP"];
		double CNP_Actual = biLinearInterpolationWithBoundedBorders(tables[index], missileMachSpeed, alphaPrimeDegrees);
		double CNP_Max = biLinearInterpolationWithBoundedBorders(tables[index], missileMachSpeed, ALPHA_PRIME_MAX);

		// Axial coefficients.
		double CX_Actual = CA0 + CAA * alphaPrimeDegrees + CAOFF;

		// Side and normal coefficients.
		double CYAERO_Actual = CYP_Actual * sinOfFourTimesPhiPrime;
		double CYAERO_Max = CYP_Max * sinOfFourTimesPhiPrime;
		double CZAERO_Actual = CN0_Actual + CNP_Actual * squaredSinOfTwoTimesPhiPrime;
		double CZAERO_Max = CN0_Max + CNP_Max * squaredSinOfTwoTimesPhiPrime;
		double CY_Actual = CYAERO_Actual * cosPhiPrime - CZAERO_Actual * sinPhiPrime;
		double CY_Max = CYAERO_Max * cosPhiPrime - CZAERO_Max * sinPhiPrime;
		double CZ_Actual = CYAERO_Actual * sinPhiPrime + CZAERO_Actual * cosPhiPrime;
		double CZ_Max = CYAERO_Max * sinPhiPrime + CZAERO_Max * cosPhiPrime;

		// Limit guidance commands.
		double maxNormalAccelerationAllowed = abs((CZ_Max * dynamicPressure * REFERENCE_AREA) / mass);
		double maxSideAccelerationAllowed = abs((CY_Max * dynamicPressure * REFERENCE_AREA) / mass);

		// Guidance.
		double ENUMissileToInterceptRelativePosition[3];
		subtractTwoVectors(missileENUPosition, targetENUPosition, ENUMissileToInterceptRelativePosition);
		threeByThreeTimesThreeByOne(missileENUToFLUMatrix, ENUMissileToInterceptRelativePosition, FLUMissileToInterceptRelativePosition);
		double forwardLeftUpMissileToInterceptPositionUnitVector[3];
		unitVec(FLUMissileToInterceptRelativePosition, forwardLeftUpMissileToInterceptPositionUnitVector);
		double forwardLeftUpMissileToInterceptLineOfSightVel[3];
		vectorProjection(forwardLeftUpMissileToInterceptPositionUnitVector, missileFLUVelocity, forwardLeftUpMissileToInterceptLineOfSightVel);
		double timeToGo, forwardLeftUpMissileToInterceptPositionMagnitude, forwardLeftUpMissileToInterceptLineOfSightVelMagnitude;
		magnitude(FLUMissileToInterceptRelativePosition, forwardLeftUpMissileToInterceptPositionMagnitude);
		magnitude(forwardLeftUpMissileToInterceptLineOfSightVel, forwardLeftUpMissileToInterceptLineOfSightVelMagnitude);
		timeToGo = forwardLeftUpMissileToInterceptPositionMagnitude / forwardLeftUpMissileToInterceptLineOfSightVelMagnitude;
		double closingVelocity[3];
		multiplyVectorTimesScalar(-1.0, missileFLUVelocity, closingVelocity);
		double closingSpeed;
		magnitude(closingVelocity, closingSpeed);
		double TEMP1[3], TEMP2;
		crossProductTwoVectors(FLUMissileToInterceptRelativePosition, closingVelocity, TEMP1);
		dotProductTwoVectors(FLUMissileToInterceptRelativePosition, FLUMissileToInterceptRelativePosition, TEMP2);
		double lineOfSightRate[3];
		divideVectorByScalar(TEMP2, TEMP1, lineOfSightRate);
		double TEMP3, TEMP4[3];
		TEMP3 = -1 * PROPORTIONAL_GUIDANCE_GAIN * closingSpeed;
		multiplyVectorTimesScalar(TEMP3, forwardLeftUpMissileToInterceptPositionUnitVector, TEMP4);
		double COMMAND[3];
		crossProductTwoVectors(TEMP4, lineOfSightRate, COMMAND);
		normalGuidanceCommand = COMMAND[2];
		double sign;
		sign = signum(normalGuidanceCommand);
		if (abs(normalGuidanceCommand) > maxNormalAccelerationAllowed)
		{
			normalGuidanceCommand = maxNormalAccelerationAllowed * sign;
		}
		sideGuidanceCommand = COMMAND[1];
		sign = signum(sideGuidanceCommand);
		if (abs(sideGuidanceCommand) > maxSideAccelerationAllowed)
		{
			sideGuidanceCommand = maxSideAccelerationAllowed * sign;
		}

		// Forces
		double axialForce = thrust - CX_Actual * dynamicPressure * REFERENCE_AREA + gravityFLU[0] * mass;
		double sideForce = CY_Actual * dynamicPressure * REFERENCE_AREA + gravityFLU[1] * mass;
		double normalForce = CZ_Actual * dynamicPressure * REFERENCE_AREA + gravityFLU[2] * mass;

		// Specific force.
		missileFLUAcceleration[0] = axialForce / mass;
		missileFLUAcceleration[1] = sideForce / mass + sideGuidanceCommand;
		missileFLUAcceleration[2] = normalForce / mass + normalGuidanceCommand;
		oneByThreeTimesThreeByThree(missileFLUAcceleration, missileENUToFLUMatrix, missileENUAcceleration);

		// Motion integration.
		double deltaPos[3];
		multiplyVectorTimesScalar(TIME_STEP, missileENUVelocity, deltaPos);
		double newMissileENUPosition[3];
		addTwoVectors(missileENUPosition, deltaPos, newMissileENUPosition);
		setArrayEquivalentToReference(missileENUPosition, newMissileENUPosition);

		double deltaVel[3];
		multiplyVectorTimesScalar(TIME_STEP, missileENUAcceleration, deltaVel);
		double newMissileENUVelocity[3];
		addTwoVectors(missileENUVelocity, deltaVel, newMissileENUVelocity);
		setArrayEquivalentToReference(missileENUVelocity, newMissileENUVelocity);

		// Alpha and beta.
		threeByThreeTimesThreeByOne(missileENUToFLUMatrix, missileENUVelocity, missileFLUVelocity);
		missileAlpha = -1.0 * atan2_0(missileFLUVelocity[2], missileFLUVelocity[0]);
		missileBeta = atan2_0(missileFLUVelocity[1], missileFLUVelocity[0]);

		// Performance and termination check.
		magnitude(FLUMissileToInterceptRelativePosition, missDistance);

		if (missileENUPosition[2] < 0)
		{
			lethality = "GROUND_COLLISION";
		}
		else if (missDistance < 5.0)
		{
			lethality = "SUCCESSFUL_INTERCEPT";
		}
		else if (FLUMissileToInterceptRelativePosition[0] < 0)
		{
			lethality = "POINT_OF_CLOSEST_APPROACH_PASSED";
		}
		else if (isnan(missileENUPosition[0]))
		{
			lethality = "NOT_A_NUMBER";
		}
		else if (missileTimeOfFlight > MAX_TIME)
		{
			lethality = "MAX_TIME_EXCEEDED";
		}

		// Log data.
		logFile << fixed << setprecision(10) <<
		missileTimeOfFlight << " " <<
		missileENUPosition[0] << " " <<
		missileENUPosition[1] << " " <<
		missileENUPosition[2] << " " <<
		targetENUPosition[0] << " " <<
		targetENUPosition[1] << " " <<
		targetENUPosition[2] << " " <<
		missileAlpha << " " <<
		missileBeta << " " <<
		lethality << endl;

		auto print_it = static_cast<int>(round(missileTimeOfFlight * 10000.0)) % 10000;
		if (print_it == 0)
		{
			cout << setprecision(6) << "STATUS AT " << missileTimeOfFlight << " E " << missileENUPosition[0] << " N " << missileENUPosition[1] << " U " << missileENUPosition[2] << " MACH " << missileMachSpeed << endl;
			lastTime = missileTimeOfFlight;
		}

	}

	cout << setprecision(6) << missileTimeOfFlight << " E " << missileENUPosition[0] << " N " << missileENUPosition[1] << " U " << missileENUPosition[2] << " MACH " << missileMachSpeed << endl;

	cout << "\n";
	cout << "FLYOUT REPORT" << endl;
	cout << setprecision(6) << "FINAL POSITION AT " << missileTimeOfFlight << " E " << missileENUPosition[0] << " N " << missileENUPosition[1] << " U " << missileENUPosition[2] << " MACH " << missileMachSpeed << endl;
	cout << setprecision(6) << "MISS DISTANCE " << missDistance << " FORWARD, LEFT, UP, MISS DISTANCE " << FLUMissileToInterceptRelativePosition[0] << " " << FLUMissileToInterceptRelativePosition[1] << " " << FLUMissileToInterceptRelativePosition[2] << endl;
	cout << "SIMULATION RESULT: " << lethality << endl;
	auto wallClockEnd = chrono::high_resolution_clock::now();
	auto simRealRunTime = chrono::duration_cast<chrono::milliseconds>(wallClockEnd - wallClockStart);
	cout << "SIMULATION RUN TIME : " << (simRealRunTime.count() / 1000.0) << " SECONDS" << endl;
	cout << "\n";

}

int main()
{

	init();
	threeDofFlyout();
	return 0;

}