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

/*
#
# AUTHOR - WILSON GRAHAM BEECH
# REFERENCE - MODELING AND SIMULATION OF AEROSPACE VEHICLE DYNAMICS, SECOND EDITON - PETER H. ZIPFEL
#
# EAST, NORTH, UP COORDINATE SYSTEM
#
# INTERCEPTOR LOCAL ORIENTATION
# ARRAY 0 >>> LOOKING DOWN THE NOZZLE OF THE INTERCEPTOR
# ARRAY 1 >>> LOOKING DOWN THE NOZZLE OF THE INTERCEPTOR, THIS POINTS OUT THE LEFT HAND SIDE
# ARRAY 2 >>> LOOKING DOWN THE NOZZLE OF THE INTERCEPTOR, THIS POINTS OUT THE TOP SIDE OF THE INTERCEPTOR
#
#                           POSITIVE NORMAL
#                                         |
#                                         |
#                                         |
#  POSITIVE SIDE -----------O----------- NEGATIVE SIDE
#                                         |
#                                         |
#                                         |
#                          NEGATIVE NORMAL
#
# NEGATIVE AXIS IS COMING OUT OF THE SCREEN STRAIGHT AT YOU
# POSITIVE AXIS IS POINTING INTO THE SCREEN DIRECTLY AWAY FROM YOU
#
# POSITIVE ALPHA INDICATES NOSE BELOW FREE STREAM VELOCITY
# POSITIVE BETA INDICATES NOSE LEFT FREE STREAM VELOCITY
# POSITIVE ROLL INDICATES NORMAL AXIS CLOCKWISELY ROTATED FROM TWELVE O'CLOCK
#
# FIN ORIENTATION
# LOOKING DOWN THE NOZZLE OF THE MISSILE
#
#                              FIN 4       FIN 1
#                                         X
#                              FIN 3       FIN 2
#
*/

//////// GLOBALS ////////
// CONSTANTS
double degToRad = 0.0174532925199432;
double radToDeg = 57.2957795130823;
double mToKm = 1.0 / 1000.0;
double pi = 3.14159;

// SIMULATION CONTROL
auto wallClockStart = chrono::high_resolution_clock::now();
bool go = true;
double timeStep = 0.001; // SECONDS
double integrationStep = 0.001; // SECONDS
double maxTime = 400; // SECONDS

// STRUCT FOR END CHECKS
struct endChecks {
	string successfulIntercept;
	string flying;
	string groundCollision;
	string pointOfClosestApproachPassed;
	string notANumber;
	string maxTimeExceeded;
	string forcedSimTermination;
};

// INIT END CHECK STRUCT
endChecks ec;

// PREDICTED INTERCEPT POINT
double pip[3];

// MISSILE PACKET
double mslTof = 0.0; // SECONDS
double mslAz;
double mslEl;
double mslLocalOrient[3][3];
double mslPos[3];
double mslRange = 0.0; // METERS
double mslVel[3];
double mslBodyVel[3];
double mslSpeed;
double mslMach = 0.0; // ND
double mslAcc[3];
double mslBodyAcc[3];
double mslAlpha = 0.0; // RADIANS
double mslBeta = 0.0; // RADIANS
double mslEuler[3];
double mslEulerDot[3] = {0.0, 0.0, 0.0}; // RADIANS PER SECOND
double mslRate[3] = {0.0, 0.0, 0.0}; // RADIANS PER SECOND
double mslRateDot[3] = {0.0, 0.0, 0.0}; // RADIANS PER SECOND^2
double mslRefArea = 0.01824; // METERS^2
double mslRefDiam = 0.1524; // METERS^2
double mslExitArea = 0.0125; // METERS^2
double mslBurnOut = 2.421; // SECONDS


/* FXN VARIABLES AND LOOP FLOW */

// TIME OF FLIGHT

// ENVIRONMENT
double grav = 0.0; // METERS PER SECOND^2
double gravBodyVec[3] = {0.0, 0.0, 0.0}; // METERS PER SECOND^2
double press = 0.0; // PASCALS
double q = 0.0; // PASCALS

// SEEKER
double seekerPitch;
double seekerYaw;
double seekerLocalOrient[3][3];
double seekerPitchErr;
double seekerYawErr;
double gk = 10; // KALMAN FILTER GAIN >>> PER SECOND
double zetak = 0.9; // KALMAN FILTER DAMPING
double wnk = 60; // KALMAN FILTER NATURAL FREQUENCY >>> RADIANS PER SECOND
double wlr; // RADIANS PER SECOND >>> POINTING YAW RATE
double wlrd = 0.0; // RADIANS PER SECOND^2 >>> DERIVATIVE OF POINTING YAW RATE
double wlr1 = 0.0; // RADIANS PER SECOND >>> YAW SIGHT LINE SPIN RATE
double wlr1d = 0.0; // RADIANS PER SECOND^2 >>> DERIVATIVE OF YAW SIGHT LINE SPIN RATE
double wlr2 = 0.0; // RADIANS PER SECOND^2 >>> SECOND STATE VARIABLE IN KALMAN FILTER, YAW
double wlr2d = 0.0; // RADIANS PER SECOND^3 >>> DERIVATIVE OF SECOND STATE VARIABLE IN KALMAN FILTER, YAW
double wlq; // RADIANS PER SECOND >>> POINTING PITCH RATE
double wlqd = 0.0; // RADIANS PER SECOND^2 >>> DERIVATIVE OF POINTING PITCH RATE
double wlq1 = 0.0; // RADIANS PER SECOND >>> PITCH SIGHT LINE SPIN RATE
double wlq1d = 0.0; // RADIANS PER SECOND^2 >>> DERIVATIVE OF PITCH SIGHT LINE SPIN RATE
double wlq2 = 0.0; // RADIANS PER SECOND^2 >>> SECOND STATE VARIABLE IN KALMAN FILTER, PITCH
double wlq2d = 0.0; // RADIANS PER SECOND^3 >>> DERIVATIVE OF SECOND STATE VARIABLE IN KALMAN FILTER, PITCH

// GUIDANCE
double forwardLeftUpMslToInterceptPos[3] = {0.0, 0.0, 0.0}; // METERS
double proNavGain = 4; // ND
double normCommand = 0.0; // METERS PER SECOND^2
double sideCommand = 0.0; // METERS PER SECOND^2
double maxAccelAllow = 500.0; // METERS PER SECOND^2 >>> ROUGHLY FIFTY Gs
double maxAccel = maxAccelAllow; // METERS PER SECOND^2

// CONTROL
double zetlagr = 0.6; // DAMPING OF CLOSED RATE LOOP >>> ND
double wrcl = 20; // FREQUENCY OF ROLL CLOSED LOOP COMPLEX POLE >>> RADIANS PER SECOND
double zrcl = 0.9; //DAMPING OF ROLL CLOSED LOOP POLE >>> ND
double yy = 0.0; // METERS PER SECOND >>> YAW FEED FORWARD INTEGRATION
double yyd = 0.0; // METERS PER SECOND >>> YAW FEED FORWARD DERIVATIVE
double zz = 0.0; // METERS PER SECOND >>> PITCH FEED FORWARD INTEGRATION
double zzd = 0.0; // METERS PER SECOND >>> PITCH FEED FORWARD DERIVATIVE
double maxDefl = 28.0; // DEGREES
double pitchFinComm = 0.0; // RADIANS
double yawFinComm = 0.0; // RADIANS
double rollFinComm = 0.0; // RADIANS
double rollAngleComm = 0.0; // RADIANS

// FIN ACTUATION
double pitchFinDefl = 0.0; // RADIANS
double yawFinDefl = 0.0; // RADIANS
double rollFinDefl = 0.0; // RADIANS
double DEL1 = 0.0; // RADIANS >>> FIN DEFLECTION
double DEL1D = 0.0; // RADIANS >>> FIN POSITION DERIVED
double DEL1DOT = 0.0; // RADIANS PER SECOND >>> FIN RATE
double DEL1DOTDOT = 0.0; // RADIANS PER S^2 >>> FIN RATE DERIVED
double DEL2 = 0.0; // RADIANS >>> FIN DEFLECTION
double DEL2D = 0.0; // RADIANS >>> FIN POSITION DERIVED
double DEL2DOT = 0.0; // RADIANS PER SECOND >>> FIN RATE
double DEL2DOTDOT = 0.0; // RADIANS PER S^2 >>> FIN RATE DERIVED
double DEL3 = 0.0; // RADIANS >>> FIN DEFLECTION
double DEL3D = 0.0; // RADIANS >>> FIN POSITION DERIVED
double DEL3DOT = 0.0; // RADIANS PER SECOND >>> FIN RATE
double DEL3DOTDOT = 0.0; // RADIANS PER S^2 >>> FIN RATE DERIVED
double DEL4 = 0.0; // RADIANS >>> FIN DEFLECTION
double DEL4D = 0.0; // RADIANS >>> FIN POSITION DERIVED
double DEL4DOT = 0.0; // RADIANS PER SECOND >>> FIN RATE
double DEL4DOTDOT = 0.0; // RADIANS PER S^2 >>> FIN RATE DERIVED
double finRadianLimit = 0.4887; // RADIANS
double finRateRadianLimit = 10.472; // RADIANS PER SECOND
double WNACT = 100; // NATURAL FREQUENCY OF ACTUATOR >>> RADIANS PER SECOND
double ZETACT = 0.7; // DAMPING OF ACTUATOR

// AERO BALLISTIC ANGLES
double alphaPrimeDeg = 0.0; // DEGREES
double sinPhiPrime = 0.0; // ND
double cosPhiPrime = 0.0; // ND
double pitchDeflAeroDeg = 0.0; // DEGREES
double yawDeflAeroDeg = 0.0; // DEGREES
double rollDeflDeg = 0.0; // DEGREES
double totalFinDeflDeg = 0.0; // DEGREES
double pitchRateAeroDeg = 0.0; // DEGREES PER SECOND
double yawRateAeroDeg = 0.0; // DEGREES PER SECOND
double rollRateDeg = 0.0; // DEGREES PER SECOND
double sinOfFourTimesPhiPrime = 0.0; // ND
double squaredSinOfTwoTimesPhiPrime = 0.0; // ND

// DATA LOOK UP
map<string, int> tableNameIndexPairs;
vector<vector<vector<double>>> tables;
double CA0 = 0.0; // ND
double CAA = 0.0; // PER DEGREE
double CAD = 0.0; // PER DEGREE^2
double CAOFF = 0.0; // ND
double CYP = 0.0; // ND 
double CYDR = 0.0; // PER DEGREE
double CN0 = 0.0; // ND 
double CNP = 0.0; // ND 
double CNDQ = 0.0; // PER DEGREE
double CLLAP = 0.0; // PER DEGREE^2
double CLLP = 0.0; // PER DEGREE
double CLLDP = 0.0; // PER DEGREE
double CLM0 = 0.0; // ND
double CLMP = 0.0; // ND
double CLMQ = 0.0; // PER DEGREE
double CLMDQ = 0.0; // PER DEGREE
double CLNP = 0.0; // ND
double mass = 0.0; // KILOGRAMS
double unAdjThrust = 0.0; // NEWTONS
double transverseMomentOfInertia = 0.0; // KILOGRAMS * METERS^2
double axialMomentOfInertia = 0.0; // KILOGRAMS * METERS^2
double cgFromNose = 0.0; // METERS
double alphaPrimeMax = 40.0; // DEGREES

// PROPULSION
double seaLevelPress = 101325; // PASCALS
double thrust = 0.0; // NEWTONS

// AERO DYNAMIC COEFFICIENTS
double launchCg;
double CX = 0.0; // ND
double CY = 0.0; // ND
double CZ = 0.0; // ND
double CL = 0.0; // ND
double CM = 0.0; // ND
double CN = 0.0; // ND

// AERO DYNAMIC REFERENCE VALUES
double CNA = 0.0; // PER DEGREE
double CMA = 0.0; // PER DEGREE
double CND = 0.0; // PER DEGREE
double CMD = 0.0; // PER DEGREE
double CMQ = 0.0; // PER DEGREE
double CLP = 0.0; // PER DEGREE
double CLD = 0.0; // PER DEGREE
double staticMargin = 0.0; // ND

// TRANSLATIONAL ACCELERATION

// ROTATIONAL ACCELERATION

// TRANSLATIONAL INTEGRATION

// ROTATIONAL INTEGRATION

// INTERCEPT
double missDistance = 0.0; // METERS

// END CHECK
string lethality;

// LOG DATA
ofstream logFile;

// FLY

// MAIN

double integrate(double dy_new, double dy, double y, double intStep) {
	return y + ((dy_new + dy) * intStep / 2);
}

void flightPathAnglesToLocalOrientation (double azimuth, double elevation, double localFrame[3][3]) {
	localFrame[0][0] = cos(elevation) * cos(azimuth);
	localFrame[0][1] = cos(elevation) * sin(azimuth);
	localFrame[0][2] = -1 * sin(elevation);
	localFrame[1][0] = -1 * sin(azimuth);
	localFrame[1][1] = cos(azimuth);
	localFrame[1][2] = 0;
	localFrame[2][0] = sin(elevation) * cos(azimuth);
	localFrame[2][1] = sin(elevation) * sin(azimuth);
	localFrame[2][2] = cos(elevation);
}

void eulerAnglesToLocalOrientation (double phi, double theta, double psi, double matrix[3][3]) {
	matrix[0][0] = cos(psi) * cos(theta);
	matrix[0][1] = sin(psi) * cos(theta);
	matrix[0][2] = -1 * sin(theta);
	matrix[1][0] = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi);
	matrix[1][1] = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
	matrix[1][2] = cos(theta) * sin(phi);
	matrix[2][0] = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);
	matrix[2][1] = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);
	matrix[2][2] = cos(theta) * cos(phi);
}

void unitVec (double vector[3], double unitVector[3]) {
	double mag = sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
	unitVector[0] = vector[0] / mag;
	unitVector[1] = vector[1] / mag;
	unitVector[2] = vector[2] / mag;
}

void azAndElFromVector (double &az, double &el, double vector[3]) {
	az = atan2(vector[1], vector[0]);
	double t1 = pow(vector[0], 2);
	double t2 = pow(vector[1], 2);
	double t3 = sqrt(t1 + t2);
	el = atan2(vector[2], t3);
}

void oneByThreeTimesThreeByThree(double arr[3], double matrix[3][3], double out[3]) {
	out[0] = matrix[0][0] * arr[0] + matrix[1][0] * arr[1] + matrix[2][0] * arr[2];
	out[1] = matrix[0][1] * arr[0] + matrix[1][1] * arr[1] + matrix[2][1] * arr[2];
	out[2] = matrix[0][2] * arr[0] + matrix[1][2] * arr[1] + matrix[2][2] * arr[2];
}

void threeByThreeTimesThreeByOne(double matrix[3][3], double arr[3], double out[3]) {
	out[0] = matrix[0][0] * arr[0] + matrix[0][1] * arr[1] + matrix[0][2] * arr[2];
	out[1] = matrix[1][0] * arr[0] + matrix[1][1] * arr[1] + matrix[1][2] * arr[2];
	out[2] = matrix[2][0] * arr[0] + matrix[2][1] * arr[1] + matrix[2][2] * arr[2];
}

void threeByThreeTimesThreeByThree(double mat1[3][3], double mat2[3][3], double out[3][3]) {
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			out[i][j] = mat1[i][0] * mat2[0][j] + mat1[i][1] * mat2[1][j] + mat1[i][2] * mat2[2][j];
		}
	}
}

void magnitude(double arr[3], double &out) {
	double t1 = pow(arr[0], 2);
	double t2 = pow(arr[1], 2);
	double t3 = pow(arr[2], 2);
	out = sqrt(t1 + t2 + t3);
}

void subtractTwoVectors(double vec1[3], double vec2[3], double out[3]) {
	out[0] = vec2[0] - vec1[0];
	out[1] = vec2[1] - vec1[1];
	out[2] = vec2[2] - vec1[2];
}

void addTwoVectors(double vec1[3], double vec2[3], double out[3]) {
	out[0] = vec2[0] + vec1[0];
	out[1] = vec2[1] + vec1[1];
	out[2] = vec2[2] + vec1[2];
}

void multiplyTwoVectors(double vec1[3], double vec2[3], double out[3]) {
	out[0] = vec2[0] * vec1[0];
	out[1] = vec2[1] * vec1[1];
	out[2] = vec2[2] * vec1[2];
}

void crossProductTwoVectors (double vec1[3], double vec2[3], double out[3]) {
	out[0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
	out[1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
	out[2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
}

void dotProductTwoVectors (double vec1[3], double vec2[3], double &out) {
	out = vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2];
}

void vectorProjection (double uv[3], double vec[3], double out[3]) {
	double uvXvu[3][3];
	uvXvu[0][0] = uv[0] * uv[0];
	uvXvu[0][1] = uv[0] * uv[1];
	uvXvu[0][2] = uv[0] * uv[2];
	uvXvu[1][0] = uv[1] * uv[0];
	uvXvu[1][1] = uv[1] * uv[1];
	uvXvu[1][2] = uv[1] * uv[2];
	uvXvu[2][0] = uv[2] * uv[0];
	uvXvu[2][1] = uv[2] * uv[1];
	uvXvu[2][2] = uv[2] * uv[2];
	out[0] = uvXvu[0][0] * vec[0] + uvXvu[0][1] * vec[1] + uvXvu[0][2] * vec[2];
	out[1] = uvXvu[1][0] * vec[0] + uvXvu[1][1] * vec[1] + uvXvu[1][2] * vec[2];
	out[2] = uvXvu[2][0] * vec[0] + uvXvu[2][1] * vec[1] + uvXvu[2][2] * vec[2];
}

void multiplyVectorTimesScalar(double scalar, double vec[3], double out[3]) {
	out[0] = scalar * vec[0];
	out[1] = scalar * vec[1];
	out[2] = scalar * vec[2];
}

void lookUpTablesFormat () {
	// LOOK UP DATA
	ifstream inFile("CPP_6DOF_SRAAM_V1/sixDofSimTables.txt");
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
							tables[tableNoTrack - 1][0].back() = dataPointDouble;
						}
						// IF THIS THE FIRST LOOP, THIS IS THE COLUMN IN THE DATA SET THAT DISPLAYS THE "ROWS" VALUES
						else if (columnCount == 1) {
							// FOR TWO DIMENSIONAL TABLE
							if (dimensions[tableNoTrack -1][1] != 2){
								// PLACE DATA POINT IN ITS PLACE
								tables[tableNoTrack - 1][rowNoTrack][0] = dataPointDouble;
							}
							// FOR ONE DIMENSIONAL TABLE
							else {
								// PLACE DATA POINT IN ITS PLACE
								tables[tableNoTrack - 1][rowNoTrack - 1][0] = dataPointDouble;
							}
						}
						// IF THIS THE SECOND LOOP, THIS IS THE COLUMN IN THE DATA SET THAT DISPLAYS THE "COLUMNS" VALUES, ONLY FOR TWO DIMENSIONAL TABLES
						else if (columnCount == 2 and dimensions[tableNoTrack -1][1] != 2) {
							// PLACE DATA POINT IN ITS PLACE
							tables[tableNoTrack - 1][0][rowNoTrack] = dataPointDouble;
						}
						// ELSE FOR ACTUAL DATA POINTS
						else {
							// FOR TWO DIMENSIONAL TABLES
							if (dimensions[tableNoTrack -1][1] != 2) {
								// PLACE DATA POINT IN ITS PLACE
								tables[tableNoTrack - 1][rowNoTrack][columnCount - 2] = dataPointDouble;
							}
							// FOR ONE DIMENSIONAL TABLES
							else {
								// PLACE DATA POINT IN ITS PLACE
								tables[tableNoTrack - 1][rowNoTrack - 1][columnCount - 1] = dataPointDouble;
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
			tables.push_back(newTable);
		}
		// STORE NAME OF TABLE
		else if (flag == 3){
			// MAP TABLE NAME INDEX PAIR
			tableNameIndexPairs.emplace(name, tableNoTrack - 1);
		}
	}
}

double linearInterpolationWithBoundedEnds(string tableName, double tableInput) {
	int tableIndex = tableNameIndexPairs[tableName];
	int lowIndex = -100000;
	int highIndex = 100000;
	double interpolatedValue;
	for (int i = 0; i < tables[tableIndex].size(); i++) {
		double refValue = tables[tableIndex][i][0];
		if (refValue > tableInput) {
			if (highIndex == 100000) {
				highIndex = i;
			}
			else if (refValue < tables[tableIndex][highIndex][0]) {
				highIndex = i;
			}
		}
		else if (refValue < tableInput) {
			if (lowIndex == -100000) {
				lowIndex = i;
			}
			else if (refValue > tables[tableIndex][lowIndex][0]) {
				lowIndex = i;
			}
		}
	}
	if (lowIndex == -100000) {
		lowIndex = 0;
		interpolatedValue = tables[tableIndex][lowIndex][1];
	}
	else if (highIndex == 100000) {
		highIndex = tables[tableIndex].size() - 1;
		interpolatedValue = tables[tableIndex][highIndex][1];
	}
	else {
		interpolatedValue = tables[tableIndex][lowIndex][1] + ((tableInput - tables[tableIndex][lowIndex][0]) * ((tables[tableIndex][highIndex][1] - tables[tableIndex][lowIndex][1]) / (tables[tableIndex][highIndex][0] - tables[tableIndex][lowIndex][0])));
	}
	return interpolatedValue;
}

double biLinearInterpolationWithBoundedBorders(string tableName, double tableRowInput, double tableColumnInput) {
	int tableIndex = tableNameIndexPairs[tableName];
	int lowRowIndex = -100000;
	int highRowIndex = 100000;
	int lowColIndex = -100000;
	int highColIndex = 100000;
	int rows = tables[tableIndex].size() - 1;
	int cols = tables[tableIndex][0].size() - 1;
	double interpolatedValue;
	for (int i = 1; i <= rows; i++) {
		double refValue = tables[tableIndex][i][0];
		if (refValue >= tableRowInput) {
			if (highRowIndex == 100000) {
				highRowIndex = i;
			}
			else if (refValue < tables[tableIndex][highRowIndex][0]) {
				highRowIndex = i;
			}
		}
		else if (refValue <= tableRowInput) {
			if (lowRowIndex == -100000) {
				lowRowIndex = i;
			}
			else if (refValue > tables[tableIndex][lowRowIndex][0]) {
				lowRowIndex = i;
			}
		}
	}
	if (lowRowIndex == -100000) {
		lowRowIndex = 1;
	}
	if (highRowIndex == 100000) {
		highRowIndex = rows;
	}
	for (int i = 1; i <= cols; i++) {
		double refValue = tables[tableIndex][0][i];
		if (refValue >= tableColumnInput) {
			if (highColIndex == 100000) {
				highColIndex = i;
			}
			else if (refValue < tables[tableIndex][0][highColIndex]) {
				highColIndex = i;
			}
		}
		else if (refValue <= tableColumnInput) {
			if (lowColIndex == -100000) {
				lowColIndex = i;
			}
			else if (refValue > tables[tableIndex][0][lowColIndex]) {
				lowColIndex = i;
			}
		}
	}
	if (lowColIndex == -100000) {
		lowColIndex = 1;
	}
	if (highColIndex == 100000) {
		highColIndex = cols;
	}

	if (lowRowIndex == highRowIndex and lowColIndex != highColIndex) {
		double x = tableColumnInput;
		double x1 = tables[tableIndex][0][lowColIndex];
		double x2 = tables[tableIndex][0][highColIndex];
		double y1 = tables[tableIndex][highRowIndex][lowColIndex];
		double y2 = tables[tableIndex][highRowIndex][highColIndex];
		interpolatedValue = y1 + (x - x1) * ((y2 - y1) / (x2 - x1));
	}
	else if (lowRowIndex != highRowIndex and lowColIndex == highColIndex) {
		double x = tableRowInput;
		double x1 = tables[tableIndex][lowRowIndex][0];
		double x2 = tables[tableIndex][highRowIndex][0];
		double y1 = tables[tableIndex][lowRowIndex][highColIndex];
		double y2 = tables[tableIndex][highRowIndex][highColIndex];
		interpolatedValue = y1 + (x - x1) * ((y2 - y1) / (x2 - x1));
	}
	else if (lowRowIndex == highRowIndex and lowColIndex == highColIndex) {
		interpolatedValue = tables[tableIndex][highRowIndex][highColIndex];
	}
	else {
		double x1 = tables[tableIndex][lowRowIndex][0];
		double x2 = tables[tableIndex][highRowIndex][0];
		double y1 = tables[tableIndex][0][lowColIndex];
		double y2 = tables[tableIndex][0][highColIndex];
		double corner11 = tables[tableIndex][lowRowIndex][lowColIndex];
		double corner12 = tables[tableIndex][lowRowIndex][highColIndex];
		double corner21 = tables[tableIndex][highRowIndex][lowColIndex];
		double corner22 = tables[tableIndex][highRowIndex][highColIndex];
		double t1 = corner11 * (x2 - tableRowInput) * (y2 - tableColumnInput);
		double t2 = corner21 * (tableRowInput - x1) * (y2 - tableColumnInput);
		double t3 = corner12 * (x2 - tableRowInput) * (tableColumnInput - y1);
		double t4 = corner22 * (tableRowInput - x1) * (tableColumnInput - y1);
		interpolatedValue = (t1 + t2 + t3 + t4) / ((x2 - x1) * (y2 - y1));
	}
	return interpolatedValue;
}

void init() {

	// INITIALIZE AND OPEN INPUT FILE
	std::ifstream inPut;
	inPut.open("CPP_6DOF_SRAAM_V1/input.txt");

	// INITIALIZE INPUTS
	double phi, theta, psi, tgtE, tgtN, tgtU;

	// READ INPUTS
	inPut >> phi >> theta >> psi >> tgtE >> tgtN >> tgtU;

	// TARGET
	pip[0] = tgtE; // METERS
	pip[1] = tgtN; // METERS
	pip[2] = tgtU; // METERS

	// MISSILE
	mslAz = degToRad * psi; // DEGREES
	mslEl = degToRad * theta; // DEGREES
	eulerAnglesToLocalOrientation(degToRad * phi, -mslEl, mslAz, mslLocalOrient); // ND
	mslPos[0] = 0.0; // METERS
	mslPos[1] = 0.0; // METERS
	mslPos[2] = 0.0; // METERS
	mslVel[0] = mslLocalOrient[0][0]; // METERS PER SECOND
	mslVel[1] = mslLocalOrient[0][1]; // METERS PER SECOND
	mslVel[2] = mslLocalOrient[0][2]; // METERS PER SECOND
	threeByThreeTimesThreeByOne(mslLocalOrient, mslVel, mslBodyVel); // INITIALIZE BODY VEL >>> UVW >>> METERS PER SECOND
	mslAcc[0] = 0.0; // METERS PER SECOND^2
	mslAcc[1] = 0.0; // METERS PER SECOND^2
	mslAcc[2] = 0.0; // METERS PER SECOND^2
	threeByThreeTimesThreeByOne(mslLocalOrient, mslAcc, mslBodyAcc); // INITIALIZE BODY ACC >>> UVW_DOT >>> METERS PER SECOND^2
	magnitude(mslVel, mslSpeed); // INITIALIZE MISSILE SPEED >>> METERS PER SECOND
	mslEuler[0] = 0.0; // PHI >>> RADIANS
	mslEuler[1] = mslEl; // THETA >>> RADIANS
	mslEuler[2] = mslAz; // PSI >>> RADIANS

	// SEEKER >>> INITIALIZE BY POINTING IT DIRECTLY AT THE PIP
	double relPos[3]; // METERS
	subtractTwoVectors(mslPos, pip, relPos); // METERS
	double relPosU[3]; // ND
	unitVec(relPos, relPosU); // ND
	double mslToInterceptU[3]; // ND
	threeByThreeTimesThreeByOne(mslLocalOrient, relPosU, mslToInterceptU); // ND
	double mslToInterceptAz, mslToInterceptEl; // RADIANS
	azAndElFromVector(mslToInterceptAz, mslToInterceptEl, mslToInterceptU); // RADIANS
	seekerPitch = mslToInterceptEl; // RADIANS
	seekerYaw = mslToInterceptAz; // RADIANS
	double seekerAttitudeToLocalTM[3][3]; // ND
	eulerAnglesToLocalOrientation(0.0, -seekerPitch, seekerYaw, seekerAttitudeToLocalTM); // ND
	threeByThreeTimesThreeByThree(seekerAttitudeToLocalTM, mslLocalOrient, seekerLocalOrient); // ND
	seekerPitchErr = 0.0; // RADIANS
	seekerYawErr = 0.0; // RADIANS
	wlr = seekerYaw;
	wlq = seekerPitch;

	// DATA LOOK UP
	lookUpTablesFormat();

	// AERO DYNAMIC COEFFICIENTS
	launchCg = linearInterpolationWithBoundedEnds("CG", mslTof); // METERS

	// END CHECK
	ec.successfulIntercept = "SUCCESSFUL INTERCEPT";
	ec.flying = "IN FLIGHT";
	ec.groundCollision = "BELOW GROUND";
	ec.pointOfClosestApproachPassed = "POINT OF CLOSEST APPROACH PASSED";
	ec.maxTimeExceeded = "MAX TIME EXCEEDED";
	ec.notANumber = "NAN";
	ec.forcedSimTermination = "FORCED SIM TERMINATION";
	lethality = ec.flying; // STATUS

	// LOG DATA
	logFile.open("CPP_6DOF_SRAAM_V1/log.txt");
	logFile << fixed << setprecision(10) << "tof posE posN posU tgtE tgtN tgtU normComm normAch pitchRate thetaRate pitchDefl alpha theta sideComm sideAch yawRate psiRate yawDefl beta psi rollComm phiRate rollRate rollDefl roll seekPitchErr seekYawErr staticMargin mach" << endl;

	// FLY

	// MAIN
	cout << "\n" << endl;
	cout << "MODEL INITIATED" << endl;
	cout << "\n" << endl;

}

void timeOfFlight() {
	mslTof += timeStep;
}

void environment() {
	double alt = mslPos[2] * mToKm; // KILOMETERS
	double rho = linearInterpolationWithBoundedEnds("RHO", alt); // KILOGRAMS PER METER^3
	grav = linearInterpolationWithBoundedEnds("GRAVITY", alt); // METERS PER S^2
	double gravLocalVec[3] = {0.0, 0.0, -grav}; // METERS PER S^2
	threeByThreeTimesThreeByOne(mslLocalOrient, gravLocalVec, gravBodyVec); // METERS PER S^2
	press = linearInterpolationWithBoundedEnds("PRESSURE", alt); // PASCALS
	double a = linearInterpolationWithBoundedEnds("SPEED_OF_SOUND", alt); // METERS PER S^2
	magnitude(mslVel, mslSpeed); // METERS PER SECOND
	mslMach = mslSpeed / a; // ND
	q = 0.5 * rho * mslSpeed * mslSpeed; // PASCALS
}

void seeker() {

	double wsq = wnk * wnk;
	double gg = gk * wsq;

	// YAW CHANNEL
	double wlr1d_new = wlr2;
	double wlr1_new = integrate(wlr1d_new, wlr1d, wlr1, integrationStep);
	wlr1 = wlr1_new;
	wlr1d = wlr1d_new;
	double wlr2d_new = gg * seekerYawErr - 2 * zetak * wnk * wlr1d - wsq * wlr1;
	double wlr2_new = integrate(wlr2d_new, wlr2d, wlr2, integrationStep);
	wlr2 = wlr2_new;
	wlr2d = wlr2d_new;

	// PITCH CHANNEL
	double wlq1d_new = wlq2;
	double wlq1_new = integrate(wlq1d_new, wlq1d, wlq1, integrationStep);
	wlq1 = wlq1_new;
	wlq1d = wlq1d_new;
	double wlq2d_new = gg * seekerPitchErr - 2 * zetak * wnk * wlq1d - wsq * wlq1;
	double wlq2_new = integrate(wlq2d_new, wlq2d, wlq2, integrationStep);
	wlq2 = wlq2_new;
	wlq2d = wlq2d_new;

	// YAW CONTROL
	double wlrd_new = wlr1 - mslRate[2];
	double wlr_new = integrate(wlrd_new, wlrd, wlr, integrationStep);
	wlr = wlr_new;
	wlrd = wlrd_new;
	seekerYaw = wlr;

	// PITCH CONTROL
	double wlqd_new = wlq1 - mslRate[1];
	double wlq_new = integrate(wlqd_new, wlqd, wlq, integrationStep);
	wlq = wlq_new;
	wlqd = wlqd_new;
	seekerPitch = wlq;

	double localRelPos[3];
	subtractTwoVectors(mslPos, pip, localRelPos);
	double seekerAttitudeToLocalTM[3][3];
	eulerAnglesToLocalOrientation(0.0, -seekerPitch, seekerYaw, seekerAttitudeToLocalTM);
	threeByThreeTimesThreeByThree(seekerAttitudeToLocalTM, mslLocalOrient, seekerLocalOrient);
	double seekerToInterceptRelPos[3];
	threeByThreeTimesThreeByOne(seekerLocalOrient, localRelPos, seekerToInterceptRelPos);
	double inducedErr[3] = {1.0, 0.5, 0.2};
	double seekerToInterceptRelPosWithErr[3];
	multiplyTwoVectors(seekerToInterceptRelPos, inducedErr, seekerToInterceptRelPosWithErr);
	azAndElFromVector(seekerYawErr, seekerPitchErr, seekerToInterceptRelPosWithErr);
	oneByThreeTimesThreeByThree(seekerToInterceptRelPosWithErr, seekerAttitudeToLocalTM, forwardLeftUpMslToInterceptPos);

}

void guidance() {
	double forwardLeftUpMslToInterceptPosU[3];
	double forwardLeftUpMslToInterceptMag;
	unitVec(forwardLeftUpMslToInterceptPos, forwardLeftUpMslToInterceptPosU);
	magnitude(forwardLeftUpMslToInterceptPos, forwardLeftUpMslToInterceptMag);
	double relVel[3];
	relVel[0] = mslVel[0] * -1;
	relVel[1] = mslVel[1] * -1;
	relVel[2] = mslVel[2] * -1;
	double relVelMag;
	magnitude(relVel, relVelMag);
	double closingVel[3];
	threeByThreeTimesThreeByOne(mslLocalOrient, relVel, closingVel);
	double TEMP1[3], TEMP2;
	crossProductTwoVectors(forwardLeftUpMslToInterceptPos, closingVel, TEMP1);
	dotProductTwoVectors(forwardLeftUpMslToInterceptPos, forwardLeftUpMslToInterceptPos, TEMP2);
	double lineOfSightRate[3];
	lineOfSightRate[0] = TEMP1[0] / TEMP2;
	lineOfSightRate[1] = TEMP1[1] / TEMP2;
	lineOfSightRate[2] = TEMP1[2] / TEMP2;
	double command[3];
	double TEMP3[3];
	TEMP3[0] = -1 * proNavGain * relVelMag * forwardLeftUpMslToInterceptPosU[0];
	TEMP3[1] = -1 * proNavGain * relVelMag * forwardLeftUpMslToInterceptPosU[1];
	TEMP3[2] = -1 * proNavGain * relVelMag * forwardLeftUpMslToInterceptPosU[2];
	crossProductTwoVectors(TEMP3, lineOfSightRate, command);
	normCommand = command[2];
	sideCommand = command[1];
	double trigRatio = atan2(normCommand, sideCommand);
	double accMag = sqrt(normCommand * normCommand + sideCommand * sideCommand);
	if (accMag > maxAccel) {
		accMag = maxAccel;
	}
	sideCommand = accMag * cos(trigRatio);
	normCommand = accMag * sin(trigRatio);
}

void control() {
	if (mslMach > 0.6) {
		
		double DNA = CNA * (q * mslRefArea / mass); // METERS PER SECOND^2
		double DMA = CMA * (q * mslRefArea * mslRefDiam / transverseMomentOfInertia); // PER SECOND^2
		double DMD = CMD * (q * mslRefArea * mslRefDiam / transverseMomentOfInertia); // PER SECOND^2
		double DMQ = CMQ * (mslRefDiam / (2 * mslSpeed)) * (q * mslRefArea * mslRefDiam / transverseMomentOfInertia); // PER SECOND
		double DLP = CLP * (mslRefDiam / (2 * mslSpeed)) * (q * mslRefArea * mslRefDiam / axialMomentOfInertia); // PER SECOND
		double DLD = CLD * (q * mslRefArea * mslRefDiam / axialMomentOfInertia); // PER SECOND^2

		double WACL = 0.013 * sqrt(q) + 7.1;
		double ZACL = 0.000559 * sqrt(q) + 0.232;
		double PACL = 14;

		// FEEDBACK GAINS
		double GAINFB3 = WACL * WACL * PACL / (DNA * DMD);
		double GAINFB2 = (2 * ZACL * WACL + PACL + DMQ - DNA / mslSpeed) / DMD;
		double GAINFB1 = (
			WACL * WACL + 2 * ZACL * WACL * PACL + DMA + DMQ * DNA / mslSpeed - GAINFB2 * DMD * DNA / mslSpeed
		) / (DNA * DMD);

		// ROLL
		double GKP = (2 * wrcl * zrcl + DLP) / DLD;
		double GKPHI = wrcl * wrcl / DLD;
		double EPHI = GKPHI * (rollAngleComm - mslEuler[0]);
		rollFinComm = EPHI - GKP * mslRate[0];

		// PITCH
		double zzdNew = normCommand - mslBodyAcc[2];
		double zzNew = integrate(zzdNew, zzd, zz, integrationStep);
		zz = zzNew;
		zzd = zzdNew;
		double deflPitch = -1 * GAINFB1 * mslBodyAcc[2] - GAINFB2 * mslRate[1] + GAINFB3 * zz;
		if (abs(deflPitch) > maxDefl) {
			if (deflPitch > 0) {
				deflPitch = maxDefl;
			}
			else if (deflPitch < 0) {
				deflPitch = -1 * maxDefl;
			}
		}
		pitchFinComm = deflPitch * degToRad;

		// YAW
		double yydNew = mslBodyAcc[1] - sideCommand;
		double yyNew = integrate(yydNew, yyd, yy, integrationStep);
		yy = yyNew;
		yyd = yydNew;
		double deflYaw = GAINFB1 * mslBodyAcc[1] - GAINFB2 * mslRate[2] + GAINFB3 * yy;
		if (abs(deflYaw) > maxDefl) {
			if (deflYaw > 0) {
				deflYaw = maxDefl;
			}
			else if (deflYaw < 0) {
				deflYaw = -1 * maxDefl;
			}
		}
		yawFinComm = deflYaw * degToRad;

	}
	else if (mslMach > 0.01) {

		double DNA = CNA * (q * mslRefArea / mass); // METERS PER SECOND^2
		double DND = CND * (q * mslRefArea / mass); // METERS PER SECOND^2
		double DMA = CMA * (q * mslRefArea * mslRefDiam / transverseMomentOfInertia); // PER SECOND^2
		double DMD = CMD * (q * mslRefArea * mslRefDiam / transverseMomentOfInertia); // PER SECOND^2
		double DMQ = CMQ * (mslRefDiam / (2 * mslSpeed)) * (q * mslRefArea * mslRefDiam / transverseMomentOfInertia); // PER SECOND
		double DLP = CLP * (mslRefDiam / (2 * mslSpeed)) * (q * mslRefArea * mslRefDiam / axialMomentOfInertia); // PER SECOND
		double DLD = CLD * (q * mslRefArea * mslRefDiam / axialMomentOfInertia); // PER SECOND^2

		// ROLL
		double GKP = (2 * wrcl * zrcl + DLP) / DLD;
		double GKPHI = wrcl * wrcl / DLD;
		double EPHI = GKPHI * (rollAngleComm - mslEuler[0]);
		rollFinComm = EPHI - GKP * mslRate[0];

		// RATE CONTROL
		double ZRATE = DNA / mslSpeed - DMA * DND / (mslSpeed * DMD); // ND
		double AA = DNA / mslSpeed - DMQ; // ND
		double BB = -1 * DMA - DMQ * DNA / mslSpeed; // ND
		double TEMP1 = AA - 2 * zetlagr * zetlagr * ZRATE; // ND
		double TEMP2 = AA * AA - 4 * zetlagr * zetlagr * BB; // ND
		double RADIX = TEMP1 * TEMP1 - TEMP2; // ND
		double GRATE = (-1 * TEMP1 + sqrt(RADIX)) / (-1 * DMD); // ND

		// PITCH
		pitchFinComm = GRATE * mslRate[1]; // RADIANS

		// YAW
		yawFinComm = GRATE * mslRate[2]; // RADIANS


	}
	else {
		rollFinComm = 0.0;
		pitchFinComm = 0.0;
		yawFinComm = 0.0;
	}
}

void actuators() {

	double DEL1C = -rollFinComm + pitchFinComm - yawFinComm;
	double DEL2C = -rollFinComm + pitchFinComm + yawFinComm;
	double DEL3C = rollFinComm + pitchFinComm - yawFinComm;
	double DEL4C = rollFinComm + pitchFinComm + yawFinComm;
	int flag;

	// FIN ONE
	flag = 0;
	if (abs(DEL1) > finRadianLimit) {
		if (DEL1 < 0) {
			DEL1 = -1 * finRadianLimit;
		}
		else if (DEL1 > 0) {
			DEL1 = finRadianLimit;
		}
		if ((DEL1 * DEL1DOT) > 0) {
			DEL1DOT = 0;
		}
	}
	if (abs(DEL1DOT) > finRateRadianLimit) {
		flag = 1;
		if (DEL1DOT < 0) {
			DEL1DOT = -1 * finRateRadianLimit;
		}
		else if (DEL1DOT > 0) {
			DEL1DOT = finRateRadianLimit;
		}
	}
	double DEL1D_NEW = DEL1DOT;
	double DEL1_NEW = integrate(DEL1D_NEW, DEL1D, DEL1, integrationStep);
	DEL1 = DEL1_NEW;
	DEL1D = DEL1D_NEW;
	double EDX1 = DEL1C - DEL1;
	double DEL1DOTDOT_NEW = WNACT * WNACT * EDX1 - 2 * ZETACT * WNACT * DEL1D;
	double DEL1DOT_NEW = integrate(DEL1DOTDOT_NEW, DEL1DOTDOT, DEL1DOT, integrationStep);
	DEL1DOT = DEL1DOT_NEW;
	DEL1DOTDOT = DEL1DOTDOT_NEW;
	if (flag == 1 and (DEL1DOT * DEL1DOTDOT) > 0) {
		DEL1DOTDOT = 0.0;
	}

	// FIN TWO
	flag = 0;
	if (abs(DEL2) > finRadianLimit) {
		if (DEL2 < 0) {
			DEL2 = -1 * finRadianLimit;
		}
		else if (DEL2 > 0) {
			DEL2 = finRadianLimit;
		}
		if ((DEL2 * DEL2DOT) > 0) {
			DEL2DOT = 0;
		}
	}
	if (abs(DEL2DOT) > finRateRadianLimit) {
		flag = 1;
		if (DEL2DOT < 0) {
			DEL2DOT = -1 * finRateRadianLimit;
		}
		else if (DEL2DOT > 0) {
			DEL2DOT = finRateRadianLimit;
		}
	}
	double DEL2D_NEW = DEL2DOT;
	double DEL2_NEW = integrate(DEL2D_NEW, DEL2D, DEL2, integrationStep);
	DEL2 = DEL2_NEW;
	DEL2D = DEL2D_NEW;
	double EDX2 = DEL2C - DEL2;
	double DEL2DOTDOT_NEW = WNACT * WNACT * EDX2 - 2 * ZETACT * WNACT * DEL2D;
	double DEL2DOT_NEW = integrate(DEL2DOTDOT_NEW, DEL2DOTDOT, DEL2DOT, integrationStep);
	DEL2DOT = DEL2DOT_NEW;
	DEL2DOTDOT = DEL2DOTDOT_NEW;
	if (flag == 1 and (DEL2DOT * DEL2DOTDOT) > 0) {
		DEL2DOTDOT = 0.0;
	}

	// FIN THREE
	flag = 0;
	if (abs(DEL3) > finRadianLimit) {
		if (DEL3 < 0) {
			DEL3 = -1 * finRadianLimit;
		}
		else if (DEL3 > 0) {
			DEL3 = finRadianLimit;
		}
		if ((DEL3 * DEL3DOT) > 0) {
			DEL3DOT = 0;
		}
	}
	if (abs(DEL3DOT) > finRateRadianLimit) {
		flag = 1;
		if (DEL3DOT < 0) {
			DEL3DOT = -1 * finRateRadianLimit;
		}
		else if (DEL3DOT > 0) {
			DEL3DOT = finRateRadianLimit;
		}
	}
	double DEL3D_NEW = DEL3DOT;
	double DEL3_NEW = integrate(DEL3D_NEW, DEL3D, DEL3, integrationStep);
	DEL3 = DEL3_NEW;
	DEL3D = DEL3D_NEW;
	double EDX3 = DEL3C - DEL3;
	double DEL3DOTDOT_NEW = WNACT * WNACT * EDX3 - 2 * ZETACT * WNACT * DEL3D;
	double DEL3DOT_NEW = integrate(DEL3DOTDOT_NEW, DEL3DOTDOT, DEL3DOT, integrationStep);
	DEL3DOT = DEL3DOT_NEW;
	DEL3DOTDOT = DEL3DOTDOT_NEW;
	if (flag == 1 and (DEL3DOT * DEL3DOTDOT) > 0) {
		DEL3DOTDOT = 0.0;
	}

	// FIN FOUR
	flag = 0;
	if (abs(DEL4) > finRadianLimit) {
		if (DEL4 < 0) {
			DEL4 = -1 * finRadianLimit;
		}
		else if (DEL4 > 0) {
			DEL4 = finRadianLimit;
		}
		if ((DEL4 * DEL4DOT) > 0) {
			DEL4DOT = 0;
		}
	}
	if (abs(DEL4DOT) > finRateRadianLimit) {
		flag = 1;
		if (DEL4DOT < 0) {
			DEL4DOT = -1 * finRateRadianLimit;
		}
		else if (DEL4DOT > 0) {
			DEL4DOT = finRateRadianLimit;
		}
	}
	double DEL4D_NEW = DEL4DOT;
	double DEL4_NEW = integrate(DEL4D_NEW, DEL4D, DEL4, integrationStep);
	DEL4 = DEL4_NEW;
	DEL4D = DEL4D_NEW;
	double EDX4 = DEL4C - DEL4;
	double DEL4DOTDOT_NEW = WNACT * WNACT * EDX4 - 2 * ZETACT * WNACT * DEL4D;
	double DEL4DOT_NEW = integrate(DEL4DOTDOT_NEW, DEL4DOTDOT, DEL4DOT, integrationStep);
	DEL4DOT = DEL4DOT_NEW;
	DEL4DOTDOT = DEL4DOTDOT_NEW;
	if (flag == 1 and (DEL4DOT * DEL4DOTDOT) > 0) {
		DEL4DOTDOT = 0.0;
	}

	rollFinDefl = (-DEL1 - DEL2 + DEL3 + DEL4) / 4;
	pitchFinDefl = (DEL1 + DEL2 + DEL3 + DEL4) / 4;
	yawFinDefl = (-DEL1 + DEL2 - DEL3 + DEL4) / 4;

}

void aeroBallisticAngles() {

	double alphaPrime = acos(cos(mslAlpha) * cos(mslBeta));
	alphaPrimeDeg = radToDeg * alphaPrime;
	double phiPrime = atan2(tan(mslBeta), sin(mslAlpha));
	sinPhiPrime = sin(phiPrime);
	cosPhiPrime = cos(phiPrime);
	double pitchDeflAeroFrame = pitchFinDefl * cosPhiPrime - yawFinDefl * sinPhiPrime;
	pitchDeflAeroDeg = radToDeg * pitchDeflAeroFrame;
	double yawDeflAeroFrame = pitchFinDefl * sinPhiPrime + yawFinDefl * cosPhiPrime;
	yawDeflAeroDeg = radToDeg * yawDeflAeroFrame;
	rollDeflDeg = radToDeg * rollFinDefl;
	totalFinDeflDeg = (abs(pitchDeflAeroDeg) + abs(yawDeflAeroDeg)) / 2;
	double pitchRateAeroFrame = mslRate[1] * cosPhiPrime - mslRate[2] * sinPhiPrime;
	pitchRateAeroDeg = radToDeg * pitchRateAeroFrame;
	double yawRateAeroFrame = mslRate[1] * sinPhiPrime + mslRate[2] * cosPhiPrime;
	yawRateAeroDeg = radToDeg * yawRateAeroFrame;
	rollRateDeg = radToDeg * mslRate[0];
	sinOfFourTimesPhiPrime = sin(4 * phiPrime);
	squaredSinOfTwoTimesPhiPrime = pow((sin(2 * phiPrime)), 2);
}

void dataLookUp() {
	CA0 = linearInterpolationWithBoundedEnds("CA0", mslMach);
	CAA = linearInterpolationWithBoundedEnds("CAA", mslMach);
	CAD = linearInterpolationWithBoundedEnds("CAD", mslMach);
	if (mslTof <= mslBurnOut) {
		CAOFF = 0.0;
	}
	else {
		CAOFF = linearInterpolationWithBoundedEnds("CAOFF", mslMach);
	}
	CYP = biLinearInterpolationWithBoundedBorders("CYP", mslMach, alphaPrimeDeg);
	CYDR = biLinearInterpolationWithBoundedBorders("CYDR", mslMach, alphaPrimeDeg);
	CN0 = biLinearInterpolationWithBoundedBorders("CN0", mslMach, alphaPrimeDeg);
	CNP = biLinearInterpolationWithBoundedBorders("CNP", mslMach, alphaPrimeDeg);
	CNDQ = biLinearInterpolationWithBoundedBorders("CNDQ", mslMach, alphaPrimeDeg);
	CLLAP = biLinearInterpolationWithBoundedBorders("CLLAP", mslMach, alphaPrimeDeg);
	CLLP = biLinearInterpolationWithBoundedBorders("CLLP", mslMach, alphaPrimeDeg);
	CLLDP = biLinearInterpolationWithBoundedBorders("CLLDP", mslMach, alphaPrimeDeg);
	CLM0 = biLinearInterpolationWithBoundedBorders("CLM0", mslMach, alphaPrimeDeg);
	CLMP = biLinearInterpolationWithBoundedBorders("CLMP", mslMach, alphaPrimeDeg);
	CLMQ = linearInterpolationWithBoundedEnds("CLMQ", mslMach);
	CLMDQ = biLinearInterpolationWithBoundedBorders("CLMDQ", mslMach, alphaPrimeDeg);
	CLNP = biLinearInterpolationWithBoundedBorders("CLNP", mslMach, alphaPrimeDeg);
	mass = linearInterpolationWithBoundedEnds("MASS", mslTof);
	unAdjThrust = linearInterpolationWithBoundedEnds("THRUST", mslTof);
	transverseMomentOfInertia = linearInterpolationWithBoundedEnds("TMOI", mslTof);
	axialMomentOfInertia = linearInterpolationWithBoundedEnds("AMOI", mslTof);
	cgFromNose = linearInterpolationWithBoundedEnds("CG", mslTof);

	double currentAccel = CN0 * q * mslRefArea / mass;
	double CN0MAX = biLinearInterpolationWithBoundedBorders("CN0", mslMach, alphaPrimeMax);
	double MAXACC = CN0MAX * q * mslRefArea / mass;
	double availAccel = MAXACC - currentAccel;
	if (availAccel < 0) {
		maxAccel = 1;
	}
	else if (availAccel > maxAccelAllow) {
		maxAccel = maxAccelAllow;
	}
	else {
		maxAccel = availAccel;
	}

}

void propulsion() {
	if (mslTof >= mslBurnOut) {
		thrust = 0.0;
	}
	else {
		thrust = unAdjThrust + (seaLevelPress - press) * mslExitArea;
	}
}

void aeroDynamicCoefficients() {
	CX = CA0 + CAA * alphaPrimeDeg + CAD * (totalFinDeflDeg * totalFinDeflDeg) + CAOFF;
	double CYAERO = CYP * sinOfFourTimesPhiPrime + CYDR * yawDeflAeroDeg;
	double CZAERO = CN0 + CNP * squaredSinOfTwoTimesPhiPrime + CNDQ * pitchDeflAeroDeg;
	CL = CLLAP * pow(alphaPrimeDeg, 2) * sinOfFourTimesPhiPrime + CLLP * rollRateDeg * mslRefDiam / (2 * mslSpeed) + CLLDP * rollDeflDeg;
	double CNAEROREF = CLNP * sinOfFourTimesPhiPrime + CLMQ * yawRateAeroDeg * mslRefDiam / (2 * mslSpeed) + CLMDQ * yawDeflAeroDeg;
	double CNAERO = CNAEROREF - CYAERO * (launchCg - cgFromNose) / mslRefDiam;
	double CMAEROREF = CLM0 + CLMP * squaredSinOfTwoTimesPhiPrime + CLMQ * pitchRateAeroDeg * mslRefDiam / (2 * mslSpeed) + CLMDQ * pitchDeflAeroDeg;
	double CMAERO = CMAEROREF - CZAERO * (launchCg - cgFromNose) / mslRefDiam;
	CY = CYAERO * cosPhiPrime - CZAERO * sinPhiPrime;
	CZ = CYAERO * sinPhiPrime + CZAERO * cosPhiPrime;
	CN = CMAERO * sinPhiPrime + CNAERO * cosPhiPrime;
	CM = CMAERO * cosPhiPrime + CNAERO * sinPhiPrime;
}

void aeroDynamicRefValues() {

	double alphaPrimeDegLookUp;
	if (alphaPrimeDeg > (alphaPrimeMax - 3)) {
		alphaPrimeDegLookUp = alphaPrimeMax - 3;
	}
	else {
		alphaPrimeDegLookUp = alphaPrimeDeg;
	}
	double alphaPrimeDegMinusThree = alphaPrimeDegLookUp - 3;
	double alphaPrimeDegPlusThree = alphaPrimeDegLookUp + 3;
	double CN0MIN = biLinearInterpolationWithBoundedBorders("CN0", mslMach, alphaPrimeDegMinusThree);
	double CN0MAX = biLinearInterpolationWithBoundedBorders("CN0", mslMach, alphaPrimeDegPlusThree);
	CNA = ((CN0MAX - CN0MIN) / (alphaPrimeDegPlusThree - alphaPrimeDegMinusThree)) * radToDeg;
	double CLM0MIN = biLinearInterpolationWithBoundedBorders("CLM0", mslMach, alphaPrimeDegMinusThree);
	double CLM0MAX = biLinearInterpolationWithBoundedBorders("CLM0", mslMach, alphaPrimeDegPlusThree);
	CMA = ((CLM0MAX - CLM0MIN) / (alphaPrimeDegPlusThree - alphaPrimeDegMinusThree) - (CNA / radToDeg) * (launchCg - cgFromNose) / mslRefDiam) * radToDeg;
	CND = CNDQ * radToDeg;
	CMD = CLMDQ * radToDeg;
	CMQ = CLMQ * radToDeg;
	CLP = CLLP * radToDeg;
	CLD = CLLDP * radToDeg;
	staticMargin = -1 * (CMA * degToRad) / (CNA * degToRad);
}

void translationalAcceleration() {

	double axialForce = thrust - CX * q * mslRefArea + gravBodyVec[0] * mass;
	double sideForce = CY * q * mslRefArea + gravBodyVec[1] * mass;
	double normalForce = CZ * q * mslRefArea + gravBodyVec[2] * mass;

	mslBodyAcc[0] = axialForce / mass - (mslRate[1] * mslBodyVel[2] - mslRate[2] * mslBodyVel[1]);
	mslBodyAcc[1] = sideForce / mass - (mslRate[2] * mslBodyVel[0] - mslRate[0] * mslBodyVel[2]);
	mslBodyAcc[2] = normalForce / mass - (mslRate[0] * mslBodyVel[1] - mslRate[1] * mslBodyVel[0]);

	oneByThreeTimesThreeByThree(mslBodyAcc, mslLocalOrient, mslAcc);

}

void rotationalAcceleration() {

	double rollMoment = CL * q * mslRefArea * mslRefDiam;
	double pitchMoment = CM * q * mslRefArea * mslRefDiam;
	double yawMoment = CN * q * mslRefArea * mslRefDiam;

	double newRollRateDot = rollMoment / axialMomentOfInertia;
	double newRollRate = integrate(newRollRateDot, mslRateDot[0], mslRate[0], integrationStep);
	mslRateDot[0] = newRollRateDot;

	double newPitchRateDot = (1 / transverseMomentOfInertia) * ((transverseMomentOfInertia - axialMomentOfInertia) * mslRate[0] * mslRate[2] + pitchMoment);
	double newPitchRate = integrate(newPitchRateDot, mslRateDot[1], mslRate[1], integrationStep);
	mslRateDot[1] = newPitchRateDot;

	double newYawRateDot = (1 / transverseMomentOfInertia) * ((axialMomentOfInertia - transverseMomentOfInertia) * mslRate[0] * mslRate[1] + yawMoment);
	double newYawRate = integrate(newYawRateDot, mslRateDot[2], mslRate[2], integrationStep);
	mslRateDot[2] = newYawRateDot;

	mslRate[0] = newRollRate;
	mslRate[1] = newPitchRate;
	mslRate[2] = newYawRate;

}

void translationalIntegration() {

	double deltaVel[3];
	multiplyVectorTimesScalar(timeStep, mslAcc, deltaVel);
	double newMslVel[3];
	addTwoVectors(mslVel, deltaVel, newMslVel);
	mslVel[0] = newMslVel[0];
	mslVel[1] = newMslVel[1];
	mslVel[2] = newMslVel[2];
	double deltaPos[3];
	multiplyVectorTimesScalar(timeStep, mslVel, deltaPos);
	double newMslPos[3];
	addTwoVectors(mslPos, deltaPos, newMslPos);
	mslPos[0] = newMslPos[0];
	mslPos[1] = newMslPos[1];
	mslPos[2] = newMslPos[2];
	double distanceTravelled;
	magnitude(deltaPos, distanceTravelled);
	mslRange += distanceTravelled;

}

void rotationalIntegration() {

	double newPhiDot = mslRate[0] + (mslRate[1] * sin(mslEuler[0]) + mslRate[2] * cos(mslEuler[0])) * tan(mslEuler[1]);
	double newPhi = integrate(newPhiDot, mslEulerDot[0], mslEuler[0], integrationStep);
	mslEulerDot[0] = newPhiDot;

	double newThetaDot = mslRate[1] * cos(mslEuler[0]) - mslRate[2] * sin(mslEuler[0]);
	double newTheta = integrate(newThetaDot, mslEulerDot[1], mslEuler[1], integrationStep);
	mslEulerDot[1] = newThetaDot;

	double newPsiDot = -1 * (mslRate[1] * sin(mslEuler[0]) + mslRate[2] * cos(mslEuler[0])) / cos(mslEuler[1]);
	double newPsi = integrate(newPsiDot, mslEulerDot[2], mslEuler[2], integrationStep);
	mslEulerDot[2] = newPsiDot;

	mslEuler[0] = newPhi;
	mslEuler[1] = newTheta;
	mslEuler[2] = newPsi;

	eulerAnglesToLocalOrientation(newPhi, -newTheta, newPsi, mslLocalOrient);

}

void aeroDynamicAngles() {

	threeByThreeTimesThreeByOne(mslLocalOrient, mslVel, mslBodyVel);
	magnitude(mslBodyVel, mslSpeed);
	mslAlpha = -1 * atan2(mslBodyVel[2], mslBodyVel[0]);
	mslBeta = atan2(mslBodyVel[1], mslBodyVel[0]);
}

void intercept() {
	magnitude(forwardLeftUpMslToInterceptPos, missDistance);
}

void endCheck() {
	if (mslPos[2] < 0) {
		lethality = ec.groundCollision;
		go = false;
	}
	else if (missDistance < 2.0) {
		lethality = ec.successfulIntercept;
		go = false;
	}
	else if (forwardLeftUpMslToInterceptPos[0] < 0.0) {
		lethality = ec.pointOfClosestApproachPassed;
		go = false;
	}
	else if (isnan(mslPos[0])) {
		lethality = ec.notANumber;
		go = false;
	}
	else if (mslTof > maxTime) {
		lethality = ec.maxTimeExceeded;
		go = false;
	}
	else if (lethality == ec.forcedSimTermination) {
		go = false;
	}
}

void logData() {
	logFile << fixed << setprecision(10) << mslTof << " " << mslPos[0] << " " << mslPos[1] << " " << mslPos[2] << " " << pip[0] << " " << pip[1] << " " << pip[2] << " " << normCommand / grav << " " << mslBodyAcc[2] / grav << " " << mslRate[1] * radToDeg << " " << mslEulerDot[1] * radToDeg << " " << pitchFinComm * radToDeg << " " << mslAlpha * radToDeg << " " << mslEuler[1] * radToDeg << " " << sideCommand / grav << " " << mslBodyAcc[1] / grav << " " << mslRate[2] * radToDeg << " " << mslEulerDot[2] * radToDeg << " " << yawFinComm * radToDeg << " " << mslBeta * radToDeg << " " << mslEuler[2] * radToDeg << " " << rollAngleComm << " " << mslEulerDot[0] * radToDeg << " " << mslRate[0] * radToDeg << " " << rollDeflDeg << " " << mslEuler[0] * radToDeg << " " <<  seekerPitchErr << " " << seekerYawErr << " " << staticMargin << " " << mslMach << "\n";
}

void fly() {
	timeOfFlight();
	environment();
	seeker();
	guidance();
	control();
	actuators();
	aeroBallisticAngles();
	dataLookUp();
	propulsion();
	aeroDynamicCoefficients();
	aeroDynamicRefValues();
	translationalAcceleration();
	rotationalAcceleration();
	translationalIntegration();
	rotationalIntegration();
	aeroDynamicAngles();
	intercept();
	endCheck();
	logData();
}

int main () {
	init();
	double lastTime = 0;
	cout << "FLIGHT" << endl;
	while (go) {
		fly();
		auto print_it = static_cast<int>(round(mslTof * 10000.0)) % 10000;
		if (print_it == 0)
		{
			cout << setprecision(6) << mslTof << " E " << mslPos[0] << " N " << mslPos[1] << " U " << mslPos[2] << " RANGE " << mslRange << " MACH " << mslMach << endl;
			lastTime = mslTof;
		}
	}
	cout << "\n" << endl;
	cout << "MISSION REPORT" << endl;
	cout << setprecision(6) << "FINAL POSITION AT " << mslTof << " E " << mslPos[0] << " N " << mslPos[1] << " U " << mslPos[2] << " RANGE " << mslRange << " MACH " << mslMach << endl;
	cout << setprecision(6) << "MISS DISTANCE " << missDistance << " FORWARD, LEFT, UP, MISS DISTANCE " << forwardLeftUpMslToInterceptPos[0] << " " << forwardLeftUpMslToInterceptPos[1] << " " << forwardLeftUpMslToInterceptPos[2] << endl;
	logFile << "\n";
	logFile << setprecision(6) << "MISS DISTANCE " << missDistance << " FORWARD, LEFT, UP, MISS DISTANCE " << forwardLeftUpMslToInterceptPos[0] << " " << forwardLeftUpMslToInterceptPos[1] << " " << forwardLeftUpMslToInterceptPos[2] << endl;
	cout << "SIMULATION RESULT: " << lethality << endl;
	auto wallClockEnd = chrono::high_resolution_clock::now();
	auto simRealRunTime = chrono::duration_cast<chrono::milliseconds>(wallClockEnd - wallClockStart);
	cout << "SIMULATION RUN TIME :" << simRealRunTime.count() << " MILLISECONDS" << endl;
	cout << "\n" << endl;
	return 0;
}