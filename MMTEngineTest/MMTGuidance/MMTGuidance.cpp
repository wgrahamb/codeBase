// Standard library.
#include "iostream"
#include "iomanip"
#include "fstream"
#include "math.h"
#include "algorithm"
#include "map"
#include "random"
#include "chrono"
#include "vector"
#include "map"

// Local utility.
#include "../util.h"

// Third party.
#include "eigen-3.4.0/eigen-3.4.0/Eigen/Core"
#include "eigen-3.4.0/eigen-3.4.0/Eigen/Dense"

// Header.
#include "MMTGuidance.h"

// Namespace.
using namespace std;

MMTGuidance::MMTGuidance()
{
	cout << "GUIDANCE CONSTRUCTED" << endl;
}

void MMTGuidance::lookUpTablesFormat (string inPutFile)
{
	// LOOK UP DATA
	ifstream inFile(inPutFile);
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
			name = line.substr(5, line.size() - 5);
			// cout << name << endl;
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
			int D1 = stoi(line.substr(4, 4));
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

void MMTGuidance::init()
{

	timeStep = 1 / 200.0;
	flag = 0;
	timeOfGuidanceStart = 0.6;
	timeOfFinLockOff = 0.2;
	rangeBetweenLastPositionAndCurrentLimit = 4.0;
	gammaDotLimit = 3.0;
	yawGammaDesiredDegrees = 15.0; // Positive means incoming from the right.
	pitchGammaDesiredDegrees = -15.0; // Positive means incoming from below.
	yawGammaCommand = 0.0;
	pitchGammaCommand = 0.0;
	velTan_d[0] = cos(pitchGammaDesiredDegrees * degToRad) * cos(yawGammaDesiredDegrees * degToRad);
	velTan_d[1] = cos(pitchGammaDesiredDegrees * degToRad) * sin(yawGammaDesiredDegrees * degToRad);
	velTan_d[2] = -1 * sin(pitchGammaDesiredDegrees * degToRad);
	lastLocalPostion = {0.0, 0.0, 0.0};
	currentLocalPostion = {0.0, 0.0, 0.0};
	lastLocalVelocity = {0.0, 0.0, 0.0};
	currentLocalVelocity = {0.0, 0.0, 0.0};
	XKN = 0.0;
	XKNGainFlag = -1;
	timeOfKalmanFilterInit = 0.0;
	flightPathAngleTolerance = 3.0;

}

void MMTGuidance::update(
	double timeOfFlight,
	double localWayPoint[3],
	double localPosition[3],
	double localVelocity[3],
	double rollAngle,
	double localOrientation[3][3],
	double rolledToNonRolledMatrix[3][3],
	double alphaDegrees,
	double betaDegrees
)
{

	wayPoint = {localWayPoint[0], localWayPoint[1], localWayPoint[2]};
	lastLocalPostion = currentLocalPostion;
	lastLocalVelocity = currentLocalVelocity;
	currentLocalPostion = {localPosition[0], localPosition[1], localPosition[2]};
	currentLocalVelocity = {localVelocity[0], localVelocity[1], localVelocity[2]};
	missileSpeed = currentLocalVelocity.norm();

	rolledToNonRolledBodyMatrix(0, 0) = rolledToNonRolledMatrix[0][0];
	rolledToNonRolledBodyMatrix(0, 1) = rolledToNonRolledMatrix[0][1];
	rolledToNonRolledBodyMatrix(0, 2) = rolledToNonRolledMatrix[0][2];
	rolledToNonRolledBodyMatrix(1, 0) = rolledToNonRolledMatrix[1][0];
	rolledToNonRolledBodyMatrix(1, 1) = rolledToNonRolledMatrix[1][1];
	rolledToNonRolledBodyMatrix(1, 2) = rolledToNonRolledMatrix[1][2];
	rolledToNonRolledBodyMatrix(2, 0) = rolledToNonRolledMatrix[2][0];
	rolledToNonRolledBodyMatrix(2, 1) = rolledToNonRolledMatrix[2][1];
	rolledToNonRolledBodyMatrix(2, 2) = rolledToNonRolledMatrix[2][2];

	missileLocalOrientation(0, 0) = localOrientation[0][0];
	missileLocalOrientation(0, 1) = localOrientation[0][1];
	missileLocalOrientation(0, 2) = localOrientation[0][2];
	missileLocalOrientation(1, 0) = localOrientation[1][0];
	missileLocalOrientation(1, 1) = localOrientation[1][1];
	missileLocalOrientation(1, 2) = localOrientation[1][2];
	missileLocalOrientation(2, 0) = localOrientation[2][0];
	missileLocalOrientation(2, 1) = localOrientation[2][1];
	missileLocalOrientation(2, 2) = localOrientation[2][2];

	localRelativePosition = currentLocalPostion - wayPoint;
	localRelativePositionU = localRelativePosition.normalized();
	localRelativePositonMag = localRelativePosition.norm();

	if (timeOfFlight < timeOfGuidanceStart && timeOfFlight < timeOfFinLockOff)
	{

		if (flag == 0)
		{
			flag = 1;
			timeOfGuidanceStart = timeOfFlight;
		}
		phi = rollAngle;
		alpha = alphaDegrees * degToRad;
		beta = betaDegrees * degToRad;
		double bodyToVelocityFrameMatrixTemp[3][3];
		flightPathAnglesToLocalOrientation(beta, alpha, bodyToVelocityFrameMatrixTemp);

		Matrix3d bodyToVelocityFrameMatrix;
		bodyToVelocityFrameMatrix(0, 0) = bodyToVelocityFrameMatrixTemp[0][0];
		bodyToVelocityFrameMatrix(0, 1) = bodyToVelocityFrameMatrixTemp[0][1];
		bodyToVelocityFrameMatrix(0, 2) = bodyToVelocityFrameMatrixTemp[0][2];
		bodyToVelocityFrameMatrix(1, 0) = bodyToVelocityFrameMatrixTemp[1][0];
		bodyToVelocityFrameMatrix(1, 1) = bodyToVelocityFrameMatrixTemp[1][1];
		bodyToVelocityFrameMatrix(1, 2) = bodyToVelocityFrameMatrixTemp[1][2];
		bodyToVelocityFrameMatrix(2, 0) = bodyToVelocityFrameMatrixTemp[2][0];
		bodyToVelocityFrameMatrix(2, 1) = bodyToVelocityFrameMatrixTemp[2][1];
		bodyToVelocityFrameMatrix(2, 2) = bodyToVelocityFrameMatrixTemp[2][2];

		//Velocity tangent unit vector in inertial frame.
		velTan_i_u = currentLocalVelocity.normalized();

		//Sine of Beta unit vector in inertial frame (rolling).
		sbeta_i_u = velTan_i_u.cross(localRelativePosition);

		//Sine of Beta unit vector in body frame (rolling).
		sbeta_b_u = missileLocalOrientation * sbeta_i_u;

		//Sine of Beta unit vector in body frame (non-rolling).
		sbeta_b_nr = rolledToNonRolledBodyMatrix * sbeta_b_u;

		//Sine of Beta unit vector in velocity frame (non-rolling).
		sbeta_vel_nr = bodyToVelocityFrameMatrix * sbeta_b_nr;

		//Missile relative velocity cross desired velocity in inertial frame (rolling).
		vXvd_i_u = velTan_i_u.cross(velTan_d);

		//Missile relative velocity cross desired velocity in body frame (rolling).
		vXvd_b_u = missileLocalOrientation *vXvd_i_u;

		//Missile relative velocity cross desired velocity in body frame (non-rolling).
		vXvd_b_nr = rolledToNonRolledBodyMatrix * vXvd_b_u;

		//Missile relative velocity Cross desired velocity in Velocity frame (non-rolling)
		vXvd_vel_nr = bodyToVelocityFrameMatrix * vXvd_b_nr;

		rangeBetweenLastPositionAndCurrent = (currentLocalPostion - lastLocalPostion).norm();
		if (rangeBetweenLastPositionAndCurrent >= rangeBetweenLastPositionAndCurrentLimit && timeOfFlight >= timeOfGuidanceStart + 1.0)
		{
			XKNGainFlag = 1;
			timeOfKalmanFilterInit = timeOfFlight;
		}

		int tableIndex;
		if (XKNGainFlag == -1)
		{
			tableIndex = tableNameIndexPairs["XKN1_G"];
			XKN = linearInterpolationWithBoundedEnds(tables[tableIndex], timeOfFlight - timeOfGuidanceStart);
		}
		else
		{
			tableIndex = tableNameIndexPairs["XKN2_G"];
			XKN = linearInterpolationWithBoundedEnds(tables[tableIndex], timeOfFlight - timeOfKalmanFilterInit);
		}
		tableIndex = tableNameIndexPairs["XKS_G"];
		XKS = linearInterpolationWithBoundedEnds(tables[tableIndex], timeOfFlight - timeOfGuidanceStart);

		yawGammaCommand = XKN * (XKS * asin(sbeta_vel_nr[2]) - asin(vXvd_vel_nr[2]));
		pitchGammaCommand = XKN * (XKS * asin(sbeta_vel_nr[1]) - asin(vXvd_vel_nr[1]));

	}

}