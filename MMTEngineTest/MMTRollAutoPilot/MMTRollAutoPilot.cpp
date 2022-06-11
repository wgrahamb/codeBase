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
#include "../constants.h"

// Third party.
#include "eigen-3.4.0/Eigen/Core"
#include "eigen-3.4.0/Eigen/Dense"

// Header.
#include "MMTRollAutoPilot.h"

MMTRollAutoPilot::MMTRollAutoPilot()
{
	cout << "MMT ROLL AUTO PILOT CONSTRUCTED" << endl;
}

void MMTRollAutoPilot::lookUpTablesFormat (string inPutFile)
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

void MMTRollAutoPilot::init()
{

	lookUpTablesFormat("MMTRollAutoPilot/tables.txt");

	double clp, cld, ajx, xa_roll, xwn_roll, xz_roll;

	int tableIndex;

	// Maybe try to zero out values or even extrapolate for out of bounds. Debug option.

	tableIndex = tableNameIndexPairs["CLP"];
	clp = linearInterpolationWithBoundedEnds(tables[tableIndex], 0.0); // mach

	tableIndex = tableNameIndexPairs["CLD"];
	cld = linearInterpolationWithBoundedEnds(tables[tableIndex], 0.0) * radToDeg; // mach

	tableIndex = tableNameIndexPairs["AJX"];
	ajx = linearInterpolationWithBoundedEnds(tables[tableIndex], 0.0); // time of flight

	tableIndex = tableNameIndexPairs["XA_ROLL"];
	xa_roll = linearInterpolationWithBoundedEnds(tables[tableIndex], 0.0); // q

	tableIndex = tableNameIndexPairs["XWN_ROLL"];
	xwn_roll = linearInterpolationWithBoundedEnds(tables[tableIndex], 0.0); // q

	tableIndex = tableNameIndexPairs["XZ_ROLL"];
	xz_roll = linearInterpolationWithBoundedEnds(tables[tableIndex], 0.0); // q

	rollFinCommandLimitDegrees = 3.0;
	t0 = 0.2;
	timeStep = 1.0 / 1000.0;
	KPR_GAIN = 1.0;
	TDC_GAIN = 7.0;
	phiHatRadians = 0.0;
	phiHatDegrees = 0.0;
	phi0 = 0.0;
	phi1 = 0.0;
	rollDot = 0.0;
	previousInPutPhi1 = 0.0;
	inPutPhi1 = 0.0;
	c0 = xa_roll * xwn_roll * xwn_roll;
	c1 = xwn_roll * xwn_roll + 2.0 * xa_roll * xz_roll * xwn_roll;
	c2 = xa_roll + 2.0 * xz_roll * xwn_roll;

}

void MMTRollAutoPilot::update(
	double timeOfFlight,
	double nonRolledBodyRate[3],
	double mach,
	double dynamicPressure,
	double rollAngle
)
{

	double clp, cld, ajx, xa_roll, xwn_roll, xz_roll;

	int tableIndex;

	// Maybe try to zero out values or even extrapolate for out of bounds. Debug option.

	tableIndex = tableNameIndexPairs["CLP"];
	clp = linearInterpolationWithBoundedEnds(tables[tableIndex], mach); // mach

	tableIndex = tableNameIndexPairs["CLD"];
	cld = linearInterpolationWithBoundedEnds(tables[tableIndex], mach) * radToDeg; // mach

	tableIndex = tableNameIndexPairs["AJX"];
	ajx = linearInterpolationWithBoundedEnds(tables[tableIndex], timeOfFlight); // time of flight

	phiHatRadians = rollAngle;
	phiHatDegrees = rollAngle * radToDeg;

	if (timeOfFlight >= t0)
	{

		LDeltaEstimate = dynamicPressure * referenceArea * referenceDiameter * cld / ajx;

		KNR = c0;
		KPR = c2 * KPR_GAIN;
		KAR = c1;

		inPutPhi1 = rollCommand - phiHatRadians;
		double newPhi1 = integrate(inPutPhi1, previousInPutPhi1, phi1, timeStep);
		phi1 = newPhi1;
		previousInPutPhi1 = inPutPhi1;

		mom_c1 = KNR * phi1 - KAR * phiHatRadians - KPR * nonRolledBodyRate[0];
		roll_dd = mom_c1;
		rollDot += roll_dd * timeStep;

		mom_icar = TDC_GAIN * (rollDot - nonRolledBodyRate[0]);
		mom_c = mom_c1 + mom_icar;

		double rollFinCommandRadians;

		if (timeOfFlight >= t0)
		{

			tableIndex = tableNameIndexPairs["XA_ROLL"];
			xa_roll = linearInterpolationWithBoundedEnds(tables[tableIndex], dynamicPressure); // q

			tableIndex = tableNameIndexPairs["XWN_ROLL"];
			xwn_roll = linearInterpolationWithBoundedEnds(tables[tableIndex], dynamicPressure); // q

			tableIndex = tableNameIndexPairs["XZ_ROLL"];
			xz_roll = linearInterpolationWithBoundedEnds(tables[tableIndex], dynamicPressure); // q

			c0 = xa_roll * xwn_roll * xwn_roll;
			c1 = xwn_roll * xwn_roll + 2.0 * xa_roll * xz_roll * xwn_roll;
			c2 = xa_roll + 2.0 * xz_roll * xwn_roll;

			KNR = c0;
			KPR = c2 * KPR_GAIN;
			KAR = c1;

			double XKI = KNR;
			double XKA = KAR;
			double XKG = KPR * KPR_GAIN;

			rollFinCommandRadians = mom_c / LDeltaEstimate;
			if (abs(rollFinCommandRadians) > (rollFinCommandLimitDegrees * degToRad))
			{
				if (rollFinCommandRadians > 0.0)
				{
					rollFinCommandRadians = rollFinCommandLimitDegrees * degToRad;
				}
				else if (rollFinCommandRadians < 0.0)
				{
					rollFinCommandRadians = -1.0 * rollFinCommandLimitDegrees * degToRad;
				}
			}

			double mcxlmtot = rollFinCommandRadians * LDeltaEstimate;
			double mcxlm = mcxlmtot;
			double mom_c_lm = mcxlm - mom_icar;
			double xxx = 1.0 / XKI * (mom_c_lm + XKA * phiHatRadians + XKG * nonRolledBodyRate[0]);
			phi1 = xxx;

		}

		rollFinCommandDegrees = rollFinCommandRadians * radToDeg;

	}
	else
	{
		rollFinCommandDegrees = 0.0;
	}

}