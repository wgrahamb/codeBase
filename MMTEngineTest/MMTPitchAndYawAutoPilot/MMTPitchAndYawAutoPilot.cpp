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
#include "MMTPitchAndYawAutoPilot.h"

MMTPitchAndYawAutoPilot::MMTPitchAndYawAutoPilot()
{
	cout << "MMT PITCH AND YAW AUTO PILOT CONSTRUCTED" << endl;
}

void MMTPitchAndYawAutoPilot::lookUpTablesFormat (string inPutFile)
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

void MMTPitchAndYawAutoPilot::init()
{

	lookUpTablesFormat("MMTRollAutoPilot/tables.txt");

	double cna, cnd, cmd0, ajy, xa_pitch_and_yaw, xwn_pitch_and_yaw, xz_pitch_and_yaw;
	double cna_a1, cna_a2, cma_b1, cma_b2;

	int tableIndex;

	// Maybe try to zero out values or even extrapolate for out of bounds. Debug option.
	// Not necessary to perform these look ups here, but serves as a test to make sure the data stored correctly.
	tableIndex = tableNameIndexPairs["CNA"];
	cna = linearInterpolationWithBoundedEnds(tables[tableIndex], 0.0); // mach

	tableIndex = tableNameIndexPairs["CND"];
	cnd = linearInterpolationWithBoundedEnds(tables[tableIndex], 0.0); // mach

	tableIndex = tableNameIndexPairs["CMD0"];
	cmd0 = linearInterpolationWithBoundedEnds(tables[tableIndex], 0.0); // mach

	tableIndex = tableNameIndexPairs["AJY"];
	ajy = linearInterpolationWithBoundedEnds(tables[tableIndex], 0.0); // time of flight

	tableIndex = tableNameIndexPairs["XA_PITCH_AND_YAW"];
	xa_pitch_and_yaw = linearInterpolationWithBoundedEnds(tables[tableIndex], 0.0); // q

	tableIndex = tableNameIndexPairs["XWN_PITCH_AND_YAW"];
	xwn_pitch_and_yaw = linearInterpolationWithBoundedEnds(tables[tableIndex], 0.0); // q

	tableIndex = tableNameIndexPairs["XZ_PITCH_AND_YAW"];
	xz_pitch_and_yaw = linearInterpolationWithBoundedEnds(tables[tableIndex], 0.0); // q

	tableIndex = tableNameIndexPairs["CNA_A1"];
	cna_a1 = linearInterpolationWithBoundedEnds(tables[tableIndex], 0.0); // mach

	tableIndex = tableNameIndexPairs["CNA_A2"];
	cna_a2 = linearInterpolationWithBoundedEnds(tables[tableIndex], 0.0); // mach

	tableIndex = tableNameIndexPairs["CMA_B1"];
	cma_b1 = linearInterpolationWithBoundedEnds(tables[tableIndex], 0.0); // mach

	tableIndex = tableNameIndexPairs["CMA_B2"];
	cma_b2 = linearInterpolationWithBoundedEnds(tables[tableIndex], 0.0); // mach

}

void MMTPitchAndYawAutoPilot::update()
{

}