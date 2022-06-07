#include "iostream"
#include "fstream"
#include "math.h"
#include "vector"
#include "sstream"

#include "aerodynamics.h"
#include "../util.h"

using namespace std;

aerodynamics::aerodynamics(mslDataPacket *dataPacket)
{
	std::cout << "AERODYNAMICS INITIATED" << std::endl;
	alphaPrimeMax = 40.0;
	formatDataTables("src/SRAAMEngine/sixDofSimAero.txt");

	dataPacket->CX = 0.0;
	dataPacket->CY = 0.0;
	dataPacket->CZ = 0.0;
	dataPacket->CL = 0.0;
	dataPacket->CN = 0.0;
	dataPacket->CM = 0.0;

	CNA = 0.0;
	CMA = 0.0;
	CND = 0.0;
	CMD = 0.0;
	CMQ = 0.0;
	CLP = 0.0;
	CLD = 0.0;
}

void aerodynamics::formatDataTables(std::string inPutDataFile)
{
	// LOOK UP DATA
	ifstream inFile(inPutDataFile);
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

void aerodynamics::update(
	mslDataPacket *dataPacket,
	double rollFinDefl,
	double pitchFinDefl,
	double yawFinDefl,
	double launchCg,
	double cgFromNose,
	double maxAccelAllowed
)
{
	aeroBallisticAngles(
		dataPacket,
		rollFinDefl,
		pitchFinDefl,
		yawFinDefl
	);
	aeroDynamicCoefficients(
		dataPacket,
		launchCg,
		cgFromNose,
		maxAccelAllowed
	);
	aeroDynamicReferenceValues(
		dataPacket,
		launchCg,
		cgFromNose
	);

	dataPacket->CX = CX;
	dataPacket->CY = CY;
	dataPacket->CZ = CZ;
	dataPacket->CL = CL;
	dataPacket->CN = CN;
	dataPacket->CM = CM;

}

void aerodynamics::aeroBallisticAngles(
	mslDataPacket *dataPacket,
	double rollFinDefl,
	double pitchFinDefl,
	double yawFinDefl
)
{
	double alphaPrime = acos(cos(dataPacket->mslAlpha) * cos(dataPacket->mslBeta));
	alphaPrimeDeg = radToDeg * alphaPrime;
	double phiPrime = atan2(tan(dataPacket->mslBeta), sin(dataPacket->mslAlpha));
	sinPhiPrime = sin(phiPrime);
	cosPhiPrime = cos(phiPrime);
	double pitchDeflAeroFrame = pitchFinDefl * cosPhiPrime - yawFinDefl * sinPhiPrime;
	pitchDeflAeroDeg = radToDeg * pitchDeflAeroFrame;
	double yawDeflAeroFrame = pitchFinDefl * sinPhiPrime + yawFinDefl * cosPhiPrime;
	yawDeflAeroDeg = radToDeg * yawDeflAeroFrame;
	rollDeflDeg = radToDeg * rollFinDefl;
	totalFinDeflDeg = (abs(pitchDeflAeroDeg) + abs(yawDeflAeroDeg)) / 2;
	double pitchRateAeroFrame = dataPacket->mslEulerDot[1] * cosPhiPrime - dataPacket->mslEulerDot[2] * sinPhiPrime;
	pitchRateAeroDeg = radToDeg * pitchRateAeroFrame;
	double yawRateAeroFrame = dataPacket->mslEulerDot[1] * sinPhiPrime + dataPacket->mslEulerDot[2] * cosPhiPrime;
	yawRateAeroDeg = radToDeg * yawRateAeroFrame;
	rollRateDeg = radToDeg * dataPacket->mslEulerDot[0];
	sinOfFourTimesPhiPrime = sin(4 * phiPrime);
	squaredSinOfTwoTimesPhiPrime = pow((sin(2 * phiPrime)), 2);
}

void aerodynamics::aeroDynamicCoefficients(
	mslDataPacket *dataPacket,
	double launchCg,
	double cgFromNose,
	double maxAccelAllowed
)
{
	double mslMach = dataPacket->mslMach;
	int tableIndex;

	tableIndex = tableNameIndexPairs["CA0"];
	CA0 = linearInterpolationWithBoundedEnds(tables[tableIndex], mslMach);

	tableIndex = tableNameIndexPairs["CAA"];
	CAA = linearInterpolationWithBoundedEnds(tables[tableIndex], mslMach);

	tableIndex = tableNameIndexPairs["CAD"];
	CAD = linearInterpolationWithBoundedEnds(tables[tableIndex], mslMach);

	tableIndex = tableNameIndexPairs["CAOFF"];
	if (dataPacket->mslTof <= 2.421) {
		CAOFF = 0.0;
	}
	else {
		CAOFF = linearInterpolationWithBoundedEnds(tables[tableIndex], mslMach);
	}
	
	tableIndex = tableNameIndexPairs["CYP"];
	CYP = biLinearInterpolationWithBoundedBorders(tables[tableIndex], mslMach, alphaPrimeDeg);
	
	tableIndex = tableNameIndexPairs["CYDR"];
	CYDR = biLinearInterpolationWithBoundedBorders(tables[tableIndex], mslMach, alphaPrimeDeg);
	
	tableIndex = tableNameIndexPairs["CN0"];
	CN0 = biLinearInterpolationWithBoundedBorders(tables[tableIndex], mslMach, alphaPrimeDeg);
	
	tableIndex = tableNameIndexPairs["CNP"];
	CNP = biLinearInterpolationWithBoundedBorders(tables[tableIndex], mslMach, alphaPrimeDeg);
	
	tableIndex = tableNameIndexPairs["CNDQ"];
	CNDQ = biLinearInterpolationWithBoundedBorders(tables[tableIndex], mslMach, alphaPrimeDeg);
	
	tableIndex = tableNameIndexPairs["CLLAP"];
	CLLAP = biLinearInterpolationWithBoundedBorders(tables[tableIndex], mslMach, alphaPrimeDeg);
	
	tableIndex = tableNameIndexPairs["CLLP"];
	CLLP = biLinearInterpolationWithBoundedBorders(tables[tableIndex], mslMach, alphaPrimeDeg);
	
	tableIndex = tableNameIndexPairs["CLLDP"];
	CLLDP = biLinearInterpolationWithBoundedBorders(tables[tableIndex], mslMach, alphaPrimeDeg);
	
	tableIndex = tableNameIndexPairs["CLM0"];
	CLM0 = biLinearInterpolationWithBoundedBorders(tables[tableIndex], mslMach, alphaPrimeDeg);
	
	tableIndex = tableNameIndexPairs["CLMP"];
	CLMP = biLinearInterpolationWithBoundedBorders(tables[tableIndex], mslMach, alphaPrimeDeg);
	
	tableIndex = tableNameIndexPairs["CLMQ"];
	CLMQ = linearInterpolationWithBoundedEnds(tables[tableIndex], mslMach);
	
	tableIndex = tableNameIndexPairs["CLMDQ"];
	CLMDQ = biLinearInterpolationWithBoundedBorders(tables[tableIndex], mslMach, alphaPrimeDeg);
	
	tableIndex = tableNameIndexPairs["CLNP"];
	CLNP = biLinearInterpolationWithBoundedBorders(tables[tableIndex], mslMach, alphaPrimeDeg);

	CX = CA0 + CAA * alphaPrimeDeg + CAD * (totalFinDeflDeg * totalFinDeflDeg) + CAOFF;
	double CYAERO = CYP * sinOfFourTimesPhiPrime + CYDR * yawDeflAeroDeg;
	double CZAERO = CN0 + CNP * squaredSinOfTwoTimesPhiPrime + CNDQ * pitchDeflAeroDeg;
	CL = CLLAP * pow(alphaPrimeDeg, 2) * sinOfFourTimesPhiPrime + CLLP * rollRateDeg * dataPacket->mslRefDiam / (2 * dataPacket->mslSpeed) + CLLDP * rollDeflDeg;
	double CNAEROREF = CLNP * sinOfFourTimesPhiPrime + CLMQ * yawRateAeroDeg * dataPacket->mslRefDiam / (2 * dataPacket->mslSpeed) + CLMDQ * yawDeflAeroDeg;
	double CNAERO = CNAEROREF - CYAERO * (launchCg - cgFromNose) / dataPacket->mslRefDiam;
	double CMAEROREF = CLM0 + CLMP * squaredSinOfTwoTimesPhiPrime + CLMQ * pitchRateAeroDeg * dataPacket->mslRefDiam / (2 * dataPacket->mslSpeed) + CLMDQ * pitchDeflAeroDeg;
	double CMAERO = CMAEROREF - CZAERO * (launchCg - cgFromNose) / dataPacket->mslRefDiam;
	CY = CYAERO * cosPhiPrime - CZAERO * sinPhiPrime;
	CZ = CYAERO * sinPhiPrime + CZAERO * cosPhiPrime;
	CN = CMAERO * sinPhiPrime + CNAERO * cosPhiPrime;
	CM = CMAERO * cosPhiPrime + CNAERO * sinPhiPrime;

	double currentAccel = CN0 * dataPacket->mslDynamicPressure * dataPacket->mslRefArea / dataPacket->mslMass;
	tableIndex = tableNameIndexPairs["CN0"];
	double CN0MAX = biLinearInterpolationWithBoundedBorders(tables[tableIndex], mslMach, alphaPrimeMax);
	double MAXACC = CN0MAX * dataPacket->mslDynamicPressure * dataPacket->mslRefArea / dataPacket->mslMass;
	double availAccel = MAXACC - currentAccel;
	if (availAccel < 0) {
		maxAccel = 1;
	}
	else if (availAccel > maxAccelAllowed) {
		maxAccel = maxAccelAllowed;
	}
	else {
		maxAccel = availAccel;
	}

}

void aerodynamics::aeroDynamicReferenceValues(
	mslDataPacket *dataPacket,
	double launchCg,
	double cgFromNose
)
{

	double mslMach = dataPacket->mslMach;
	int tableIndex;

	double alphaPrimeDegLookUp;
	if (alphaPrimeDeg > (alphaPrimeMax - 3)) {
		alphaPrimeDegLookUp = alphaPrimeMax - 3;
	}
	else {
		alphaPrimeDegLookUp = alphaPrimeDeg;
	}
	double alphaPrimeDegMinusThree = alphaPrimeDegLookUp - 3;
	double alphaPrimeDegPlusThree = alphaPrimeDegLookUp + 3;
	tableIndex = tableNameIndexPairs["CN0"];
	double CN0MIN = biLinearInterpolationWithBoundedBorders(tables[tableIndex], mslMach, alphaPrimeDegMinusThree);
	double CN0MAX = biLinearInterpolationWithBoundedBorders(tables[tableIndex], mslMach, alphaPrimeDegPlusThree);
	CNA = ((CN0MAX - CN0MIN) / (alphaPrimeDegPlusThree - alphaPrimeDegMinusThree)) * radToDeg;
	tableIndex = tableNameIndexPairs["CLM0"];
	double CLM0MIN = biLinearInterpolationWithBoundedBorders(tables[tableIndex], mslMach, alphaPrimeDegMinusThree);
	double CLM0MAX = biLinearInterpolationWithBoundedBorders(tables[tableIndex], mslMach, alphaPrimeDegPlusThree);
	CMA = ((CLM0MAX - CLM0MIN) / (alphaPrimeDegPlusThree - alphaPrimeDegMinusThree) - (CNA / radToDeg) * (launchCg - cgFromNose) / dataPacket->mslRefDiam) * radToDeg;
	CND = CNDQ * radToDeg;
	CMD = CLMDQ * radToDeg;
	CMQ = CLMQ * radToDeg;
	CLP = CLLP * radToDeg;
	CLD = CLLDP * radToDeg;
	staticMargin = -1 * (CMA * degToRad) / (CNA * degToRad);

}