#include "util.h"
#include "vector"
#include "map"

using namespace std;

class massAndMotor
{

	public:

		// CONSTRUCTOR
		massAndMotor(mslDataPacket *dataPacket, std::string inPutDataFile);
		double seaLevelPress;
		double mslRefArea, mslRefDiam, mslExitArea, mslBurnOut, launchCg;
		double mass, unAdjThrust, transverseMomentOfInertia, axialMomentOfInertia, cgFromNose, thrust;
		void formatDataTables(std::string inPutDataFile);
		map<string, int> tableNameIndexPairs;
		vector<vector<vector<double>>> tables;


		// UPDATE FUNCTION
		void update(mslDataPacket *dataPacket);

};