#include "../util.h"
#include "vector"
#include "map"

using namespace std;

class aerodynamics
{

	public:

		// CONSTRUCTOR
		aerodynamics(mslDataPacket *dataPacket);
		double alphaPrimeMax;
		void formatDataTables(std::string inPutDataFile);
		map<string, int> tableNameIndexPairs;
		vector<vector<vector<double>>> tables;

		// UPDATE FUNCTION
		void update(
			mslDataPacket *dataPacket,
			double rollFinDefl,
			double pitchFinDefl,
			double yawFinDefl,
			double launchCg,
			double cgFromNose,
			double maxAccelAllowed
		);

		// CLASS FUNCTIONS
		void aeroBallisticAngles(
			mslDataPacket *dataPacket,
			double rollFinDefl,
			double pitchFinDefl,
			double yawFinDefl
		);
		double alphaPrimeDeg, sinPhiPrime, cosPhiPrime, pitchDeflAeroDeg, yawDeflAeroDeg, rollDeflDeg, totalFinDeflDeg, pitchRateAeroDeg, yawRateAeroDeg, rollRateDeg, sinOfFourTimesPhiPrime, squaredSinOfTwoTimesPhiPrime;
		
		void aeroDynamicCoefficients(
			mslDataPacket *dataPacket,
			double launchCg,
			double cgFromNose,
			double maxAccelAllowed
		);
		double CA0, CAA, CAD, CAOFF, CYP, CYDR, CN0, CNP, CNDQ, CLLAP, CLLP, CLLDP, CLM0, CLMP, CLMQ, CLMDQ, CLNP;
		double CX, CY, CZ, CL, CM, CN;
		double maxAccel;

		void aeroDynamicReferenceValues(
			mslDataPacket *dataPacket,
			double launchCg,
			double cgFromNose
		);
		double CNA, CMA, CND, CMD, CMQ, CLP, CLD, staticMargin;

};