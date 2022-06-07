#include "util.h"
#include "vector"
#include "map"

using namespace std;

class aerodynamics
{

	public:

		// CONSTRUCTOR
		aerodynamics(mslDataPacket *dataPacket, std::string inPutDataFile, double alpha_prime_max);
		double alphaPrimeMax;
		void formatDataTables(std::string inPutDataFile);
		map<string, int> tableNameIndexPairs;
		vector<vector<vector<double>>> tables;

		// UPDATE FUNCTION
		void update(mslDataPacket *dataPacket);
		mslDataPacket tempDataPacket;

		// CLASS FUNCTIONS
		void aeroBallisticAngles(mslDataPacket *dataPacket);
		double alphaPrimeDeg, sinPhiPrime, cosPhiPrime, pitchDeflAeroDeg, yawDeflAeroDeg, rollDeflDeg, totalFinDeflDeg, pitchRateAeroDeg, yawRateAeroDeg, rollRateDeg, sinOfFourTimesPhiPrime, squaredSinOfTwoTimesPhiPrime;
		
		void aeroDynamicCoefficients(mslDataPacket *dataPacket);
		double CA0, CAA, CAD, CAOFF, CYP, CYDR, CN0, CNP, CNDQ, CLLAP, CLLP, CLLDP, CLM0, CLMP, CLMQ, CLMDQ, CLNP;
		double CX, CY, CZ, CL, CM, CN;

		void aeroDynamicReferenceValues(mslDataPacket *dataPacket);
		double CNA, CMA, CND, CMD, CMQ, CLP, CLD, staticMargin;

};