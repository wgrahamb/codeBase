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

// Namespace.
using namespace std;
using namespace Eigen;

class MMTRollAutoPilot
{

	public:

		// Functions.
		MMTRollAutoPilot();
		map<string, int> tableNameIndexPairs;
		vector<vector<vector<double>>> tables;
		void lookUpTablesFormat(string inPutFile);
		void init();
		void update(
			double timeOfFlight,
			double nonRolledBodyRate[3],
			double mach,
			double dynamicPressure,
			double rollAngle
		);

		// Variables.
		double rollFinCommandLimitDegrees;
		double t0;
		double timeStep;
		double KPR_GAIN;
		double TDC_GAIN;
		double phiHatRadians;
		double phiHatDegrees;
		double phi0;
		double phi1;
		double rollDot;
		double roll_dd;
		double previousInPutPhi1;
		double inPutPhi1;
		double c0;
		double c1;
		double c2;
		double LDeltaEstimate;
		double KNR;
		double KPR;
		double KAR;
		const double rollCommand = 0.0;
		double mom_c1;
		double mom_icar;
		double mom_c;
		double rollFinCommandDegrees;

};