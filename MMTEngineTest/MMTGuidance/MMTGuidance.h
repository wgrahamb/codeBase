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

// Namespace.
using namespace std;
using namespace Eigen;

class MMTGuidance
{

	public:

		// Functions.
		MMTGuidance();
		map<string, int> tableNameIndexPairs;
		vector<vector<vector<double>>> tables;
		void lookUpTablesFormat(string inPutFile);
		void init();
		void update(
			double timeOfFlight,
			double localWayPoint[3],
			double localPosition[3],
			double localVelocity[3],
			double rollAngle,
			double localOrientation[3][3],
			double rolledToNonRolledMatrix[3][3],
			double alphaDegrees,
			double betaDegrees
		);

		// Variables
		double timeStep;
		int flag;
		double timeOfFinLockOff;
		const double rollCommand = 0.0;
		double timeOfGuidanceStart;
		double timeOfKalmanFilterInit;
		int XKNGainFlag;
		Vector3d currentLocalPostion;
		Vector3d currentLocalVelocity;
		Vector3d lastLocalPostion;
		Vector3d lastLocalVelocity;
		double missileSpeed;
		double phi;
		Vector3d wayPoint;
		double yawGammaDesiredDegrees;
		double pitchGammaDesiredDegrees;
		Vector3d velTan_d;
		Vector3d localRelativePosition;
		Vector3d localRelativePositionU;
		double localRelativePositonMag;
		Vector3d bodyVel;
		Vector3d bodyVelNonRolled;
		double alpha;
		double beta;
		Vector3d velTan_i_u;
		Vector3d sbeta_i_u;
		Vector3d sbeta_b_u;
		Vector3d sbeta_b_nr;
		Vector3d sbeta_vel_nr;
		Vector3d vXvd_i_u;
		Vector3d vXvd_b_u;
		Vector3d vXvd_b_nr;
		Vector3d vXvd_vel_nr;
		double yawGammaCommand;
		double pitchGammaCommand;
		double XKS;
		double XKN;
		double rangeBetweenLastPositionAndCurrent;
		double rangeBetweenLastPositionAndCurrentLimit;
		double gammaDotLimit;
		double flightPathAngleTolerance;
		Matrix3d rolledToNonRolledBodyMatrix;
		Matrix3d missileLocalOrientation;

};