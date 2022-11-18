// Utility.
#include "util.h"

// Missile model.
#include "missileModel.h"
#include "secondOrderActuator.h"
#include "ATM1976.h"

// Namespace.
using namespace std;
auto wallClockStart = chrono::high_resolution_clock::now();

int main()
{

	// Instantiate inputs.
	int    ballistic;
	int    INTEGRATION_METHOD;
	double phiRads;
	double thetaRads;
	double psiRads;
	double posE;
	double posN;
	double posU;
	double tgtE;
	double tgtN;
	double tgtU;
	int    LogData;
	int    ConsoleReport;
	string ID;

	// Instantiate input file.
	ifstream InputFile;

	// Open input file.
	InputFile.open("CPP_6DOF_SRAAM_V2/input/input.txt");

	// Populate input.
	InputFile
	>> ballistic
	>> INTEGRATION_METHOD
	>> phiRads
	>> thetaRads
	>> psiRads
	>> posE
	>> posN
	>> posU
	>> tgtE
	>> tgtN
	>> tgtU
	>> LogData
	>> ConsoleReport
	>> ID;

	// Instantiate missile.
	Missile missile;

	// Format data tables. Only happens once.
	formatTables(missile, "CPP_6DOF_SRAAM_V2/input/tables.txt");

	// Trajectory and integration type.
	missile.isBallistic = ballistic;
	missile.INTEGRATION_METHOD = INTEGRATION_METHOD;
	
	// Emplacement.
	phiRads *= degToRad;
	thetaRads *= degToRad;
	psiRads *= degToRad;
	double launchPosition[3] = {posE, posN, posU};
	emplace(missile, phiRads, thetaRads, psiRads, launchPosition);
	
	// Waypoint.
	double pip[3] = {tgtE, tgtN, tgtU};
	setArrayEquivalentToReference(missile.waypoint, pip);
	seekerOn(missile);

	// Set lethality to flying. Missile will not fly unless.
	missile.lethality = "FLYING";
	missile.isLaunched = true;

	// six dof missile flight.
	Missile missile1 = clone(missile);
	sixDofFly(missile1, ID, LogData, ConsoleReport, 400.0);

	// // three dof missile flight.
	// Missile missile2 = clone(missile);
	// threeDofFly(missile2, "missile2", LogData, ConsoleReport, 400.0);

	// Console report and terminate.
	auto wallClockEnd = chrono::high_resolution_clock::now();
	auto simRealRunTime = chrono::duration_cast<chrono::milliseconds>(wallClockEnd - wallClockStart);
	cout << "\nSIMULATION RUN TIME :" << simRealRunTime.count() << " MILLISECONDS" << endl;
	cout << "\n";
	return 0;

}