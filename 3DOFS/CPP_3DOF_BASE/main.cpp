
// Standard.
#include "iostream"
#include "iomanip"
#include "chrono"
#include "math.h"
#include "vector"

//Utility.
#include "util.h"

// Namespace.
using namespace std;

// Constant time step.
const double TIME_STEP = 1.0 / 1000.0;
const double MAX_TIME = 200.0;

// Missile struct.
struct Missile
{

	double waypoint[3]; // Meters.
	string lethality; // Lethality.
	double tof; // Seconds.
	double enuPos[3]; // Meters.
	double rng; // Meters.
	double FLUMissileToPipPosition[3]; // Meters.
	double missDistance; // Meters.
	double yaw; // Radians.
	double pitch; // Radians.
	double ENUToFLU[3][3]; // Non dimensional.
	double enuVel[3]; // Meters per second.
	double fluVel[3]; // Meters per second.
	double normalGuidanceCommand; // Meters per s^2.
	double sideGuidanceCommand; // Meters per s^2.
	double enuAcc[3]; // Meters per s^2.
	double spcfForce[3]; // Meters per s^2.

};

void emplace(Missile &missile, double azimuthDegrees, double elevationDegrees, double missileSpeed)
{

	missile.lethality = "LOITERING";
	missile.tof = 0.0;
	setArrayEquivalentToZero(missile.enuPos);
	missile.rng = 0.0;
	missile.yaw = azimuthDegrees * degToRad;
	missile.pitch = elevationDegrees * degToRad;
	flightPathAnglesToLocalOrientation(missile.yaw, -1.0 * missile.pitch, missile.ENUToFLU);
	multiplyVectorTimesScalar(missileSpeed, missile.ENUToFLU[0], missile.enuVel);
	threeByThreeTimesThreeByOne(missile.ENUToFLU, missile.enuVel, missile.fluVel);
	missile.normalGuidanceCommand = 0.0;
	missile.sideGuidanceCommand = 0.0;
	setArrayEquivalentToZero(missile.enuAcc);
	setArrayEquivalentToZero(missile.spcfForce);

}

void waypoint(Missile &missile, double pip[3])
{

	setArrayEquivalentToReference(missile.waypoint, pip);
	double ENUMissileToPipPosition[3];
	subtractTwoVectors(missile.enuPos, missile.waypoint, ENUMissileToPipPosition);
	threeByThreeTimesThreeByOne(missile.ENUToFLU, ENUMissileToPipPosition, missile.FLUMissileToPipPosition);
	magnitude(missile.FLUMissileToPipPosition, missile.missDistance);

}

void guidance(Missile &missile)
{

	double localRelativePosition[3];
	subtractTwoVectors(missile.enuPos, missile.waypoint, localRelativePosition);
	threeByThreeTimesThreeByOne(missile.ENUToFLU, localRelativePosition, missile.FLUMissileToPipPosition);
	double forwardLeftUpMissileToInterceptPositionUnitVector[3];
	unitVec(missile.FLUMissileToPipPosition, forwardLeftUpMissileToInterceptPositionUnitVector);
	double closingVelocity[3];
	multiplyVectorTimesScalar(-1.0, missile.fluVel, closingVelocity);
	double closingSpeed;
	magnitude(closingVelocity, closingSpeed);
	double TEMP1[3], TEMP2;
	crossProductTwoVectors(missile.FLUMissileToPipPosition, closingVelocity, TEMP1);
	dotProductTwoVectors(missile.FLUMissileToPipPosition, missile.FLUMissileToPipPosition, TEMP2);
	double lineOfSightRate[3];
	divideVectorByScalar(TEMP2, TEMP1, lineOfSightRate);
	double TEMP3, TEMP4[3];
	double proportionalGuidanceGain = 3.0;
	TEMP3 = -1 * proportionalGuidanceGain * closingSpeed;
	multiplyVectorTimesScalar(TEMP3, forwardLeftUpMissileToInterceptPositionUnitVector, TEMP4);
	double COMMAND[3];
	crossProductTwoVectors(TEMP4, lineOfSightRate, COMMAND);
	missile.normalGuidanceCommand = COMMAND[2];
	missile.sideGuidanceCommand = COMMAND[1];

}

void missileMotion(Missile &missile)
{

	missile.spcfForce[0] = 0.0;
	missile.spcfForce[1] = missile.sideGuidanceCommand;
	missile.spcfForce[2] = missile.normalGuidanceCommand;
	oneByThreeTimesThreeByThree(missile.spcfForce, missile.ENUToFLU, missile.enuAcc);

	double deltaVelocity[3];
	multiplyVectorTimesScalar(TIME_STEP, missile.enuAcc, deltaVelocity);
	double newMissileVelocity[3];
	addTwoVectors(missile.enuVel, deltaVelocity, newMissileVelocity);
	setArrayEquivalentToReference(missile.enuVel, newMissileVelocity);

	double deltaPosition[3];
	multiplyVectorTimesScalar(TIME_STEP, missile.enuVel, deltaPosition);
	double newMissilePosition[3];
	addTwoVectors(missile.enuPos, deltaPosition, newMissilePosition);
	setArrayEquivalentToReference(missile.enuPos, newMissilePosition);

	missile.tof += TIME_STEP;

	double distanceTravelled;
	magnitude(deltaPosition, distanceTravelled);
	missile.rng += distanceTravelled;

	azAndElFromVector(missile.yaw, missile.pitch, missile.enuVel);
	flightPathAnglesToLocalOrientation(missile.yaw, -1 * missile.pitch, missile.ENUToFLU);

}

void endCheck(Missile &missile)
{

	magnitude(missile.FLUMissileToPipPosition, missile.missDistance);

	if (missile.enuPos[2] < 0.0)
	{
		missile.lethality = "GROUND";
	}
	else if (missile.missDistance < 5.0)
	{
		missile.lethality = "INTERCEPT";
	}
	else if (missile.FLUMissileToPipPosition[0] < 0.0)
	{
		missile.lethality = "POCA"; // POINT OF CLOSEST APPROACH.
	}
	else if (isnan(missile.enuPos[0]))
	{
		missile.lethality = "NAN";
	}
	else if (missile.tof > MAX_TIME)
	{
		missile.lethality = "TIME";
	}

}

void update(Missile &missile)
{

	guidance(missile);
	missileMotion(missile);
	endCheck(missile);

}

void fly(Missile &missile, bool LogData, bool ConsoleReport, string identity)
{

	// Run.
	double lastTime = 0;
	ofstream LogFile;
	if (LogData)
	{
		string fileName = "3DOFS/CPP_3DOF_BASE/output/flyout_" + identity + ".txt";
		LogFile.open(fileName);
		LogFile << "tof posE posN posU tgtE tgtN tgtU\n";
	}

	if (ConsoleReport)
	{
		cout << identity << " LAUNCH" << endl;
	}

	missile.lethality = "FLYING";
	while (missile.lethality == "FLYING")
	{
		update(missile);
		if (LogData)
		{
			LogFile << missile.tof << " " <<
			missile.enuPos[0] << " " <<
			missile.enuPos[1] << " " <<
			missile.enuPos[2] << " " <<
			missile.waypoint[0] << " " <<
			missile.waypoint[1] << " " <<
			missile.waypoint[2] << "\n";
		}
		if (ConsoleReport)
		{
			auto print_it = static_cast<int>(round(missile.tof * 10000.0)) % 10000;
			if (print_it == 0)
			{
				cout << setprecision(10) <<
				"TOF " << missile.tof <<
				" ENU " << missile.enuPos[0] <<
				" " << missile.enuPos[1] <<
				" " << missile.enuPos[2] << endl;
				lastTime = missile.tof;
			}
		}
	}

	// Console report.
	if (ConsoleReport)
	{
		cout << "\n";
		cout << identity << " REPORT" << endl;
		cout << setprecision(10) << "FINAL STATUS AT TIME OF FLIGHT " << missile.tof << " X " << missile.enuPos[0] << " Y " << missile.enuPos[1] << " Z " << missile.enuPos[2] << " RANGE " << missile.rng << endl;
		cout << setprecision(10) << "MISS DISTANCE " << missile.missDistance << " >>> FORWARD, LEFT, UP MISS DISTANCE " << missile.FLUMissileToPipPosition[0] << " " << missile.FLUMissileToPipPosition[1] << " " << missile.FLUMissileToPipPosition[2] << endl;
		cout << "SIMULATION RESULT: " << missile.lethality << endl;
	}

}

int main()
{

	// Start wall clock.
	auto wallClockStart = chrono::high_resolution_clock::now();

	// Create missile.
	Missile originalMissile;

	// Format console output.
	cout << "\n";

	// Initialize.
	double launchAzimuth = 45.0;
	double launchElevation = 75.0;
	double missileSpeed = 350.0;
	emplace(originalMissile, launchAzimuth, launchElevation, missileSpeed);

	// Set pip.
	double pip[3];
	pip[0] = 3000.0;
	pip[1] = 3000.0;
	pip[2] = 3000.0;
	waypoint(originalMissile, pip);

	// Fly good shot.
	fly(originalMissile, true, true, "originalMissile");

	// Run time.
	cout << "\n";
	auto wallClockEnd = chrono::high_resolution_clock::now();
	auto simRealRunTime = chrono::duration_cast<chrono::milliseconds>(wallClockEnd - wallClockStart);
	cout << "SIMULATION RUN TIME :" << simRealRunTime.count() / 1000.0 << " SECONDS" << endl;
	cout << "\n" << endl;

	// Terminate program.
	return 0;

}