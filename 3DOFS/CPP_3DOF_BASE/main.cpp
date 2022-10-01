
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

	double pip[3]; // Meters.
	string lethality; // Lethality.
	double timeOfFlight; // Seconds.
	double ENUPosition[3]; // Meters.
	double range; // Meters.
	double FLUMissileToPipPosition[3]; // Meters.
	double missDistance; // Meters.
	double yaw; // Radians.
	double pitch; // Radians.
	double ENUToFLU[3][3]; // Non dimensional.
	double ENUVelocity[3]; // Meters per second.
	double FLUVelocity[3]; // Meters per second.
	double normalGuidanceCommand; // Meters per s^2.
	double sideGuidanceCommand; // Meters per s^2.
	double ENUAcceleration[3]; // Meters per s^2.
	double FLUAcceleration[3]; // Meters per s^2.

};

void emplace(Missile &missile, double azimuthDegrees, double elevationDegrees, double missileSpeed)
{

	missile.lethality = "LOITERING";
	missile.timeOfFlight = 0.0;
	setArrayEquivalentToZero(missile.ENUPosition);
	missile.range = 0.0;
	missile.yaw = azimuthDegrees * degToRad;
	missile.pitch = elevationDegrees * degToRad;
	flightPathAnglesToLocalOrientation(missile.yaw, -1.0 * missile.pitch, missile.ENUToFLU);
	multiplyVectorTimesScalar(missileSpeed, missile.ENUToFLU[0], missile.ENUVelocity);
	threeByThreeTimesThreeByOne(missile.ENUToFLU, missile.ENUVelocity, missile.FLUVelocity);
	missile.normalGuidanceCommand = 0.0;
	missile.sideGuidanceCommand = 0.0;
	setArrayEquivalentToZero(missile.ENUAcceleration);
	setArrayEquivalentToZero(missile.FLUAcceleration);

}

void waypoint(Missile &missile, double pip[3])
{

	setArrayEquivalentToReference(missile.pip, pip);
	double ENUMissileToPipPosition[3];
	subtractTwoVectors(missile.ENUPosition, missile.pip, ENUMissileToPipPosition);
	threeByThreeTimesThreeByOne(missile.ENUToFLU, ENUMissileToPipPosition, missile.FLUMissileToPipPosition);
	magnitude(missile.FLUMissileToPipPosition, missile.missDistance);

}

void guidance(Missile &missile)
{

	double localRelativePosition[3];
	subtractTwoVectors(missile.ENUPosition, missile.pip, localRelativePosition);
	threeByThreeTimesThreeByOne(missile.ENUToFLU, localRelativePosition, missile.FLUMissileToPipPosition);
	double forwardLeftUpMissileToInterceptPositionUnitVector[3];
	unitVec(missile.FLUMissileToPipPosition, forwardLeftUpMissileToInterceptPositionUnitVector);
	double closingVelocity[3];
	multiplyVectorTimesScalar(-1.0, missile.FLUVelocity, closingVelocity);
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

	missile.FLUAcceleration[0] = 0.0;
	missile.FLUAcceleration[1] = missile.sideGuidanceCommand;
	missile.FLUAcceleration[2] = missile.normalGuidanceCommand;
	oneByThreeTimesThreeByThree(missile.FLUAcceleration, missile.ENUToFLU, missile.ENUAcceleration);

	double deltaVelocity[3];
	multiplyVectorTimesScalar(TIME_STEP, missile.ENUAcceleration, deltaVelocity);
	double newMissileVelocity[3];
	addTwoVectors(missile.ENUVelocity, deltaVelocity, newMissileVelocity);
	setArrayEquivalentToReference(missile.ENUVelocity, newMissileVelocity);

	double deltaPosition[3];
	multiplyVectorTimesScalar(TIME_STEP, missile.ENUVelocity, deltaPosition);
	double newMissilePosition[3];
	addTwoVectors(missile.ENUPosition, deltaPosition, newMissilePosition);
	setArrayEquivalentToReference(missile.ENUPosition, newMissilePosition);

	missile.timeOfFlight += TIME_STEP;

	double distanceTravelled;
	magnitude(deltaPosition, distanceTravelled);
	missile.range += distanceTravelled;

	azAndElFromVector(missile.yaw, missile.pitch, missile.ENUVelocity);
	flightPathAnglesToLocalOrientation(missile.yaw, -1 * missile.pitch, missile.ENUToFLU);

}

void endCheck(Missile &missile)
{

	magnitude(missile.FLUMissileToPipPosition, missile.missDistance);

	if (missile.ENUPosition[2] < 0.0)
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
	else if (isnan(missile.ENUPosition[0]))
	{
		missile.lethality = "NAN";
	}
	else if (missile.timeOfFlight > MAX_TIME)
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
			LogFile << missile.timeOfFlight << " " <<
			missile.ENUPosition[0] << " " <<
			missile.ENUPosition[1] << " " <<
			missile.ENUPosition[2] << " " <<
			missile.pip[0] << " " <<
			missile.pip[1] << " " <<
			missile.pip[2] << "\n";
		}
		if (ConsoleReport)
		{
			auto print_it = static_cast<int>(round(missile.timeOfFlight * 10000.0)) % 10000;
			if (print_it == 0)
			{
				cout << setprecision(10) <<
				"TOF " << missile.timeOfFlight <<
				" ENU " << missile.ENUPosition[0] <<
				" " << missile.ENUPosition[1] <<
				" " << missile.ENUPosition[2] << endl;
				lastTime = missile.timeOfFlight;
			}
		}
	}

	// Console report.
	if (ConsoleReport)
	{
		cout << "\n";
		cout << identity << " REPORT" << endl;
		cout << setprecision(10) << "FINAL STATUS AT TIME OF FLIGHT " << missile.timeOfFlight << " X " << missile.ENUPosition[0] << " Y " << missile.ENUPosition[1] << " Z " << missile.ENUPosition[2] << " RANGE " << missile.range << endl;
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