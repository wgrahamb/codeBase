// Standard.
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <math.h>
#include <vector>
#include <map>
#include <algorithm>

// Utility.
#include "util.h"

// Missile model.
#include "missileModel.h"

// Namespace.
using namespace std;

/* To do */
// For missile model, needs a guidance law for pip and for moving target. Also a "seeker on" mode.
// Pip selection algorithm needs work.
	// Should be able to account for a slewing launcher.
	// This makes for better shots.
// Start a simple simulation loop.
// Structs for NOVICE players.
// Scene generator.
// Scene loader.
// Seeker on mode for missile.
// Algorithm to scale acceleration limit with energy.
// Each missile should have a pip and target states.

// Simulation control.
auto wallClockStart = chrono::high_resolution_clock::now(); // Start tracking real time.
const double CONSTANT_TIME_STEP = 1 / 1000.0; // Seconds. Common sense to have a uniform time step if can.
const double CONSTANT_HALF_TIME_STEP = CONSTANT_TIME_STEP / 2.0; // Seconds. For rk2 and rk4 integration.

// Struct only for holding a triplet array. All my functions are designed for tripley arrays.
struct triple
{

	double triplet[3];
	triple();
	triple(double input[3]) : triplet{input[0], input[1], input[2]} {}

};
typedef pair<double, triple> trajectoryPoint; // Specific pair that defines a point on a trajectory. Contains a time of flight and a position. Used for propagations.
typedef vector<trajectoryPoint> trajectory; // Specific vector to hold trajectory points. Defines a full trajectory.

// Defines an asset. A position that soldiers are trying to protect.
struct asset
{

	string identity;
	triple ENUPosition;
	double simulationTime;
	string status;

};

// Defines a launcher. Determines the launch orientation of the missile.
struct launcher
{

	string identity;
	triple ENUPosition;
	double simulationTime;
	string status;

	double launcherAzimuth;
	double launcherElevation;
	vector<Missile> inventory;

};

// Defines a solder. This is an active interceptor.
struct soldier
{

	string identity;
	triple ENUPosition;
	double simulationTime;
	string status;

	Missile interceptor;

};



trajectory propagateTarget(double targetENUPosition[3], double targetENUVelocity[3])
{

	trajectory targetTrajectory; // 0 Place is time of flight. 1, 2, and 3 place is the target position.

	triple initialTargetPosition(targetENUPosition);
	targetTrajectory.push_back(make_pair(0.0, initialTargetPosition));

	for (int i = 0; i < 100000; i++)
	{

		double time = CONSTANT_TIME_STEP * i;
		double deltaPos[3];
		multiplyVectorTimesScalar(CONSTANT_TIME_STEP, targetENUVelocity, deltaPos);
		addTwoVectors(initialTargetPosition.triplet, deltaPos, initialTargetPosition.triplet);
		targetTrajectory.push_back(make_pair(time, initialTargetPosition));
		if (initialTargetPosition.triplet[2] < 0.0) // Below ground.
		{
			break;
		}

	}

	return targetTrajectory;

}

// Pip selection algorithm is good for now. May need work for optimization.
// I'm thinking a "radar" range of 20000.
// Pip selection works for targets that are moving toward or away from the launch site.
triple pipSelection(Missile &missile, trajectory targetTrajectory, bool showProcess)
{

	// Return.
	double retPip[3];

	// Determines return.
	bool goodShot = false;

	// Flyout time step.
	double flyoutTimeStep = 1.0 / 200.0; // Smaller time step for faster flyouts.

	// Check to see if the target is moving away or toward the missile.
	int lowIndex = 0;
	int highIndex = targetTrajectory.size() - 1;

	trajectoryPoint firstTrajPoint = targetTrajectory[lowIndex];
	double firstTrajPointDistance, firstTrajPointRelPos[3];
	subtractTwoVectors(firstTrajPoint.second.triplet, missile.ENUPosition, firstTrajPointRelPos);
	magnitude(firstTrajPointRelPos, firstTrajPointDistance);

	trajectoryPoint lastTrajPoint = targetTrajectory[highIndex];
	double lastTrajPointDistance, lastTrajPointRelPos[3];
	subtractTwoVectors(lastTrajPoint.second.triplet, missile.ENUPosition, lastTrajPointRelPos);
	magnitude(lastTrajPointRelPos, lastTrajPointDistance);

	// The way it looks for a good shot depends on whether it is moving away or toward the launcher.
	int increment;
	int shotCheckIndex;
	if (lastTrajPointDistance > firstTrajPointDistance) // Moving away.
	{
		increment = (1 / CONSTANT_TIME_STEP);
		shotCheckIndex = lowIndex + increment;
	}
	else // Moving toward you.
	{
		increment = -1.0 * (1 / CONSTANT_TIME_STEP);
		shotCheckIndex = highIndex + increment;
	}

	// Find a good shot.
	int loopCount = 0;
	while (not goodShot)
	{

		loopCount += 1;
		Missile missileCopy = missile;
		trajectoryPoint currentShot = targetTrajectory[shotCheckIndex];
		triple currentPip = currentShot.second;
		setArrayEquivalentToReference(missileCopy.pip, currentPip.triplet);
		initSeeker(missileCopy);
		string id = "flyout" + to_string(loopCount);

		// Check range to target. Works best within five click.
		trajectoryPoint trajPoint = targetTrajectory[shotCheckIndex];
		double trajPointDistance, trajPointRelPos[3];
		subtractTwoVectors(trajPoint.second.triplet, missile.ENUPosition, trajPointRelPos);
		magnitude(trajPointRelPos, trajPointDistance);
		if (trajPointDistance < 3000.0 || trajPointDistance > 6000.0)
		{
			shotCheckIndex += increment;
			continue;
		}

		threeDofFly(missileCopy, id, true, false, 400.0);
		double ratio = missileCopy.timeOfFlight / currentShot.first;

		if (showProcess)
		{

			cout << "FLYOUT " << loopCount << endl;
			cout << "TARGET TIME OF FLIGHT " << currentShot.first << endl;
			cout << "MISSILE TIME OF FLIGHT " << missileCopy.timeOfFlight << endl;
			cout << "GOOD SHOT CHECK " << ratio << endl;
			cout << "INDEX " << shotCheckIndex << endl;
			cout << "SIMULATION RESULT " << missileCopy.lethality << endl;
			cout << "\n";

		}

		// We only care about intercepts.
		if (missileCopy.lethality == "SUCCESSFUL_INTERCEPT")
		{
			if (ratio > 1.0) // Too late. Move the target closer.
			{
				shotCheckIndex += increment;
			}
			// The first time this condition is hit it means a good shot is found.
			// Even if the interceptor arrives seconds early the launch will be scheduled.
			// This way the interceptor arrives on time.
			else
			{
				cout << "GOOD SHOT FOUND!\n";
				setArrayEquivalentToReference(retPip, currentPip.triplet);
				goodShot = true;
			}
		}
		else // Otherwise move the target closer.
		{
			shotCheckIndex += increment;
		}

		if (loopCount > 30) // Any higher and most likely there is no good shot.
		{
			break;
		}

	}

	triple ret(retPip);
	return ret;

}

int main ()
{

	Missile Missile1;
	initUnLaunchedMissile(Missile1, 0.0, 89.0, -20.0);
	Missile1.pip[0] = 4000.0;
	Missile1.pip[1] = 0.0;
	Missile1.pip[2] = 3000.0;
	Missile Missile2 = Missile1;

	// Missile1.pip[0] = 3000.0;
	// Missile1.pip[1] = 1000.0;
	// Missile1.pip[2] = 3000.0;

	// sixDofFly(Missile1, "log", true, true, 400.0);
	// threeDofFly(Missile2, "log", true, true, 400.0);

	// double targetENUVelocity[3] = {-400.0, 0.0, -450.0};
	// double targetENUPosition[3] = {10000.0, 0.0, 10000.0};
	double targetENUVelocity[3] = {500.0, 0.0, 0.0};
	double targetENUPosition[3] = {-5000.0, 0.0, 3000.0};
	// double targetENUVelocity[3] = {-800.0, 0.0, 0.0};
	// double targetENUPosition[3] = {20000.0, 0.0, 3000.0};
	trajectory target = propagateTarget(targetENUPosition, targetENUVelocity);
	triple missile2Pip = pipSelection(Missile2, target, true);
	setArrayEquivalentToReference(Missile2.pip, missile2Pip.triplet);
	initSeeker(Missile1);
	sixDofFly(Missile2, "missile2", true, true, 400.0);

	auto wallClockEnd = chrono::high_resolution_clock::now();
	auto simRealRunTime = chrono::duration_cast<chrono::milliseconds>(wallClockEnd - wallClockStart);
	cout << "SIMULATION RUN TIME : " << (simRealRunTime.count() / 1000.0) << " SECONDS" << endl;
	// cout << "\n" << endl;
	return 0;

}