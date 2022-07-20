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
#include <random>

// Utility.
#include "util.h"

// Missile model.
#include "missileModel.h"

// Namespace.
using namespace std;

// Start wall clock.
auto wallClockStart = chrono::high_resolution_clock::now(); // Start tracking real time.

/* To do */
// For missile model, needs a guidance law for pip and for moving target. Also a "seeker on" mode.
// Start a simple simulation loop.
// Structs for NOVICE players.
// Scene generator.
// Scene loader.

/* Players. */
struct Asset
{

	string identity;
	double ENUPosition[3];
	double simulationTime;
	string status;

};

struct Launcher
{

	string identity;
	double ENUPosition[3];
	double simulationTime;
	string status;

	double launcherAzimuth;
	double launcherElevation;
	vector<Missile> inventory;

};

struct Soldier
{

	string identity;
	double ENUPosition[3];
	double simulationTime;
	string status;

	Missile missile;

};

struct Threat
{

	string identity;
	double ENUPosition[3];
	double simulationTime;
	string status;

	double ENUVelocity[3];

};

struct NOVICE
{

	vector<Asset> Assets;
	vector<Launcher> Launchers;
	vector<Soldier> Soldiers;
	vector<Threat> Threats;

};

// Simulation control.
const double CONSTANT_TIME_STEP = 1 / 500.0; // Seconds. Good to have a uniform time step if can.
double SIM_TIME = 0.0;
random_device generator;
mt19937 rng(generator());
uniform_int_distribution<int> assetAndLauncherDistro(0, 5000);
uniform_real_distribution<double> azimuthDistro((-90.0 * degToRad), (90.0 * degToRad));
uniform_real_distribution<double> elevationDistro(0.0, (89.0 * degToRad));
uniform_int_distribution<int> threatNorthDistro(0, 20000);
uniform_int_distribution<int> threatEastAndUpDistro(15000, 20000);
uniform_int_distribution<int> threatVelDistro(-1000, -500);

NOVICE generate_scene(int numberOfAssets, int numberOfInterceptors, int numberOfThreats)
{

	NOVICE NOVICE_PLAYERS;
	int maxNumberOfInterceptorsPerLauncher = 15;
	int interceptors = numberOfInterceptors;

	for (int i = 0; i < numberOfAssets; i++)
	{

		Asset asset;
		asset.identity = "asset_" + to_string(i);
		asset.ENUPosition[0] = assetAndLauncherDistro(rng);
		asset.ENUPosition[1] = assetAndLauncherDistro(rng);
		asset.ENUPosition[2] = 0.0;
		consolePrintArray(asset.identity ,asset.ENUPosition);
		asset.simulationTime = SIM_TIME;
		asset.status = "ALIVE";
		NOVICE_PLAYERS.Assets.push_back(asset);

	}
	int numberOfLaunchers = ceil(numberOfInterceptors / maxNumberOfInterceptorsPerLauncher);
	for (int i = 0; i < numberOfLaunchers; i++)
	{

		Launcher launcher;
		launcher.identity = "launcher_" + to_string(i);
		launcher.ENUPosition[0] = assetAndLauncherDistro(rng);
		launcher.ENUPosition[1] = assetAndLauncherDistro(rng);
		launcher.ENUPosition[2] = 0.0;
		consolePrintArray(launcher.identity ,launcher.ENUPosition);
		launcher.simulationTime = SIM_TIME;
		launcher.status = "ALIVE";

		launcher.launcherAzimuth =  azimuthDistro(rng);
		launcher.launcherElevation = elevationDistro(rng);

		Missile missileOriginal;
		initUnLaunchedMissile(missileOriginal, 0.0, launcher.launcherElevation, launcher.launcherAzimuth, launcher.ENUPosition);

		int ii = -1;
		while (interceptors > 0)
		{

			ii += 1;
			interceptors -= 1;
			Missile missileCopy = missileOriginal;
			launcher.inventory.push_back(missileCopy);
			if (ii > (maxNumberOfInterceptorsPerLauncher - 2))
			{
				break;
			}

		}

		NOVICE_PLAYERS.Launchers.push_back(launcher);

	}

	return NOVICE_PLAYERS;

}

// Trajectory propagatePositionAndVelocity(double ENUPosition[3], double ENUVelocity[3])
// {

// 	Trajectory retTrajectory; // 0 Place is time of flight. 1, 2, and 3 place is the target position.

// 	Triple position(ENUPosition);
// 	retTrajectory.push_back(make_pair(0.0, position));

// 	for (int i = 0; i < 100000; i++)
// 	{

// 		double time = CONSTANT_TIME_STEP * i;
// 		double deltaPos[3];
// 		multiplyVectorTimesScalar(CONSTANT_TIME_STEP, ENUVelocity, deltaPos);
// 		addTwoVectors(position.array, deltaPos, position.array);
// 		retTrajectory.push_back(make_pair(time, position));
// 		if (position.array[2] < 0.0) // Below ground.
// 		{
// 			break;
// 		}

// 	}

// 	return retTrajectory;

// }

// Pip selection algorithm needs work.
// Launcher is an input, to account for a slewing launcher.

/*Criteria for a good shot*/
// Pre flight check.
// Target must be a threat to an asset. Say propagation must be within 1000 meters of an asset.
// Range between 3000 meters and 6000 meters. This allows the missile some maneuvering room.
// Shot must be forward firing, AKA FLUMtoPip[0] should be > 0.
// slant must be within plus 40 degrees, minus 40 degrees of azimuth line of sight
// slant must be within plus 20 degrees, minus 0 degrees of elevation line of sight.

// Post flight check
// Successful intercept.
// Missile MUST arrive at pip before the target does. Even if by seconds, the launch will be scheduled.

// Triple pipSelection(Missile &missile, Trajectory targetTrajectory, bool showProcess)
// {

// 	double temp[3] = {0.0, 0.0, 0.0};
// 	Triple ret(temp);
// 	return ret;

// }

int main ()
{

	int nAssets = 1;
	int nInterceptors  = 15;
	int nThreats = 1;
	NOVICE sceneTest = generate_scene(nAssets, nInterceptors, nThreats);

	Missile msl;
	double initPos[3] = {0.0, 0.0, 0.0};
	initUnLaunchedMissile(msl, 0.0, (45 * degToRad), (45.0 * degToRad), initPos);
	msl.pip[0] = 3000.0;
	msl.pip[1] = 1000.0;
	msl.pip[2] = 3000.0;
	initSeeker(msl);
	msl.lethality = "FLYING";
	sixDofFly(msl, "test", true, true, 200.0);

	auto wallClockEnd = chrono::high_resolution_clock::now();
	auto simRealRunTime = chrono::duration_cast<chrono::milliseconds>(wallClockEnd - wallClockStart);
	cout << "SIMULATION RUN TIME : " << (simRealRunTime.count() / 1000.0) << " SECONDS" << endl;
	return 0;

}