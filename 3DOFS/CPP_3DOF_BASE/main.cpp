#include "iostream"
#include "iomanip"
#include "chrono"
#include "math.h"

#include "util.h"

using namespace std;

// GLOBALS
bool go; // FLAG
double timeOfFlightStep; // SECONDS
double maxSimulationTime; // SECONDS
double wayPoint[3]; // METERS
string lethality; // FLAG
double missileTimeOfFlight; // SECONDS
double missileLocalPosition[3]; // METERS
double missileRange; // METERS
double forwardLeftUpMissileToInterceptPosition[3]; // METERS
double missDistance; // METERS
double missileYaw; // RADIANS
double missilePitch; // RADIANS
double missileLocalOrientation[3][3]; // NON DIMENSIONAL
double missileVelocity[3]; // METERS PER SECOND
double missileBodyVelocity[3]; // METERS PER SECOND
double normalGuidanceCommand; // METERS PER S^2
double sideGuidanceCommand; // METERS PER S^2
double missileAcceleration[3]; // METERS PER S^2
double missileBodyAcceleration[3]; // METERS PER S^2

void init()
{

	go = true;
	timeOfFlightStep = 0.001;
	maxSimulationTime = 100000.0;
	wayPoint[0] = 5000000.0;
	wayPoint[1] = 5000000.0;
	wayPoint[2] = 5000000.0; // THIS IS THE TARGET
	lethality = "FLYING";
	missileTimeOfFlight = 0.0;
	missileLocalPosition[0] = 0.0;
	missileLocalPosition[1] = 0.0;
	missileLocalPosition[2] = 0.0;
	missileRange = 0.0;
	missileVelocity[0] = unituni() * 200.0;
	missileVelocity[1] = unituni() * 200.0;
	missileVelocity[2] = unituni() * 200.0; // THIS IS THE MISSILES SPEED, IT RESPONDS TO GUIDANCE COMMANDS, AND ENCOUNTERS NO RESISTANCE. IT WILL HIT.
	azAndElFromVector(missileYaw, missilePitch, missileVelocity);
	flightPathAnglesToLocalOrientation(missileYaw, -1 * missilePitch, missileLocalOrientation);
	double localRelativePosition[3];
	subtractTwoVectors(missileLocalPosition, wayPoint, localRelativePosition);
	threeByThreeTimesThreeByOne(missileLocalOrientation, localRelativePosition, forwardLeftUpMissileToInterceptPosition);
	threeByThreeTimesThreeByOne(missileLocalOrientation, missileVelocity, missileBodyVelocity);
	normalGuidanceCommand = 0.0;
	sideGuidanceCommand = 0.0;
	missileAcceleration[0] = 0.0;
	missileAcceleration[1] = 0.0;
	missileAcceleration[2] = 0.0;
	missileBodyAcceleration[0] = 0.0;
	missileBodyAcceleration[1] = 0.0;
	missileBodyAcceleration[2] = 0.0;
	magnitude(wayPoint, missDistance);

}

void timeOfFlight()
{

	missileTimeOfFlight += timeOfFlightStep;

}

void guidance()
{

	double localRelativePosition[3];
	subtractTwoVectors(missileLocalPosition, wayPoint, localRelativePosition);
	threeByThreeTimesThreeByOne(missileLocalOrientation, localRelativePosition, forwardLeftUpMissileToInterceptPosition);
	double forwardLeftUpMissileToInterceptPositionUnitVector[3];
	unitVec(forwardLeftUpMissileToInterceptPosition, forwardLeftUpMissileToInterceptPositionUnitVector);
	double closingVelocity[3];
	multiplyVectorTimesScalar(-1.0, missileBodyVelocity, closingVelocity);
	double closingSpeed;
	magnitude(closingVelocity, closingSpeed);
	double TEMP1[3], TEMP2;
	crossProductTwoVectors(forwardLeftUpMissileToInterceptPosition, closingVelocity, TEMP1);
	dotProductTwoVectors(forwardLeftUpMissileToInterceptPosition, forwardLeftUpMissileToInterceptPosition, TEMP2);
	double lineOfSightRate[3];
	divideVectorByScalar(TEMP2, TEMP1, lineOfSightRate);
	double TEMP3, TEMP4[3];
	double proportionalGuidanceGain = 3.0;
	TEMP3 = -1 * proportionalGuidanceGain * closingSpeed;
	multiplyVectorTimesScalar(TEMP3, forwardLeftUpMissileToInterceptPositionUnitVector, TEMP4);
	double COMMAND[3];
	crossProductTwoVectors(TEMP4, lineOfSightRate, COMMAND);
	normalGuidanceCommand = COMMAND[2];
	sideGuidanceCommand = COMMAND[1];

}

void integrate()
{

	missileBodyAcceleration[0] = 0.0;
	missileBodyAcceleration[1] = sideGuidanceCommand;
	missileBodyAcceleration[2] = normalGuidanceCommand;
	oneByThreeTimesThreeByThree(missileBodyAcceleration, missileLocalOrientation, missileAcceleration);

	double deltaVelocity[3];
	multiplyVectorTimesScalar(timeOfFlightStep, missileAcceleration, deltaVelocity);
	double newMissileVelocity[3];
	addTwoVectors(missileVelocity, deltaVelocity, newMissileVelocity);
	missileVelocity[0] = newMissileVelocity[0];
	missileVelocity[1] = newMissileVelocity[1];
	missileVelocity[2] = newMissileVelocity[2];

	double deltaPosition[3];
	multiplyVectorTimesScalar(timeOfFlightStep, missileVelocity, deltaPosition);
	double newMissilePosition[3];
	addTwoVectors(missileLocalPosition, deltaPosition, newMissilePosition);
	missileLocalPosition[0] = newMissilePosition[0];
	missileLocalPosition[1] = newMissilePosition[1];
	missileLocalPosition[2] = newMissilePosition[2];

	double distanceTravelled;
	magnitude(deltaPosition, distanceTravelled);
	missileRange += distanceTravelled;

}

void orientation()
{

	azAndElFromVector(missileYaw, missilePitch, missileVelocity);
	flightPathAnglesToLocalOrientation(missileYaw, -1 * missilePitch, missileLocalOrientation);

}

void performance()
{
	magnitude(forwardLeftUpMissileToInterceptPosition, missDistance);
}

void endCheck()
{

	if (missileLocalPosition[2] < 0.0)
	{
		lethality = "GROUND COLLISION";
		go = false;
	}
	else if (missDistance < 5.0)
	{
		lethality = "SUCCESSFUL INTERCEPT";
		go = false;
	}
	else if (forwardLeftUpMissileToInterceptPosition[0] < 0.0)
	{
		lethality = "POINT OF CLOSEST APPROACH PASSED";
		go = false;
	}
	else if (isnan(missileLocalPosition[0]))
	{
		lethality = "NOT A NUMBER";
		go = false;
	}
	else if (missileTimeOfFlight > maxSimulationTime)
	{
		lethality = "MAXIMUM SIMULATION TIME EXCEEDED";
		go = false;
	}

}

void fly()
{
	timeOfFlight();
	guidance();
	integrate();
	orientation();
	performance();
	endCheck();
}

int main()
{

	// START WALL CLOCK
	auto wallClockStart = chrono::high_resolution_clock::now();

	// FORMAT CONSOLE OUTPUT
	cout << "\n" << endl;

	// INITIATE FUNCTION
	init();

	// RUN SIMULATION
	double lastTime = 0;
	while (go)
	{
		fly();
		auto print_it = static_cast<int>(round(missileTimeOfFlight * 10000.0)) % 10000;
		if (print_it == 0)
		{
			cout << setprecision(10) << "STATUS AT TIME OF FLIGHT " << missileTimeOfFlight << " X " << missileLocalPosition[0] << " Y " << missileLocalPosition[1] << " Z " << missileLocalPosition[2] << " RANGE " << missileRange << endl;
			lastTime = missileTimeOfFlight;
		}
	}

	// CONSOLE REPORT
	cout << "\n" << endl;
	cout << "MISSION REPORT" << endl;
	cout << setprecision(10) << "FINAL STATUS AT TIME OF FLIGHT " << missileTimeOfFlight << " X " << missileLocalPosition[0] << " Y " << missileLocalPosition[1] << " Z " << missileLocalPosition[2] << " RANGE " << missileRange << endl;
	cout << setprecision(10) << "MISS DISTANCE " << missDistance << " >>> FORWARD, LEFT, UP MISS DISTANCE " << forwardLeftUpMissileToInterceptPosition[0] << " " << forwardLeftUpMissileToInterceptPosition[1] << " " << forwardLeftUpMissileToInterceptPosition[2] << endl;
	cout << "SIMULATION RESULT: " << lethality << endl;
	auto wallClockEnd = chrono::high_resolution_clock::now();
	auto simRealRunTime = chrono::duration_cast<chrono::milliseconds>(wallClockEnd - wallClockStart);
	cout << "SIMULATION RUN TIME :" << simRealRunTime.count() / 1000.0 << " SECONDS" << endl;
	cout << "\n" << endl;

	// TERMINATE PROGRAM
	return 0;

}