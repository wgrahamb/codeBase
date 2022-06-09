#include "iostream"
#include "iomanip"
#include "fstream"
#include "chrono"
#include "math.h"

#include "util.h"

using namespace std;

// GLOBALS
bool go; // FLAG
double VARIABLE_TIME_STEP; // SECONDS
const double CONSTANT_TIME_STEP = 0.001; // SECONDS
double maxSimulationTime; // SECONDS
double wayPoint[3]; // METERS
string lethality; // FLAG
double missileTimeOfFlight; // SECONDS
double missileLocalPosition[3]; // METERS
double forwardLeftUpMissileToInterceptPosition[3]; // METERS
double missDistance; // METERS
double missileYaw; // RADIANS
double missilePitch; // RADIANS
double missileLocalOrientation[3][3]; // NON DIMENSIONAL
double missileLocalVelocity[3]; // METERS PER SECOND
double missileBodyVelocity[3]; // METERS PER SECOND
double normalGuidanceCommand; // METERS PER S^2
double sideGuidanceCommand; // METERS PER S^2
double missileLocalAcceleration[3]; // METERS PER S^2
double missileBodyAcceleration[3]; // METERS PER S^2
ofstream outPut;

// FOR RK4 INTEGRATION
int PASS;
double P1[3], V1[3], A1[3];
double P2[3], V2[3], A2[3];
double P3[3], V3[3], A3[3];
double P4[3], V4[3], A4[3];

void init()
{

	ifstream inPut;
	inPut.open("input.txt");
	inPut >> wayPoint[0] >> wayPoint[1] >> wayPoint[2] >> missileLocalVelocity[0] >> missileLocalVelocity[1] >> missileLocalVelocity[2];

	go = true;
	VARIABLE_TIME_STEP = CONSTANT_TIME_STEP;
	maxSimulationTime = 100000.0;
	lethality = "FLYING";
	missileTimeOfFlight = 0.0;
	missileLocalPosition[0] = 0.0;
	missileLocalPosition[1] = 0.0;
	missileLocalPosition[2] = 0.0;
	azAndElFromVector(missileYaw, missilePitch, missileLocalVelocity);
	flightPathAnglesToLocalOrientation(missileYaw, -1 * missilePitch, missileLocalOrientation);
	double localRelativePosition[3];
	subtractTwoVectors(missileLocalPosition, wayPoint, localRelativePosition);
	threeByThreeTimesThreeByOne(missileLocalOrientation, localRelativePosition, forwardLeftUpMissileToInterceptPosition);
	threeByThreeTimesThreeByOne(missileLocalOrientation, missileLocalVelocity, missileBodyVelocity);
	normalGuidanceCommand = 0.0;
	sideGuidanceCommand = 0.0;
	missileLocalAcceleration[0] = 0.0;
	missileLocalAcceleration[1] = 0.0;
	missileLocalAcceleration[2] = 0.0;
	missileBodyAcceleration[0] = 0.0;
	missileBodyAcceleration[1] = 0.0;
	missileBodyAcceleration[2] = 0.0;
	magnitude(wayPoint, missDistance);

	outPut.open("output.txt");
	outPut << "TOF EAST NORTH UP EAST_V NORTH_V UP_V EAST_A NORTH_A UP_A" << endl;
	outPut << missileTimeOfFlight << " " << missileLocalPosition[0] << " " << missileLocalPosition[1] << " " << missileLocalPosition[2] << " " << missileLocalVelocity[0] << " " << missileLocalVelocity[1] << " " << missileLocalVelocity[2] << " " << missileLocalAcceleration[0] << " " << missileLocalAcceleration[1] << " " << missileLocalAcceleration[2] << endl;

}

void timeOfFlight()
{

	if (PASS == 0)
	{
		missileTimeOfFlight += CONSTANT_TIME_STEP;
	}
	

}

void guidance()
{

	double localRelativePosition[3];
	subtractTwoVectors(missileLocalPosition, wayPoint, localRelativePosition);
	threeByThreeTimesThreeByOne(missileLocalOrientation, localRelativePosition, forwardLeftUpMissileToInterceptPosition);
	threeByThreeTimesThreeByOne(missileLocalOrientation, missileLocalVelocity, missileBodyVelocity);
	double forwardLeftUpMissileToInterceptPositionUnitVector[3];
	unitVec(forwardLeftUpMissileToInterceptPosition, forwardLeftUpMissileToInterceptPositionUnitVector);
	double forwardLeftUpMissileToInterceptLineOfSightVel[3];
	vectorProjection(forwardLeftUpMissileToInterceptPositionUnitVector, missileBodyVelocity, forwardLeftUpMissileToInterceptLineOfSightVel);
	double timeToGo, forwardLeftUpMissileToInterceptPositionMagnitude, forwardLeftUpMissileToInterceptLineOfSightVelMagnitude;
	magnitude(forwardLeftUpMissileToInterceptPosition, forwardLeftUpMissileToInterceptPositionMagnitude);
	magnitude(forwardLeftUpMissileToInterceptLineOfSightVel, forwardLeftUpMissileToInterceptLineOfSightVelMagnitude);
	timeToGo = forwardLeftUpMissileToInterceptPositionMagnitude / forwardLeftUpMissileToInterceptLineOfSightVelMagnitude;
	if (timeToGo < 5)
	{
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
	else
	{
		double lineOfAttack[3];
		lineOfAttack[0] = forwardLeftUpMissileToInterceptPositionUnitVector[0] - 0.3;
		lineOfAttack[1] = forwardLeftUpMissileToInterceptPositionUnitVector[1] - 0.3;
		lineOfAttack[2] = -0.5;
		double forwardLeftUpMissileToInterceptLineOfAttackVel[3];
		vectorProjection(lineOfAttack, missileBodyVelocity, forwardLeftUpMissileToInterceptLineOfAttackVel);
		double G = 1 - exp(-0.001 * forwardLeftUpMissileToInterceptPositionMagnitude);
		normalGuidanceCommand = 1.5 * (forwardLeftUpMissileToInterceptLineOfSightVel[2] + G * forwardLeftUpMissileToInterceptLineOfAttackVel[2]);
		sideGuidanceCommand = 1.5 * (forwardLeftUpMissileToInterceptLineOfSightVel[1] + G * forwardLeftUpMissileToInterceptLineOfAttackVel[1]);
	}

}

void rk4Integrate()
{

	missileBodyAcceleration[0] = 0.0;
	missileBodyAcceleration[1] = sideGuidanceCommand;
	missileBodyAcceleration[2] = normalGuidanceCommand;
	oneByThreeTimesThreeByThree(missileBodyAcceleration, missileLocalOrientation, missileLocalAcceleration);

	if (PASS == 0)
	{

		P1[0] = missileLocalPosition[0];
		P1[1] = missileLocalPosition[1];
		P1[2] = missileLocalPosition[2];

		V1[0] = missileLocalVelocity[0];
		V1[1] = missileLocalVelocity[1];
		V1[2] = missileLocalVelocity[2];

		missileLocalPosition[0] = P1[0];
		missileLocalPosition[1] = P1[1];
		missileLocalPosition[2] = P1[2];

		missileLocalVelocity[0] = V1[0];
		missileLocalVelocity[1] = V1[1];
		missileLocalVelocity[2] = V1[2];

		PASS += 1;
		VARIABLE_TIME_STEP = CONSTANT_TIME_STEP / 2.0;

	}
	else if (PASS == 1)
	{

		A1[0] = missileLocalAcceleration[0];
		A1[1] = missileLocalAcceleration[1];
		A1[2] = missileLocalAcceleration[2];

		P2[0] = P1[0] + V1[0] * VARIABLE_TIME_STEP;
		P2[1] = P1[1] + V1[1] * VARIABLE_TIME_STEP;
		P2[2] = P1[2] + V1[2] * VARIABLE_TIME_STEP;

		V2[0] = V1[0] + A1[0] * VARIABLE_TIME_STEP;
		V2[1] = V1[1] + A1[1] * VARIABLE_TIME_STEP;
		V2[2] = V1[2] + A1[2] * VARIABLE_TIME_STEP;

		missileLocalPosition[0] = P2[0];
		missileLocalPosition[1] = P2[1];
		missileLocalPosition[2] = P2[2];

		missileLocalVelocity[0] = V2[0];
		missileLocalVelocity[1] = V2[1];
		missileLocalVelocity[2] = V2[2];

		PASS += 1;
		VARIABLE_TIME_STEP = CONSTANT_TIME_STEP / 2.0;

	}
	else if (PASS == 2)
	{

		A2[0] = missileLocalAcceleration[0];
		A2[1] = missileLocalAcceleration[1];
		A2[2] = missileLocalAcceleration[2];

		P3[0] = P1[0] + V2[0] * VARIABLE_TIME_STEP;
		P3[1] = P1[1] + V2[1] * VARIABLE_TIME_STEP;
		P3[2] = P1[2] + V2[2] * VARIABLE_TIME_STEP;

		V3[0] = V1[0] + A2[0] * VARIABLE_TIME_STEP;
		V3[1] = V1[1] + A2[1] * VARIABLE_TIME_STEP;
		V3[2] = V1[2] + A2[2] * VARIABLE_TIME_STEP;

		missileLocalPosition[0] = P3[0];
		missileLocalPosition[1] = P3[1];
		missileLocalPosition[2] = P3[2];

		missileLocalVelocity[0] = V3[0];
		missileLocalVelocity[1] = V3[1];
		missileLocalVelocity[2] = V3[2];

		PASS += 1;
		VARIABLE_TIME_STEP = CONSTANT_TIME_STEP;

	}
	else if (PASS == 3)
	{

		A3[0] = missileLocalAcceleration[0];
		A3[1] = missileLocalAcceleration[1];
		A3[2] = missileLocalAcceleration[2];

		P4[0] = P1[0] + V3[0] * VARIABLE_TIME_STEP;
		P4[1] = P1[1] + V3[1] * VARIABLE_TIME_STEP;
		P4[2] = P1[2] + V3[2] * VARIABLE_TIME_STEP;

		V4[0] = V1[0] + A3[0] * VARIABLE_TIME_STEP;
		V4[1] = V1[1] + A3[1] * VARIABLE_TIME_STEP;
		V4[2] = V1[2] + A3[2] * VARIABLE_TIME_STEP;

		missileLocalPosition[0] = P4[0];
		missileLocalPosition[1] = P4[1];
		missileLocalPosition[2] = P4[2];

		missileLocalVelocity[0] = V4[0];
		missileLocalVelocity[1] = V4[1];
		missileLocalVelocity[2] = V4[2];

		PASS += 1;
		VARIABLE_TIME_STEP = CONSTANT_TIME_STEP / 6.0;

	}
	else if (PASS == 4)
	{

		A4[0] = missileLocalAcceleration[0];
		A4[1] = missileLocalAcceleration[1];
		A4[2] = missileLocalAcceleration[2];

		missileLocalPosition[0] = P1[0] + (V1[0] + 2 * V2[0] + 2 * V3[0] + V4[0]) * VARIABLE_TIME_STEP;
		missileLocalPosition[1] = P1[1] + (V1[1] + 2 * V2[1] + 2 * V3[1] + V4[1]) * VARIABLE_TIME_STEP;
		missileLocalPosition[2] = P1[2] + (V1[2] + 2 * V2[2] + 2 * V3[2] + V4[2]) * VARIABLE_TIME_STEP;

		missileLocalVelocity[0] = V1[0] + (A1[0] + 2 * A2[0] + 2 * A3[0] + A4[0]) * VARIABLE_TIME_STEP;
		missileLocalVelocity[1] = V1[1] + (A1[1] + 2 * A2[1] + 2 * A3[1] + A4[1]) * VARIABLE_TIME_STEP;
		missileLocalVelocity[2] = V1[2] + (A1[2] + 2 * A2[2] + 2 * A3[2] + A4[2]) * VARIABLE_TIME_STEP;

		// Reset variables.
		PASS = 0;
		VARIABLE_TIME_STEP = CONSTANT_TIME_STEP;
		P1[0] = 0.0;
		P1[1] = 0.0;
		P1[2] = 0.0;
		V1[0] = 0.0;
		V1[1] = 0.0;
		V1[2] = 0.0;
		A1[0] = 0.0;
		A1[1] = 0.0;
		A1[2] = 0.0;
		P2[0] = 0.0;
		P2[1] = 0.0;
		P2[2] = 0.0;
		V2[0] = 0.0;
		V2[1] = 0.0;
		V2[2] = 0.0;
		A2[0] = 0.0;
		A2[1] = 0.0;
		A2[2] = 0.0;
		P3[0] = 0.0;
		P3[1] = 0.0;
		P3[2] = 0.0;
		V3[0] = 0.0;
		V3[1] = 0.0;
		V3[2] = 0.0;
		A3[0] = 0.0;
		A3[1] = 0.0;
		A3[2] = 0.0;
		P4[0] = 0.0;
		P4[1] = 0.0;
		P4[2] = 0.0;
		V4[0] = 0.0;
		V4[1] = 0.0;
		V4[2] = 0.0;
		A4[0] = 0.0;
		A4[1] = 0.0;
		A4[2] = 0.0;

	}

}

void orientation()
{

	azAndElFromVector(missileYaw, missilePitch, missileLocalVelocity);
	flightPathAnglesToLocalOrientation(missileYaw, -1 * missilePitch, missileLocalOrientation);

}

void performance()
{
	magnitude(forwardLeftUpMissileToInterceptPosition, missDistance);
}

void logData()
{
	if (PASS == 0)
	{
		outPut << missileTimeOfFlight << " " << missileLocalPosition[0] << " " << missileLocalPosition[1] << " " << missileLocalPosition[2] << " " << missileLocalVelocity[0] << " " << missileLocalVelocity[1] << " " << missileLocalVelocity[2] << " " << missileLocalAcceleration[0] << " " << missileLocalAcceleration[1] << " " << missileLocalAcceleration[2] << endl;
	}
	
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
	rk4Integrate();
	orientation();
	performance();
	logData();
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
			if (PASS == 0)
			{
				cout << setprecision(10) << "STATUS AT TIME OF FLIGHT " << missileTimeOfFlight << " X " << missileLocalPosition[0] << " Y " << missileLocalPosition[1] << " Z " << missileLocalPosition[2] << endl;
			}
			lastTime = missileTimeOfFlight;
		}
	}

	// CONSOLE REPORT
	cout << "\n" << endl;
	cout << "MISSION REPORT" << endl;
	cout << setprecision(10) << "FINAL STATUS AT TIME OF FLIGHT " << missileTimeOfFlight << " X " << missileLocalPosition[0] << " Y " << missileLocalPosition[1] << " Z " << missileLocalPosition[2] << endl;
	cout << setprecision(10) << "MISS DISTANCE " << missDistance << " >>> FORWARD, LEFT, UP MISS DISTANCE " << forwardLeftUpMissileToInterceptPosition[0] << " " << forwardLeftUpMissileToInterceptPosition[1] << " " << forwardLeftUpMissileToInterceptPosition[2] << endl;
	cout << "SIMULATION RESULT: " << lethality << endl;
	auto wallClockEnd = chrono::high_resolution_clock::now();
	auto simRealRunTime = chrono::duration_cast<chrono::milliseconds>(wallClockEnd - wallClockStart);
	cout << "SIMULATION RUN TIME :" << simRealRunTime.count() / 1000.0 << " SECONDS" << endl;
	cout << "\n" << endl;

	// TERMINATE PROGRAM
	return 0;

}