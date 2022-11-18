
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
#include <memory>

// Utility.
#include "util.h"

// Missile model.
#include "missileModel.h"
#include "secondOrderActuator.h"
#include "ATM1976.h"

// Namespace.
using namespace std;

/* Missile Model */
/*
#
# Author - Wilson Graham Beech.
# Reference - Modeling and Simulation of Aerospace Vehicle Dynamics,
# Second Edition - Peter H. Zipfel.
#
# ENU = East, North, Up Coordinate System.
# FLU = Forward, Left, Up Coordinate System.
#
# Interceptor Orientation.
# Array 0, Axis - Looking down the nozzle of the interceptor.
# Array 1, Side - Looking down the nozzle of the interceptor, this points out the left hand side.
# Array 2, Normal - Looking down the nozzle of the interceptor, this points out the top side.
#
#                                  Positive normal.
#                                     |
#                                     |
#                                     |
#               Positive side. -------O------- Negative side.
#                                     |
#                                     |
#                                     |
#                                  Negative normal.
#
# Negative axis is pointing out of the screen directly at you.
# Positive axis is pointing into the screen directly away from you.
#
# Positive alpha indicates nose below free stream velocity.
# Positive beta indicates nose left of free stream velocity.
# Positive roll indicates normal axis clockwisely rotated from twelve o'clock.
#
# Fin orientation, looking down the nozzle of the missile.
#
#                    Fin 4    Fin 1
#                           X
#                    Fin 3    Fin 2
#
*/

// GB.
// Parses text file with missile model tables. Should only be called once.
void formatTables (Missile &missile, string dataFile)
{
	// LOOK UP DATA
	ifstream inFile(dataFile);
	// STRING OF FILE LINE
	string line;
	// TABLE NUMBER
	int tableNoTrack = 0;
	// ROW NUMBER
	int rowNoTrack = 0;
	// VECTOR TO STORE TABLE DIMENSIONS
	vector<vector<int>> dimensions;
	// LOOP
	while(getline(inFile, line))
	{
		// FLAG FOR INDICATION OF LINE CLASSIFICATION >>> 1 = ONE DIMENSIONAL TABLE SIZE; TWO = TWO DIMENSIONAL TABLE SIZE; THREE = TABLE NAME
		int flag = 0;
		// INITIALIZE NAME OF TABLE
		string name;
		// INITIALIZE DIMENSION OF SPECIFIC TABLE
		vector<int> dimension;
		// FIND TABLE NAME
		if (line.substr(0, 4) == "NAME")
		{
			// RE INIT ROW NUMBER TRACKER
			rowNoTrack = 0;
			// STORE NAME OF TABLE
			name = line.substr(5, line.size() - 6);
			// TRACK TABLE NUMBER
			tableNoTrack += 1;
			// MARK FLAG FOR LATER USE
			flag = 3;
		}
		// FIND TABLE DIMENSION
		else if (line.substr(0, 2) == "NX")
		{
			// MARK FLAG FOR LATER USE
			flag = 1;
			// STORE "ROWS" DIMENSION
			int D1 = stoi(line.substr(4, 3));
			// STORE "ROWS" DIMENSIONS IN VECTOR
			dimension.push_back(D1);
			// INITIALIZE "COLUMNS" DIMENSION
			int D2 = 0;
			// CHECK FOR A DETERMINED "COLUMNS" DIMENSION
			for (int i = 3; i < line.size(); i++) {
				// CHECK
				if (line.substr(i, 2) == "NX")
				{
					// MARK FLAG FOR LATER USE
					flag = 2;
					// ADD ONE TO ROWS DIMENSION SINCE THIS IS A TWO DIMENSIONAL TABLE
					dimension[0] += 1;
					// STORE "COLUMNS" DIMENSION
					D2 = stoi(line.substr(i+4, 3)) + 1;
					// STORE "COLUMNS" DIMENSION IN VECTOR
					dimension.push_back(D2);
				}
			}
			// IF NO DETERMINED SECOND DIMENSION
			if (D2 == 0)
			{
				// "COLUMNS" DIMENSION BECOMES TWO
				D2 = 2;
				// STORE "COLUMNS" DIMENSION IN VECTOR
				dimension.push_back(D2);
			}
		}
		// NOTHING FLAGGED, NEXT ITERATION
		if (flag == 0)
		{
			// ONLY CHECK IF A TABLE HAS BEEN INITIALIZED
			if (dimensions.size() > 0)
			{
				// COUNT ROW NUMBER
				rowNoTrack += 1;
				// PARSE LINE THROUGH A STREAM
				istringstream parseLine(line);
				// INITIALIZE COLUMN COUNTER
				int columnCount = 0;
				// LOOP THROUGH ONE ROW, ALL COLUMNS
				do
				{
					// ITERATE COLUMN COUNTER
					columnCount += 1;
					// INITIALIZE DATA POINT
					string dataPoint;
					// GRAB DATA POINT FROM PARSES
					parseLine >> dataPoint;
					// CHECK TO MAKE SURE IT IS NOT WHITESPACE
					if (dataPoint.find_first_not_of(' ') != std::string::npos)
					{
						// CONVERT STRING TO DOUBLE
						double dataPointDouble = stod(dataPoint);
						/////////// FOR THIS SPECIFIC SET OF DATA, CHECK FOR 90. THERE ARE 14 ROWS AND 15 COLUMNS FOR THE TWO DIMENSIONAL TABLES WHICH MEANS THIS IS A SPECIFIC PIECE OF CODE. WOULD HAVE TO BE ALTERED FOR DIFFERING DATA SETS.
						if (dataPointDouble == 90)
						{
							// PLACE IT AT THE FAR RIGHT CORNER
							missile.tables[tableNoTrack - 1][0].back() = dataPointDouble;
						}
						// IF THIS THE FIRST LOOP, THIS IS THE COLUMN IN THE DATA SET THAT DISPLAYS THE "ROWS" VALUES
						else if (columnCount == 1)
						{
							// FOR TWO DIMENSIONAL TABLE
							if (dimensions[tableNoTrack -1][1] != 2)
							{
								// PLACE DATA POINT IN ITS PLACE
								missile.tables[tableNoTrack - 1][rowNoTrack][0] = dataPointDouble;
							}
							// FOR ONE DIMENSIONAL TABLE
							else
							{
								// PLACE DATA POINT IN ITS PLACE
								missile.tables[tableNoTrack - 1][rowNoTrack - 1][0] = dataPointDouble;
							}
						}
						// IF THIS THE SECOND LOOP, THIS IS THE COLUMN IN THE DATA SET THAT DISPLAYS THE "COLUMNS" VALUES, ONLY FOR TWO DIMENSIONAL TABLES
						else if (columnCount == 2 and dimensions[tableNoTrack -1][1] != 2)
						{
							// PLACE DATA POINT IN ITS PLACE
							missile.tables[tableNoTrack - 1][0][rowNoTrack] = dataPointDouble;
						}
						// ELSE FOR ACTUAL DATA POINTS
						else
						{
							// FOR TWO DIMENSIONAL TABLES
							if (dimensions[tableNoTrack -1][1] != 2)
							{
								// PLACE DATA POINT IN ITS PLACE
								missile.tables[tableNoTrack - 1][rowNoTrack][columnCount - 2] = dataPointDouble;
							}
							// FOR ONE DIMENSIONAL TABLES
							else
							{
								// PLACE DATA POINT IN ITS PLACE
								missile.tables[tableNoTrack - 1][rowNoTrack - 1][columnCount - 1] = dataPointDouble;
							}
						}
					}
				} while (parseLine);
			}
		}
		// CREATE A TABLE OF CORRECT SIZE AND STORE IT
		else if (flag == 1 or flag == 2)
		{
			// STORE VECTOR OF DIMENSIONS
			dimensions.push_back(dimension);
			// SEPERATE ROW DIMENSION
			int rows = dimension[0];
			// SEPERATE COLUMN DIMENSION
			int columns = dimension[1];
			// CREATE TABLE
			vector<vector<double>> newTable(rows, vector<double>(columns));
			// TOP LEFT CORNER OF TABLE UNUSED
			newTable[0][0] = 0.0;
			// STORE NEW TABLE IN VECTOR
			missile.tables.push_back(newTable);
		}
		// STORE NAME OF TABLE
		else if (flag == 3)
		{
			// MAP TABLE NAME INDEX PAIR
			missile.tableNameIndexPairs.emplace(name, tableNoTrack - 1);
		}
	}
}

Missile clone(const Missile &missile)
{

	Missile ret;
	ret = missile;
	ret.FIN1 = nullptr;
	ret.FIN2 = nullptr;
	ret.FIN3 = nullptr;
	ret.FIN4 = nullptr;
	ret.FIN1 = make_shared<secondOrderActuator>(*missile.FIN1);
	ret.FIN2 = make_shared<secondOrderActuator>(*missile.FIN2);
	ret.FIN3 = make_shared<secondOrderActuator>(*missile.FIN3);
	ret.FIN4 = make_shared<secondOrderActuator>(*missile.FIN4);
	return ret;

}

// Emplacement.
void emplace(Missile &missile, double phiRads, double thetaRads, double psiRads, double ENUPosition[3])
{

	// Missile.
	missile.enuAttitude[0] = phiRads;
	missile.enuAttitude[1] = thetaRads;
	missile.enuAttitude[2] = psiRads;
	eulerAnglesToLocalOrientation(phiRads, -thetaRads, psiRads, missile.enuToFlu);
	missile.enuPos[0] = ENUPosition[0];
	missile.enuPos[1] = ENUPosition[1];
	missile.enuPos[2] = ENUPosition[2];
	missile.enuVel[0] = missile.enuToFlu[0][0];
	missile.enuVel[1] = missile.enuToFlu[0][1];
	missile.enuVel[2] = missile.enuToFlu[0][2];
	threeByThreeTimesThreeByOne(missile.enuToFlu, missile.enuVel, missile.fluVel);
	missile.enuAcc[0] = 0.0;
	missile.enuAcc[1] = 0.0;
	missile.enuAcc[2] = 0.0;
	missile.spcfForce[0] = 0.0;
	missile.spcfForce[1] = 0.0;
	missile.spcfForce[2] = 0.0;
	magnitude(missile.enuVel, missile.spd);

	// Set missile lethality.
	missile.lethality = "LOITERING"; // STATUS

}

// For the case of new flyouts as well as "seeker on." Must have a pip or target state to initialize.
void seekerOn(Missile &missile)
{

	// Intialize seeker.
	double relPos[3];
	subtractTwoVectors(missile.enuPos, missile.waypoint, relPos);
	double relPosU[3];
	unitVec(relPos, relPosU);
	double mslToInterceptU[3];
	threeByThreeTimesThreeByOne(missile.enuToFlu, relPosU, mslToInterceptU);
	double mslToInterceptAz, mslToInterceptEl;
	azAndElFromVector(mslToInterceptAz, mslToInterceptEl, mslToInterceptU);
	missile.skrThtErr = mslToInterceptEl;
	missile.skrPsiErr = mslToInterceptAz;
	missile.skrTht = 0.0;
	missile.skrPsi = 0.0;
	missile.skrWlr = missile.skrPsi;
	missile.skrWlq = missile.skrTht;

}

// Defines the atmosphere around the missile.
void atmosphere(Missile &missile)
{

	magnitude(missile.enuVel, missile.spd);
	auto ATM = atm1976_metric::update(missile.enuPos[2], missile.spd);
	missile.grav = ATM.g;
	missile.p = ATM.p;
	missile.q = ATM.q;
	missile.mach = ATM.mach;

}

void seeker(Missile &missile)
{

	if (!missile.isBallistic)
	{

		double wsq = SEEKER_KF_WN * SEEKER_KF_WN;
		double gg = SEEKER_KF_G * wsq;

		// Yaw channel.
		double wlr1d_new = missile.skrWlr2;
		double wlr1_new = trapezoidIntegrate(wlr1d_new, missile.skrWlr1Dot, missile.skrWlr1, missile.timeStep);
		missile.skrWlr1 = wlr1_new;
		missile.skrWlr1Dot = wlr1d_new;
		double wlr2d_new = gg * missile.skrPsiErr - 2 * SEEKER_KF_ZETA * SEEKER_KF_WN * missile.skrWlr1Dot - wsq * missile.skrWlr1;
		double wlr2_new = trapezoidIntegrate(wlr2d_new, missile.skrWlr2Dot, missile.skrWlr2, missile.timeStep);
		missile.skrWlr2 = wlr2_new;
		missile.skrWlr2Dot = wlr2d_new;

		// Yaw control.
		double wlrd_new = missile.skrWlr1 - missile.rate[2];
		double wlr_new = trapezoidIntegrate(wlrd_new, missile.skrWlrDot, missile.skrWlr, missile.timeStep);
		missile.skrWlr = wlr_new;
		missile.skrWlrDot = wlrd_new;
		missile.skrPsi = missile.skrWlr;

		// Pitch channel.
		double wlq1d_new = missile.skrWlq2;
		double wlq1_new = trapezoidIntegrate(wlq1d_new, missile.skrWlq1Dot, missile.skrWlq1, missile.timeStep);
		missile.skrWlq1 = wlq1_new;
		missile.skrWlq1Dot = wlq1d_new;
		double wlq2d_new = gg * missile.skrThtErr - 2 * SEEKER_KF_ZETA * SEEKER_KF_WN * missile.skrWlq1Dot - wsq * missile.skrWlq1;
		double wlq2_new = trapezoidIntegrate(wlq2d_new, missile.skrWlq2Dot, missile.skrWlq2, missile.timeStep);
		missile.skrWlq2 = wlq2_new;
		missile.skrWlq2Dot = wlq2d_new;

		// Pitch control.
		double wlqd_new = missile.skrWlq1 - missile.rate[1];
		double wlq_new = trapezoidIntegrate(wlqd_new, missile.skrWlqDot, missile.skrWlq, missile.timeStep);
		missile.skrWlq = wlq_new;
		missile.skrWlqDot = wlqd_new;
		missile.skrTht = missile.skrWlq;

		// Convert seeker data to FLU relative position for guidance.
		double localRelPos[3];
		subtractTwoVectors(missile.enuPos, missile.waypoint, localRelPos);
		double seekerAttitudeToLocalTM[3][3];
		eulerAnglesToLocalOrientation(0.0, -missile.skrTht, missile.skrPsi, seekerAttitudeToLocalTM);
		threeByThreeTimesThreeByThree(seekerAttitudeToLocalTM, missile.enuToFlu, missile.skrEnuToFlu);
		double seekerToInterceptRelPos[3];
		threeByThreeTimesThreeByOne(missile.skrEnuToFlu, localRelPos, seekerToInterceptRelPos);
		double inducedErr[3] = {1.0, 0.5, 0.2};
		double seekerToInterceptRelPosWithErr[3];
		multiplyTwoVectors(seekerToInterceptRelPos, inducedErr, seekerToInterceptRelPosWithErr);
		azAndElFromVector(missile.skrPsiErr, missile.skrThtErr, seekerToInterceptRelPosWithErr);
		oneByThreeTimesThreeByThree(seekerToInterceptRelPosWithErr, seekerAttitudeToLocalTM, missile.mslToWaypoint);

	}
	else
	{

		double relPos[3];
		subtractTwoVectors(missile.enuPos, missile.waypoint, relPos);
		double mslToIntercept[3];
		threeByThreeTimesThreeByOne(missile.enuToFlu, relPos, mslToIntercept);
		setArrayEquivalentToReference(missile.mslToWaypoint, mslToIntercept);

	}

}

void guidance(Missile &missile)
{

	if (!missile.isBallistic)
	{

		double forwardLeftUpMissileToInterceptPositionUnitVector[3];
		unitVec(missile.mslToWaypoint, forwardLeftUpMissileToInterceptPositionUnitVector);
		double forwardLeftUpMissileToInterceptLineOfSightVel[3];
		vectorProjection(forwardLeftUpMissileToInterceptPositionUnitVector, missile.fluVel, forwardLeftUpMissileToInterceptLineOfSightVel);
		double timeToGo, forwardLeftUpMissileToInterceptPositionMagnitude, forwardLeftUpMissileToInterceptLineOfSightVelMagnitude;
		magnitude(missile.mslToWaypoint, forwardLeftUpMissileToInterceptPositionMagnitude);
		magnitude(forwardLeftUpMissileToInterceptLineOfSightVel, forwardLeftUpMissileToInterceptLineOfSightVelMagnitude);
		missile.timeToGo = forwardLeftUpMissileToInterceptPositionMagnitude / forwardLeftUpMissileToInterceptLineOfSightVelMagnitude;
		// if (missile.timeToGo < 5)
		if (true)
		{
			if (!missile.isHoming)
			{
				missile.isHoming = true;
			}
			double closingVelocity[3];
			multiplyVectorTimesScalar(-1.0, missile.fluVel, closingVelocity);
			double closingSpeed;
			magnitude(closingVelocity, closingSpeed);
			double TEMP1[3], TEMP2;
			crossProductTwoVectors(missile.mslToWaypoint, closingVelocity, TEMP1);
			dotProductTwoVectors(missile.mslToWaypoint, missile.mslToWaypoint, TEMP2);
			double lineOfSightRate[3];
			divideVectorByScalar(TEMP2, TEMP1, lineOfSightRate);
			double TEMP3, TEMP4[3];
			TEMP3 = -1 * PROPORTIONAL_GUIDANCE_GAIN * closingSpeed;
			multiplyVectorTimesScalar(TEMP3, forwardLeftUpMissileToInterceptPositionUnitVector, TEMP4);
			double COMMAND[3];
			crossProductTwoVectors(TEMP4, lineOfSightRate, COMMAND);
			missile.normComm = COMMAND[2];
			missile.sideComm = COMMAND[1];
		}
		else
		{
			double lineOfAttack[3];
			lineOfAttack[0] = forwardLeftUpMissileToInterceptPositionUnitVector[0];
			if (forwardLeftUpMissileToInterceptPositionUnitVector[1] < 0.0)
			{
				lineOfAttack[1] = 0 * degToRad;
			}
			else
			{
				lineOfAttack[1] = 0 * degToRad;
			}
			lineOfAttack[2] = -5 * degToRad;
			double TEMP1[3];
			TEMP1[0] = 4.0 * forwardLeftUpMissileToInterceptPositionUnitVector[0] - 1.5 * lineOfAttack[0];
			TEMP1[1] = 4.0 * forwardLeftUpMissileToInterceptPositionUnitVector[1] - 1.5 * lineOfAttack[1];
			TEMP1[2] = 4.0 * forwardLeftUpMissileToInterceptPositionUnitVector[2] - 1.5 * lineOfAttack[2];
			double closingSpeedUnit[3];
			unitVec(missile.fluVel, closingSpeedUnit);
			double TEMP2[3];
			crossProductTwoVectors(TEMP1, closingSpeedUnit, TEMP2);
			double TEMP3[3];
			crossProductTwoVectors(closingSpeedUnit, TEMP2, TEMP3);
			double closingSpeed;
			magnitude(missile.fluVel, closingSpeed);
			double TEMP4 = closingSpeed * closingSpeed / forwardLeftUpMissileToInterceptPositionMagnitude;
			double COMMAND[3];
			multiplyVectorTimesScalar(TEMP4, TEMP3, COMMAND);
			missile.normComm = COMMAND[2];
			missile.sideComm = COMMAND[1];
		}

		double accelerationMagnitude = sqrt(missile.sideComm * missile.sideComm + missile.normComm * missile.normComm);
		double trigonometricRatio = atan2(missile.normComm, missile.sideComm);
		if (accelerationMagnitude > missile.commLimit)
		{

			accelerationMagnitude = missile.commLimit;

		}
		missile.normComm = accelerationMagnitude * sin(trigonometricRatio);
		missile.sideComm = accelerationMagnitude * cos(trigonometricRatio);

	}
	else
	{

		missile.normComm = 0.0;
		missile.sideComm = 0.0;

	}

}

void control(Missile &missile)
{

	if (!missile.isBallistic)
	{

		// Roll autopilot.
		double rollAngleGain = 1.0;
		double rollRateProportionalGain = 0.011;
		double rollRateDerivativeGain = 0.000034125;
		double phiAngleError = ROLL_ANGLE_COMMAND - missile.enuAttitude[0]; // Radians.
		double rollRateCommand = rollAngleGain * phiAngleError; // Radians per second.
		double signOfRollRateCommand = signum(rollRateCommand); // Non dimensional.
		if (abs(rollRateCommand) > rollAngleGain) // Limit rate command.
		{
			rollRateCommand = rollAngleGain * signOfRollRateCommand;
		}
		missile.lastRollPropErr = missile.rollPropErr;
		missile.rollPropErr = rollRateCommand - missile.rate[0]; // Radians per second.
		double derivativeRollRateError = (missile.rollPropErr - missile.lastRollPropErr) / missile.timeStep;
		
		missile.rollFinComm =
		rollRateProportionalGain * missile.rollPropErr +
		rollRateDerivativeGain * derivativeRollRateError; // Radians.

		// Pitch autopilot.
		double pitchRateCommandLimit = 20;
		double pitchRateProportionalGain = 0.08;
		double pitchRateDerivativeGain = 0.000375;
		double pitchRateIntegralGain = 0.0018;
		double guidancePitchRateCommand = -missile.normComm * 6 / missile.spd;
		double signOfPitchRateCommand = signum(guidancePitchRateCommand);
		if (abs(guidancePitchRateCommand) > pitchRateCommandLimit)
		{
			guidancePitchRateCommand = signOfPitchRateCommand * pitchRateCommandLimit;
		}
		missile.lastPitchPropErr = missile.pitchPropErr;
		missile.pitchPropErr = (guidancePitchRateCommand + (missile.grav / missile.spd)) + missile.rate[1];
		double derivativePitchRateError = (missile.pitchPropErr - missile.lastPitchPropErr) / missile.timeStep;
		missile.pitchIntErr += (missile.pitchPropErr * missile.timeStep);

		missile.pitchFinComm = 
		pitchRateProportionalGain * missile.pitchPropErr +
		pitchRateDerivativeGain * derivativePitchRateError +
		pitchRateIntegralGain * missile.pitchIntErr;

		// Yaw autopilot.
		double yawRateCommandLimit = 20;
		double yawRateProportionalGain = 0.11;
		double yawRateDerivativeGain = 0.000375;
		double yawRateIntegralGain = 0.0018;
		double guidanceYawRateCommand = -missile.sideComm * 6 / missile.spd;
		double signOfYawRateCommand = signum(guidanceYawRateCommand);
		if (abs(guidanceYawRateCommand) > yawRateCommandLimit)
		{
			guidanceYawRateCommand = signOfYawRateCommand * yawRateCommandLimit;
		}
		missile.lastYawPropErr = missile.yawPropErr;
		missile.yawPropErr = guidanceYawRateCommand + missile.rate[2];
		double derivativeYawRateError = (missile.yawPropErr - missile.lastYawPropErr) / missile.timeStep;
		missile.yawIntErr += (missile.yawPropErr * missile.timeStep);

		missile.yawFinComm =
		yawRateProportionalGain * missile.yawPropErr +
		yawRateDerivativeGain * derivativeYawRateError +
		yawRateIntegralGain * missile.yawIntErr;

	}
	else
	{

		missile.rollFinComm = 0.0;
		missile.pitchFinComm = 0.0;
		missile.yawFinComm = 0.0;

	}

}

void actuators(Missile &missile)
{

	if (!missile.isBallistic)
	{

		// Fin commands.
		double DEL1C = -missile.rollFinComm + missile.pitchFinComm - missile.yawFinComm;
		double DEL2C = -missile.rollFinComm + missile.pitchFinComm + missile.yawFinComm;
		double DEL3C = missile.rollFinComm + missile.pitchFinComm - missile.yawFinComm;
		double DEL4C = missile.rollFinComm + missile.pitchFinComm + missile.yawFinComm;

		missile.FIN1DEFL = missile.FIN1->update(DEL1C * radToDeg, missile.timeStep);
		missile.FIN2DEFL = missile.FIN2->update(DEL2C * radToDeg, missile.timeStep);
		missile.FIN3DEFL = missile.FIN3->update(DEL3C * radToDeg, missile.timeStep);
		missile.FIN4DEFL = missile.FIN4->update(DEL4C * radToDeg, missile.timeStep);

		// Attitude fin deflections.
		missile.rollFinDeflection = ((-missile.FIN1DEFL - missile.FIN2DEFL + missile.FIN3DEFL + missile.FIN4DEFL) / 4) * degToRad;
		missile.pitchFinDeflection = ((missile.FIN1DEFL + missile.FIN2DEFL + missile.FIN3DEFL + missile.FIN4DEFL) / 4) * degToRad;
		missile.yawFinDeflection = ((-missile.FIN1DEFL + missile.FIN2DEFL - missile.FIN3DEFL + missile.FIN4DEFL) / 4) * degToRad;

	}
	else
	{
		missile.rollFinDeflection = 0.0;
		missile.pitchFinDeflection = 0.0;
		missile.yawFinDeflection = 0.0;
	}
	

}

void aerodynamicAnglesAndConversions(Missile &missile)
{

	missile.alphaRadians = -1 * atan2(missile.fluVel[2], missile.fluVel[0]);
	missile.betaRadians = atan2(missile.fluVel[1], missile.fluVel[0]);
	missile.alphaDegrees = missile.alphaRadians * radToDeg;
	missile.betaDegrees = missile.betaRadians * radToDeg;
	missile.alphaPrimeRadians = acos(cos(missile.alphaRadians) * cos(missile.betaRadians));
	missile.alphaPrimeDegrees = radToDeg * missile.alphaPrimeRadians;
	double phiPrime = atan2(tan(missile.betaRadians), sin(missile.alphaRadians));
	missile.sinPhiPrime = sin(phiPrime);
	missile.cosPhiPrime = cos(phiPrime);
	double pitchDeflAeroFrame = missile.pitchFinDeflection * missile.cosPhiPrime - missile.yawFinDeflection * missile.sinPhiPrime;
	missile.pitchAeroBallisticFinDeflectionDegrees = radToDeg * pitchDeflAeroFrame;
	double yawDeflAeroFrame = missile.pitchFinDeflection * missile.sinPhiPrime + missile.yawFinDeflection * missile.cosPhiPrime;
	missile.yawAeroBallisticFinDeflectionDegrees = radToDeg * yawDeflAeroFrame;
	missile.rollFinDeflectionDegrees = radToDeg * missile.rollFinDeflection;
	missile.totalFinDeflectionDegrees = (abs(missile.pitchAeroBallisticFinDeflectionDegrees) + abs(missile.yawAeroBallisticFinDeflectionDegrees)) / 2;
	double pitchRateAeroFrame = missile.rate[1] * missile.cosPhiPrime - missile.rate[2] * missile.sinPhiPrime;
	missile.pitchAeroBallisticBodyRateDegrees = radToDeg * pitchRateAeroFrame;
	double yawRateAeroFrame = missile.rate[1] * missile.sinPhiPrime + missile.rate[2] * missile.cosPhiPrime;
	missile.yawAeroBallisticBodyRateDegrees = radToDeg * yawRateAeroFrame;
	missile.rollRateDegrees = radToDeg * missile.rate[0];
	missile.sinOfFourTimesPhiPrime = sin(4 * phiPrime);
	missile.squaredSinOfTwoTimesPhiPrime = pow((sin(2 * phiPrime)), 2);

}

void tableLookUps(Missile &missile)
{

	int index;

	index = missile.tableNameIndexPairs["CA0"];
	missile.CA0 = linearInterpolationWithBoundedEnds(missile.tables[index], missile.mach);

	index = missile.tableNameIndexPairs["CAA"];
	missile.CAA = linearInterpolationWithBoundedEnds(missile.tables[index], missile.mach);

	index = missile.tableNameIndexPairs["CAD"];
	missile.CAD = linearInterpolationWithBoundedEnds(missile.tables[index], missile.mach);

	index = missile.tableNameIndexPairs["CAOFF"];
	if (missile.tof <= ROCKET_BURN_OUT_TIME)
	{
		missile.CA_POWER_CORRECTION = 0.0;
	}
	else
	{
		missile.CA_POWER_CORRECTION = linearInterpolationWithBoundedEnds(missile.tables[index], missile.mach);
	}

	index = missile.tableNameIndexPairs["CYP"];
	missile.CYP = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.mach, missile.alphaPrimeDegrees);

	index = missile.tableNameIndexPairs["CYDR"];
	missile.CYDR = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.mach, missile.alphaPrimeDegrees);

	index = missile.tableNameIndexPairs["CN0"];
	missile.CN0 = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.mach, missile.alphaPrimeDegrees);

	index = missile.tableNameIndexPairs["CNP"];
	missile.CNP = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.mach, missile.alphaPrimeDegrees);

	index = missile.tableNameIndexPairs["CNDQ"];
	missile.CNDQ = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.mach, missile.alphaPrimeDegrees);

	index = missile.tableNameIndexPairs["CLLAP"];
	missile.CLLAP = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.mach, missile.alphaPrimeDegrees);

	index = missile.tableNameIndexPairs["CLLP"];
	missile.CLLP = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.mach, missile.alphaPrimeDegrees);

	index = missile.tableNameIndexPairs["CLLDP"];
	missile.CLLDP = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.mach, missile.alphaPrimeDegrees);

	index = missile.tableNameIndexPairs["CLM0"];
	missile.CLM0 = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.mach, missile.alphaPrimeDegrees);

	index = missile.tableNameIndexPairs["CLMP"];
	missile.CLMP = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.mach, missile.alphaPrimeDegrees);

	index = missile.tableNameIndexPairs["CLMQ"];
	missile.CLMQ = linearInterpolationWithBoundedEnds(missile.tables[index], missile.mach);

	index = missile.tableNameIndexPairs["CLMDQ"];
	missile.CLMDQ = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.mach, missile.alphaPrimeDegrees);

	index = missile.tableNameIndexPairs["CLNP"];
	missile.CLNP = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.mach, missile.alphaPrimeDegrees);

	index = missile.tableNameIndexPairs["MASS"];
	missile.mass = linearInterpolationWithBoundedEnds(missile.tables[index], missile.tof);

	index = missile.tableNameIndexPairs["THRUST"];
	missile.unadjustedThrust = linearInterpolationWithBoundedEnds(missile.tables[index], missile.tof);

	index = missile.tableNameIndexPairs["TMOI"];
	missile.transverseMomentOfInertia = linearInterpolationWithBoundedEnds(missile.tables[index], missile.tof);

	index = missile.tableNameIndexPairs["AMOI"];
	missile.axialMomentOfInertia = linearInterpolationWithBoundedEnds(missile.tables[index], missile.tof);

	index = missile.tableNameIndexPairs["CG"];
	missile.centerOfGravityFromNose = linearInterpolationWithBoundedEnds(missile.tables[index], missile.tof);

}

void accelerationLimit(Missile &missile)
{

	int index;

	double currentAccelerationEstimate = missile.CN0 * missile.q * REFERENCE_AREA / missile.mass;
	index = missile.tableNameIndexPairs["CN0"];
	double CN0MAX = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.mach, ALPHA_PRIME_MAX);
	double maximumAccelerationEstimate = CN0MAX * missile.q * REFERENCE_AREA / missile.mass;
	double availableAccelerationEstimate = maximumAccelerationEstimate - currentAccelerationEstimate;

	if (missile.isHoming)
	{
		missile.commLimit = currentAccelerationEstimate + 50;
	}
	else
	{

		if (availableAccelerationEstimate < 0)
		{
			missile.commLimit = 1;
		}
		else if (availableAccelerationEstimate > MAXIMUM_ACCELERATION)
		{
			missile.commLimit = MAXIMUM_ACCELERATION;
		}
		else
		{
			missile.commLimit = availableAccelerationEstimate;
		}

	}

}

void propulsion(Missile &missile)
{

	if (missile.tof >= ROCKET_BURN_OUT_TIME)
	{
		missile.thrust = 0.0;
	}
	else
	{
		missile.thrust = missile.unadjustedThrust + (SEA_LEVEL_PRESSURE - missile.p) * THRUST_EXIT_AREA;
	}

}

void aerodynamics(Missile &missile)
{

	double CYAERO = missile.CYP * missile.sinOfFourTimesPhiPrime + missile.CYDR * missile.yawAeroBallisticFinDeflectionDegrees;
	double CZAERO = missile.CN0 + missile.CNP * missile.squaredSinOfTwoTimesPhiPrime + missile.CNDQ * missile.pitchAeroBallisticFinDeflectionDegrees;
	double CNAEROREF = missile.CLNP * missile.sinOfFourTimesPhiPrime + missile.CLMQ * missile.yawAeroBallisticBodyRateDegrees * REFERENCE_DIAMETER / (2 * missile.spd) + missile.CLMDQ * missile.yawAeroBallisticFinDeflectionDegrees;
	double CNAERO = CNAEROREF - CYAERO * (LAUNCH_CENTER_OF_GRAVITY_FROM_NOSE - missile.centerOfGravityFromNose) / REFERENCE_DIAMETER;
	double CMAEROREF = missile.CLM0 + missile.CLMP * missile.squaredSinOfTwoTimesPhiPrime + missile.CLMQ * missile.pitchAeroBallisticBodyRateDegrees * REFERENCE_DIAMETER / (2 * missile.spd) + missile.CLMDQ * missile.pitchAeroBallisticFinDeflectionDegrees;
	double CMAERO = CMAEROREF - CZAERO * (LAUNCH_CENTER_OF_GRAVITY_FROM_NOSE - missile.centerOfGravityFromNose) / REFERENCE_DIAMETER;
	
	missile.CX = missile.CA0 + missile.CAA * missile.alphaPrimeDegrees + missile.CAD * (missile.totalFinDeflectionDegrees * missile.totalFinDeflectionDegrees) + missile.CA_POWER_CORRECTION;
	missile.CY = CYAERO * missile.cosPhiPrime - CZAERO * missile.sinPhiPrime;
	missile.CZ = CYAERO * missile.sinPhiPrime + CZAERO * missile.cosPhiPrime;
	missile.CL = missile.CLLAP * missile.alphaPrimeDegrees * missile.alphaPrimeDegrees * missile.sinOfFourTimesPhiPrime + missile.CLLP * missile.rollRateDegrees * REFERENCE_DIAMETER / (2 * missile.spd) + missile.CLLDP * missile.rollFinDeflectionDegrees;
	missile.CM = CMAERO * missile.cosPhiPrime + CNAERO * missile.sinPhiPrime;
	missile.CN = -CMAERO * missile.sinPhiPrime + CNAERO * missile.cosPhiPrime;

}

void aerodynamicDerivatives(Missile &missile)
{

	int index;

	double alphaPrimeDegLookUp;
	if (missile.alphaPrimeDegrees > (ALPHA_PRIME_MAX - 3))
	{
		alphaPrimeDegLookUp = ALPHA_PRIME_MAX - 3;
	}
	else
	{
		alphaPrimeDegLookUp = missile.alphaPrimeDegrees;
	}
	double alphaPrimeDegMinusThree = alphaPrimeDegLookUp - 3;
	double alphaPrimeDegPlusThree = alphaPrimeDegLookUp + 3;
	index = missile.tableNameIndexPairs["CN0"];
	double CN0MIN = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.mach, alphaPrimeDegMinusThree);
	double CN0MAX = biLinearInterpolationWithBoundedBorders(missile.tables[index],missile.mach, alphaPrimeDegPlusThree);
	index = missile.tableNameIndexPairs["CLM0"];
	double CLM0MIN = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.mach, alphaPrimeDegMinusThree);
	double CLM0MAX = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.mach, alphaPrimeDegPlusThree);

	missile.CNA = ((CN0MAX - CN0MIN) / (alphaPrimeDegPlusThree - alphaPrimeDegMinusThree)) * radToDeg;
	missile.CMA = ((CLM0MAX - CLM0MIN) / (alphaPrimeDegPlusThree - alphaPrimeDegMinusThree) - (missile.CNA / radToDeg) * (LAUNCH_CENTER_OF_GRAVITY_FROM_NOSE - missile.centerOfGravityFromNose) / REFERENCE_DIAMETER) * radToDeg;
	missile.CND = missile.CNDQ * radToDeg;
	missile.CMD = missile.CLMDQ * radToDeg;
	missile.CMQ = missile.CLMQ * radToDeg;
	missile.CLP = missile.CLLP * radToDeg;
	missile.CLD = missile.CLLDP * radToDeg;
	missile.staticMargin = -1 * (missile.CMA * degToRad) / (missile.CNA * degToRad);

}

void eulerIntegrateStates(Missile &missile)
{

	missile.INTEGRATION_PASS = 0;

	setArrayEquivalentToReference(missile.P0, missile.enuPos);
	setArrayEquivalentToReference(missile.V0, missile.enuVel);
	setArrayEquivalentToReference(missile.W0, missile.rate);
	setArrayEquivalentToReference(missile.E0, missile.enuAttitude);

	setArrayEquivalentToReference(missile.A1, missile.enuAcc);
	setArrayEquivalentToReference(missile.WD1, missile.rateDot);
	setArrayEquivalentToReference(missile.ED1, missile.enuAttitudeDot);

	double deltaPos[3];
	multiplyVectorTimesScalar(missile.timeStep, missile.V0, deltaPos);
	addTwoVectors(missile.P0, deltaPos, missile.P1);

	double distanceTravelled;
	magnitude(deltaPos, distanceTravelled);
	missile.rng += distanceTravelled;

	double deltaVel[3];
	multiplyVectorTimesScalar(missile.timeStep, missile.A1, deltaVel);
	addTwoVectors(missile.V0, deltaVel, missile.V1);

	double deltaOmega[3];
	multiplyVectorTimesScalar(missile.timeStep, missile.WD1, deltaOmega);
	addTwoVectors(missile.W0, deltaOmega, missile.W1);

	double deltaEuler[3];
	multiplyVectorTimesScalar(missile.timeStep, missile.ED1, deltaEuler);
	addTwoVectors(missile.E0, deltaEuler, missile.E1);

	setArrayEquivalentToReference(missile.enuPos, missile.P1);
	setArrayEquivalentToReference(missile.enuVel, missile.V1);
	setArrayEquivalentToReference(missile.rate, missile.W1);
	setArrayEquivalentToReference(missile.enuAttitude, missile.E1);

	if (missile.isLaunched)
	{
		missile.tof += missile.timeStep;
	}

	setArrayEquivalentToZero(missile.P0);
	setArrayEquivalentToZero(missile.V0);
	setArrayEquivalentToZero(missile.W0);
	setArrayEquivalentToZero(missile.E0);

	setArrayEquivalentToZero(missile.A1);
	setArrayEquivalentToZero(missile.WD1);
	setArrayEquivalentToZero(missile.ED1);

	setArrayEquivalentToZero(missile.P1);
	setArrayEquivalentToZero(missile.V1);
	setArrayEquivalentToZero(missile.W1);
	setArrayEquivalentToZero(missile.E1);

}

void rk2IntegrateStates(Missile &missile)
{

	if (missile.INTEGRATION_PASS == 0)
	{

		missile.INTEGRATION_PASS += 1;

		setArrayEquivalentToReference(missile.P0, missile.enuPos);
		setArrayEquivalentToReference(missile.V0, missile.enuVel);
		setArrayEquivalentToReference(missile.W0, missile.rate);
		setArrayEquivalentToReference(missile.E0, missile.enuAttitude);

		setArrayEquivalentToReference(missile.A1, missile.enuAcc);
		setArrayEquivalentToReference(missile.WD1, missile.rateDot);
		setArrayEquivalentToReference(missile.ED1, missile.enuAttitudeDot);

		double deltaPos[3];
		multiplyVectorTimesScalar(missile.halfTimeStep, missile.V0, deltaPos);
		addTwoVectors(missile.P0, deltaPos, missile.P1);

		double deltaVel[3];
		multiplyVectorTimesScalar(missile.halfTimeStep, missile.A1, deltaVel);
		addTwoVectors(missile.V0, deltaVel, missile.V1);

		double deltaOmega[3];
		multiplyVectorTimesScalar(missile.halfTimeStep, missile.WD1, deltaOmega);
		addTwoVectors(missile.W0, deltaOmega, missile.W1);

		double deltaEuler[3];
		multiplyVectorTimesScalar(missile.halfTimeStep, missile.ED1, deltaEuler);
		addTwoVectors(missile.E0, deltaEuler, missile.E1);

		setArrayEquivalentToReference(missile.enuPos, missile.P1);
		setArrayEquivalentToReference(missile.enuVel, missile.V1);
		setArrayEquivalentToReference(missile.rate, missile.W1);
		setArrayEquivalentToReference(missile.enuAttitude, missile.E1);

		if (missile.isLaunched)
		{
			missile.tof += missile.halfTimeStep;
		}

	}
	else if (missile.INTEGRATION_PASS == 1)
	{

		missile.INTEGRATION_PASS = 0;

		setArrayEquivalentToReference(missile.A2, missile.enuAcc);
		setArrayEquivalentToReference(missile.WD2, missile.rateDot);
		setArrayEquivalentToReference(missile.ED2, missile.enuAttitudeDot);

		double deltaPos[3];
		multiplyVectorTimesScalar(missile.timeStep, missile.V1, deltaPos);
		addTwoVectors(missile.P0, deltaPos, missile.P2);

		double distanceTravelled;
		magnitude(deltaPos, distanceTravelled);
		missile.rng += distanceTravelled;

		double deltaVel[3];
		multiplyVectorTimesScalar(missile.timeStep, missile.A2, deltaVel);
		addTwoVectors(missile.V0, deltaVel, missile.V2);

		double deltaOmega[3];
		multiplyVectorTimesScalar(missile.timeStep, missile.WD2, deltaOmega);
		addTwoVectors(missile.W0, deltaOmega, missile.W2);

		double deltaEuler[3];
		multiplyVectorTimesScalar(missile.timeStep, missile.ED2, deltaEuler);
		addTwoVectors(missile.E0, deltaEuler, missile.E2);

		setArrayEquivalentToReference(missile.enuPos, missile.P2);
		setArrayEquivalentToReference(missile.enuVel, missile.V2);
		setArrayEquivalentToReference(missile.rate, missile.W2);
		setArrayEquivalentToReference(missile.enuAttitude, missile.E2);

		if (missile.isLaunched)
		{
			missile.tof += missile.halfTimeStep;
		}

		setArrayEquivalentToZero(missile.P0);
		setArrayEquivalentToZero(missile.V0);
		setArrayEquivalentToZero(missile.W0);
		setArrayEquivalentToZero(missile.E0);

		setArrayEquivalentToZero(missile.A1);
		setArrayEquivalentToZero(missile.WD1);
		setArrayEquivalentToZero(missile.ED1);

		setArrayEquivalentToZero(missile.P1);
		setArrayEquivalentToZero(missile.V1);
		setArrayEquivalentToZero(missile.W1);
		setArrayEquivalentToZero(missile.E1);

		setArrayEquivalentToZero(missile.A2);
		setArrayEquivalentToZero(missile.WD2);
		setArrayEquivalentToZero(missile.ED2);

		setArrayEquivalentToZero(missile.P2);
		setArrayEquivalentToZero(missile.V2);
		setArrayEquivalentToZero(missile.W2);
		setArrayEquivalentToZero(missile.E2);

	}

}

void rk4IntegrateStates(Missile &missile)
{

	if (missile.INTEGRATION_PASS == 0)
	{

		missile.INTEGRATION_PASS += 1;

		setArrayEquivalentToReference(missile.P0, missile.enuPos);
		setArrayEquivalentToReference(missile.V0, missile.enuVel);
		setArrayEquivalentToReference(missile.W0, missile.rate);
		setArrayEquivalentToReference(missile.E0, missile.enuAttitude);

		setArrayEquivalentToReference(missile.A1, missile.enuAcc);
		setArrayEquivalentToReference(missile.WD1, missile.rateDot);
		setArrayEquivalentToReference(missile.ED1, missile.enuAttitudeDot);

		double deltaPos[3];
		multiplyVectorTimesScalar(missile.halfTimeStep, missile.V0, deltaPos);
		addTwoVectors(missile.P0, deltaPos, missile.P1);

		double deltaVel[3];
		multiplyVectorTimesScalar(missile.halfTimeStep, missile.A1, deltaVel);
		addTwoVectors(missile.V0, deltaVel, missile.V1);

		double deltaOmega[3];
		multiplyVectorTimesScalar(missile.halfTimeStep, missile.WD1, deltaOmega);
		addTwoVectors(missile.W0, deltaOmega, missile.W1);

		double deltaEuler[3];
		multiplyVectorTimesScalar(missile.halfTimeStep, missile.ED1, deltaEuler);
		addTwoVectors(missile.E0, deltaEuler, missile.E1);

		setArrayEquivalentToReference(missile.enuPos, missile.P1);
		setArrayEquivalentToReference(missile.enuVel, missile.V1);
		setArrayEquivalentToReference(missile.rate, missile.W1);
		setArrayEquivalentToReference(missile.enuAttitude, missile.E1);

		if (missile.isLaunched)
		{
			missile.tof += missile.halfTimeStep;
		}

	}
	else if (missile.INTEGRATION_PASS == 1)
	{

		missile.INTEGRATION_PASS += 1;

		setArrayEquivalentToReference(missile.A2, missile.enuAcc);
		setArrayEquivalentToReference(missile.WD2, missile.rateDot);
		setArrayEquivalentToReference(missile.ED2, missile.enuAttitudeDot);

		double deltaPos[3];
		multiplyVectorTimesScalar(missile.halfTimeStep, missile.V1, deltaPos);
		addTwoVectors(missile.P0, deltaPos, missile.P2);

		double deltaVel[3];
		multiplyVectorTimesScalar(missile.halfTimeStep, missile.A2, deltaVel);
		addTwoVectors(missile.V0, deltaVel, missile.V2);

		double deltaOmega[3];
		multiplyVectorTimesScalar(missile.halfTimeStep, missile.WD2, deltaOmega);
		addTwoVectors(missile.W0, deltaOmega, missile.W2);

		double deltaEuler[3];
		multiplyVectorTimesScalar(missile.halfTimeStep, missile.ED2, deltaEuler);
		addTwoVectors(missile.E0, deltaEuler, missile.E2);

		setArrayEquivalentToReference(missile.enuPos, missile.P2);
		setArrayEquivalentToReference(missile.enuVel, missile.V2);
		setArrayEquivalentToReference(missile.rate, missile.W2);
		setArrayEquivalentToReference(missile.enuAttitude, missile.E2);

	}
	else if (missile.INTEGRATION_PASS == 2)
	{

		missile.INTEGRATION_PASS += 1;

		setArrayEquivalentToReference(missile.A3, missile.enuAcc);
		setArrayEquivalentToReference(missile.WD3, missile.rateDot);
		setArrayEquivalentToReference(missile.ED3, missile.enuAttitudeDot);

		double deltaPos[3];
		multiplyVectorTimesScalar(missile.timeStep, missile.V2, deltaPos);
		addTwoVectors(missile.P0, deltaPos, missile.P3);

		double deltaVel[3];
		multiplyVectorTimesScalar(missile.timeStep, missile.A3, deltaVel);
		addTwoVectors(missile.V0, deltaVel, missile.V3);

		double deltaOmega[3];
		multiplyVectorTimesScalar(missile.timeStep, missile.WD3, deltaOmega);
		addTwoVectors(missile.W0, deltaOmega, missile.W3);

		double deltaEuler[3];
		multiplyVectorTimesScalar(missile.timeStep, missile.ED3, deltaEuler);
		addTwoVectors(missile.E0, deltaEuler, missile.E3);

		setArrayEquivalentToReference(missile.enuPos, missile.P3);
		setArrayEquivalentToReference(missile.enuVel, missile.V3);
		setArrayEquivalentToReference(missile.rate, missile.W3);
		setArrayEquivalentToReference(missile.enuAttitude, missile.E3);

		if (missile.isLaunched)
		{
			missile.tof += missile.halfTimeStep;
		}

	}
	else if (missile.INTEGRATION_PASS == 3)
	{

		missile.INTEGRATION_PASS = 0;

		setArrayEquivalentToReference(missile.A4, missile.enuAcc);
		setArrayEquivalentToReference(missile.WD4, missile.rateDot);
		setArrayEquivalentToReference(missile.ED4, missile.enuAttitudeDot);

		double deltaPos[3];
		deltaPos[0] = (missile.V0[0] + missile.V1[0] * 2 + missile.V2[0] * 2 + missile.V3[0]) * (missile.timeStep / 6.0);
		deltaPos[1] = (missile.V0[1] + missile.V1[1] * 2 + missile.V2[1] * 2 + missile.V3[1]) * (missile.timeStep / 6.0);
		deltaPos[2] = (missile.V0[2] + missile.V1[2] * 2 + missile.V2[2] * 2 + missile.V3[2]) * (missile.timeStep / 6.0);
		addTwoVectors(missile.P0, deltaPos, missile.P4);

		double distanceTravelled;
		magnitude(deltaPos, distanceTravelled);
		missile.rng += distanceTravelled;

		double deltaVel[3];
		deltaVel[0] = (missile.A1[0] + missile.A2[0] * 2 + missile.A3[0] * 2 + missile.A4[0]) * (missile.timeStep / 6.0);
		deltaVel[1] = (missile.A1[1] + missile.A2[1] * 2 + missile.A3[1] * 2 + missile.A4[1]) * (missile.timeStep / 6.0);
		deltaVel[2] = (missile.A1[2] + missile.A2[2] * 2 + missile.A3[2] * 2 + missile.A4[2]) * (missile.timeStep / 6.0);
		addTwoVectors(missile.V0, deltaVel, missile.V4);

		double deltaOmega[3];
		deltaOmega[0] = (missile.WD1[0] + missile.WD2[0] * 2 + missile.WD3[0] * 2 + missile.WD4[0]) * (missile.timeStep / 6.0);
		deltaOmega[1] = (missile.WD1[1] + missile.WD2[1] * 2 + missile.WD3[1] * 2 + missile.WD4[1]) * (missile.timeStep / 6.0);
		deltaOmega[2] = (missile.WD1[2] + missile.WD2[2] * 2 + missile.WD3[2] * 2 + missile.WD4[2]) * (missile.timeStep / 6.0);
		addTwoVectors(missile.W0, deltaOmega, missile.W4);

		double deltaEuler[3];
		deltaEuler[0] = (missile.ED1[0] + missile.ED2[0] * 2 + missile.ED3[0] * 2 + missile.ED4[0]) * (missile.timeStep / 6.0);
		deltaEuler[1] = (missile.ED1[1] + missile.ED2[1] * 2 + missile.ED3[1] * 2 + missile.ED4[1]) * (missile.timeStep / 6.0);
		deltaEuler[2] = (missile.ED1[2] + missile.ED2[2] * 2 + missile.ED3[2] * 2 + missile.ED4[2]) * (missile.timeStep / 6.0);
		addTwoVectors(missile.E0, deltaEuler, missile.E4);

		setArrayEquivalentToReference(missile.enuPos, missile.P4);
		setArrayEquivalentToReference(missile.enuVel, missile.V4);
		setArrayEquivalentToReference(missile.rate, missile.W4);
		setArrayEquivalentToReference(missile.enuAttitude, missile.E4);

		setArrayEquivalentToZero(missile.P0);
		setArrayEquivalentToZero(missile.V0);
		setArrayEquivalentToZero(missile.W0);
		setArrayEquivalentToZero(missile.E0);

		setArrayEquivalentToZero(missile.A1);
		setArrayEquivalentToZero(missile.WD1);
		setArrayEquivalentToZero(missile.ED1);

		setArrayEquivalentToZero(missile.P1);
		setArrayEquivalentToZero(missile.V1);
		setArrayEquivalentToZero(missile.W1);
		setArrayEquivalentToZero(missile.E1);

		setArrayEquivalentToZero(missile.A2);
		setArrayEquivalentToZero(missile.WD2);
		setArrayEquivalentToZero(missile.ED2);

		setArrayEquivalentToZero(missile.P2);
		setArrayEquivalentToZero(missile.V2);
		setArrayEquivalentToZero(missile.W2);
		setArrayEquivalentToZero(missile.E2);

		setArrayEquivalentToZero(missile.A3);
		setArrayEquivalentToZero(missile.WD3);
		setArrayEquivalentToZero(missile.ED3);

		setArrayEquivalentToZero(missile.P3);
		setArrayEquivalentToZero(missile.V3);
		setArrayEquivalentToZero(missile.W3);
		setArrayEquivalentToZero(missile.E3);

		setArrayEquivalentToZero(missile.A4);
		setArrayEquivalentToZero(missile.WD4);
		setArrayEquivalentToZero(missile.ED4);

		setArrayEquivalentToZero(missile.P4);
		setArrayEquivalentToZero(missile.V4);
		setArrayEquivalentToZero(missile.W4);
		setArrayEquivalentToZero(missile.E4);

	}

}

void missileMotion(Missile &missile)
{

	/* Derivatives. */

	// Forces.
	double axialForce = missile.thrust - missile.CX * missile.q * REFERENCE_AREA + missile.fluGrav[0] * missile.mass;
	double sideForce = missile.CY * missile.q * REFERENCE_AREA + missile.fluGrav[1] * missile.mass;
	double normalForce = missile.CZ * missile.q * REFERENCE_AREA + missile.fluGrav[2] * missile.mass;

	// Moments.
	double rollMoment = missile.CL * missile.q * REFERENCE_AREA * REFERENCE_DIAMETER;
	double pitchMoment = missile.CM * missile.q * REFERENCE_AREA * REFERENCE_DIAMETER;
	double yawMoment = missile.CN * missile.q * REFERENCE_AREA * REFERENCE_DIAMETER;

	// Specific force.
	missile.spcfForce[0] = axialForce / missile.mass;
	missile.spcfForce[1] = sideForce / missile.mass;
	missile.spcfForce[2] = normalForce / missile.mass;

	// Rotate FLU acceleration into ENU acceleration.
	oneByThreeTimesThreeByThree(missile.spcfForce, missile.enuToFlu, missile.enuAcc);

	// Omega dot.
	missile.rateDot[0] = rollMoment / missile.axialMomentOfInertia;
	missile.rateDot[1] = (1 / missile.transverseMomentOfInertia) * ((missile.transverseMomentOfInertia - missile.axialMomentOfInertia) * missile.rate[0] * missile.rate[2] + pitchMoment);
	missile.rateDot[2] = (1 / missile.transverseMomentOfInertia) * ((missile.axialMomentOfInertia - missile.transverseMomentOfInertia) * missile.rate[0] * missile.rate[1] + yawMoment);

	// Euler dot.
	missile.enuAttitudeDot[0] = missile.rate[0] + (missile.rate[1] * sin(missile.enuAttitude[0]) + missile.rate[2] * cos(missile.enuAttitude[0])) * tan(missile.enuAttitude[1]);
	missile.enuAttitudeDot[1] = missile.rate[1] * cos(missile.enuAttitude[0]) - missile.rate[2] * sin(missile.enuAttitude[0]);
	missile.enuAttitudeDot[2] = (missile.rate[1] * sin(missile.enuAttitude[0]) + missile.rate[2] * cos(missile.enuAttitude[0])) / cos(missile.enuAttitude[1]);

	// STATE.
	if (missile.INTEGRATION_METHOD == 0)
	{
		eulerIntegrateStates(missile);
	}
	else if (missile.INTEGRATION_METHOD == 1)
	{
		rk2IntegrateStates(missile);
	}
	else if (missile.INTEGRATION_METHOD == 2)
	{
		rk4IntegrateStates(missile);
	}

	// Adjust local to body direction cosine matrix.
	eulerAnglesToLocalOrientation(
		missile.enuAttitude[0],
		-missile.enuAttitude[1],
		missile.enuAttitude[2],
		missile.enuToFlu
	);

	threeByThreeTimesThreeByOne(missile.enuToFlu, missile.enuVel, missile.fluVel);

}

void performanceAndTerminationCheck(Missile &missile, double maxTime)
{

	magnitude(missile.mslToWaypoint, missile.missDistance);

	if (!missile.isBallistic)
	{

		if (missile.enuPos[2] < 0)
		{
			missile.lethality = "GROUND_COLLISION";
		}
		else if (missile.missDistance < 5.0)
		{
			missile.lethality = "SUCCESSFUL_INTERCEPT";
		}
		else if (missile.mslToWaypoint[0] < 0)
		{
			missile.lethality = "POINT_OF_CLOSEST_APPROACH_PASSED";
		}
		else if (isnan(missile.enuPos[0]))
		{
			missile.lethality = "NOT_A_NUMBER";
		}
		else if (missile.tof > maxTime)
		{
			missile.lethality = "MAX_TIME_EXCEEDED";
		}

	}
	else
	{

		if (missile.enuPos[2] < 0)
		{
			missile.lethality = "GROUND_COLLISION";
		}
		else if (isnan(missile.enuPos[0]))
		{
			missile.lethality = "NOT_A_NUMBER";
		}
		else if (missile.tof > maxTime)
		{
			missile.lethality = "MAX_TIME_EXCEEDED";
		}

	}



}

void writeLogFileHeader(ofstream &logFile)
{

	// Logging everything.
	logFile << fixed << setprecision(10) <<
	"tgtE" <<
	" " << "tgtN" <<
	" " << "tgtU" <<
	" " << "tof" <<
	" " << "posE" <<
	" " << "posN" <<
	" " << "posU" <<
	" " << "range" <<
	" " << "velE" <<
	" " << "velN" <<
	" " << "velU" <<
	" " << "u" <<
	" " << "v" <<
	" " << "w" <<
	" " << "speed" <<
	" " << "mach" <<
	" " << "accE" <<
	" " << "accN" <<
	" " << "accU" <<
	" " << "udot" <<
	" " << "vdot" <<
	" " << "wdot" <<
	" " << "ENUToFLU_0_0" <<
	" " << "ENUToFLU_0_1" <<
	" " << "ENUToFLU_0_2" <<
	" " << "ENUToFLU_1_0" <<
	" " << "ENUToFLU_1_1" <<
	" " << "ENUToFLU_1_2" <<
	" " << "ENUToFLU_2_0" <<
	" " << "ENUToFLU_2_1" <<
	" " << "ENUToFLU_2_2" <<
	" " << "alphaRadians" <<
	" " << "betaRadians" <<
	" " << "alphaDegrees" <<
	" " << "betaDegrees" <<
	" " << "phi" <<
	" " << "theta" <<
	" " << "psi" <<
	" " << "phiDot" <<
	" " << "thetaDot" <<
	" " << "psiDot" <<
	" " << "p" <<
	" " << "q" <<
	" " << "r" <<
	" " << "pdot" <<
	" " << "qdot" <<
	" " << "rdot" <<
	" " << "gravity" <<
	" " << "axialGravity" <<
	" " << "sideGravity" <<
	" " << "normalGravity" <<
	" " << "pressure" <<
	" " << "dynamicPressure" <<
	" " << "seekerPitch" <<
	" " << "seekerYaw" <<
	" " << "seekerENUToFLU_0_0" <<
	" " << "seekerENUToFLU_0_1" <<
	" " << "seekerENUToFLU_0_2" <<
	" " << "seekerENUToFLU_1_0" <<
	" " << "seekerENUToFLU_1_1" <<
	" " << "seekerENUToFLU_1_2" <<
	" " << "seekerENUToFLU_2_0" <<
	" " << "seekerENUToFLU_2_1" <<
	" " << "seekerENUToFLU_2_2" <<
	" " << "seekerPitchError" <<
	" " << "seekerYawError" <<
	" " << "seekerWLR" <<
	" " << "seekerWLRD" <<
	" " << "seekerWLR1" <<
	" " << "seekerWLR1D" <<
	" " << "seekerWLR2" <<
	" " << "seekerWLR2D" <<
	" " << "seekerWLQ" <<
	" " << "seekerWLQD" <<
	" " << "seekerWLQ1" <<
	" " << "seekerWLQ1D" <<
	" " << "seekerWLQ2" <<
	" " << "seekerWLQ2D" <<
	" " << "homing" <<
	" " << "timeToGo" <<
	" " << "missileToInterceptRelativePositionForward"
	" " << "missileToInterceptRelativePositionLeft"
	" " << "missileToInterceptRelativePositionUp"
	" " << "guidanceNormalCommand" <<
	" " << "guidanceSideCommand" <<
	" " << "accelerationLimit" <<
	" " << "lastRollRateError" <<
	" " << "rollRateError" <<
	" " << "rollFinCommand" <<
	" " << "lastPitchRateError" <<
	" " << "pitchRateError" <<
	" " << "pitchFinCommand" <<
	" " << "lastYawRateError" <<
	" " << "yawRateError" <<
	" " << "yawFinCommand" <<
	" " << "rollFinDeflection" <<
	" " << "pitchFinDeflection" <<
	" " << "yawFinDeflection" <<
	" " << "finOneDeflection" <<
	" " << "finTwoDeflection" <<
	" " << "finThreeDeflection" <<
	" " << "finFourDeflection" <<
	" " << "alphaPrimeRadians" <<
	" " << "alphaPrimeDegrees"
	" " << "sinPhiPrime" <<
	" " << "cosPhiPrime" <<
	" " << "rollFinDeflectionDegrees" <<
	" " << "pitchFinDeflectionDegreesAeroBallisticFrame" <<
	" " << "yawFinDeflectionDegreesAeroBallisticFrame" <<
	" " << "totalFinDeflectionDegrees" <<
	" " << "pitchRateDegreesAeroBallisticFrame" <<
	" " << "yawRateDegreesAeroBallisticFrame" <<
	" " << "rollRateDegrees" <<
	" " << "sinOfFourTimesPhiPrime" <<
	" " << "squaredSinOfTwoTimesPhiPrime"
	" " << "CA0" <<
	" " << "CAA" <<
	" " << "CAD" <<
	" " << "CAOFF" <<
	" " << "CYP" <<
	" " << "CYDR" <<
	" " << "CN0" <<
	" " << "CNP" <<
	" " << "CNDQ" <<
	" " << "CLLAP" <<
	" " << "CLLP" <<
	" " << "CLLDP" <<
	" " << "CLM0" <<
	" " << "CLMP" <<
	" " << "CLMQ" <<
	" " << "CLMDQ" <<
	" " << "CLNP" <<
	" " << "mass" <<
	" " << "unadjustedThrust" <<
	" " << "transverseMomentOfInertia" <<
	" " << "axialMomentOfInertia" <<
	" " << "centerOfGravityFromNose" <<
	" " << "thrust" <<
	" " << "CX" <<
	" " << "CY" <<
	" " << "CZ" <<
	" " << "CL" <<
	" " << "CM" <<
	" " << "CN" <<
	" " << "CNA" <<
	" " << "CMA" <<
	" " << "CND" <<
	" " << "CMD" <<
	" " << "CMQ" <<
	" " << "CLP" <<
	" " << "CLD" <<
	" " << "staticMargin" <<
	" " << "missDistance" <<
	" " << "lethality" <<
	" " << "launch" <<
	"\n";

}

void logData(Missile &missile, ofstream &logFile)
{

	logFile << fixed << setprecision(10) <<
	missile.waypoint[0] << " " <<
	missile.waypoint[1] << " " <<
	missile.waypoint[2] << " " <<
	missile.tof << " " <<
	missile.enuPos[0] << " " <<
	missile.enuPos[1] << " " <<
	missile.enuPos[2] << " " <<
	missile.rng << " " <<
	missile.enuVel[0] << " " <<
	missile.enuVel[1] << " " <<
	missile.enuVel[2] << " " <<
	missile.fluVel[0] << " " <<
	missile.fluVel[1] << " " <<
	missile.fluVel[2] << " " <<
	missile.spd << " " <<
	missile.mach << " " <<
	missile.enuAcc[0] << " " <<
	missile.enuAcc[1] << " " <<
	missile.enuAcc[2] << " " <<
	missile.spcfForce[0] << " " <<
	missile.spcfForce[1] << " " <<
	missile.spcfForce[2] << " " <<
	missile.enuToFlu[0][0] << " " <<
	missile.enuToFlu[0][1] << " " <<
	missile.enuToFlu[0][2] << " " <<
	missile.enuToFlu[1][0] << " " <<
	missile.enuToFlu[1][1] << " " <<
	missile.enuToFlu[1][2] << " " <<
	missile.enuToFlu[2][0] << " " <<
	missile.enuToFlu[2][1] << " " <<
	missile.enuToFlu[2][2] << " " <<
	missile.alphaRadians << " " <<
	missile.betaRadians << " " <<
	missile.alphaDegrees << " " <<
	missile.betaDegrees << " " <<
	missile.enuAttitude[0] << " " <<
	missile.enuAttitude[1] << " " <<
	missile.enuAttitude[2] << " " <<
	missile.enuAttitudeDot[0] << " " <<
	missile.enuAttitudeDot[1] << " " <<
	missile.enuAttitudeDot[2] << " " <<
	missile.rate[0] << " " <<
	missile.rate[1] << " " <<
	missile.rate[2] << " " <<
	missile.rateDot[0] << " " <<
	missile.rateDot[1] << " " <<
	missile.rateDot[2] << " " <<
	missile.grav << " " <<
	missile.fluGrav[0] << " " <<
	missile.fluGrav[1] << " " <<
	missile.fluGrav[2] << " " <<
	missile.p << " " <<
	missile.q << " " <<
	missile.skrTht << " " <<
	missile.skrPsi << " " <<
	missile.skrEnuToFlu[0][0] << " " <<
	missile.skrEnuToFlu[0][1] << " " <<
	missile.skrEnuToFlu[0][2] << " " <<
	missile.skrEnuToFlu[1][0] << " " <<
	missile.skrEnuToFlu[1][1] << " " <<
	missile.skrEnuToFlu[1][2] << " " <<
	missile.skrEnuToFlu[2][0] << " " <<
	missile.skrEnuToFlu[2][1] << " " <<
	missile.skrEnuToFlu[2][2] << " " <<
	missile.skrThtErr << " " <<
	missile.skrPsiErr << " " <<
	missile.skrWlr << " " <<
	missile.skrWlrDot << " " <<
	missile.skrWlr1 << " " <<
	missile.skrWlr1Dot << " " <<
	missile.skrWlr2 << " " <<
	missile.skrWlr2Dot << " " <<
	missile.skrWlq << " " <<
	missile.skrWlqDot << " " <<
	missile.skrWlq1 << " " <<
	missile.skrWlq1Dot << " " <<
	missile.skrWlq2 << " " <<
	missile.skrWlq2Dot << " " <<
	missile.isHoming << " " <<
	missile.timeToGo << " " <<
	missile.mslToWaypoint[0] << " " <<
	missile.mslToWaypoint[1] << " " <<
	missile.mslToWaypoint[2] << " " <<
	missile.normComm << " " <<
	missile.sideComm << " " <<
	missile.commLimit << " " <<
	missile.lastRollPropErr << " " <<
	missile.rollPropErr << " " <<
	missile.rollFinComm << " " <<
	missile.lastPitchPropErr << " " <<
	missile.pitchPropErr << " " <<
	missile.pitchFinComm << " " <<
	missile.lastYawPropErr << " " <<
	missile.yawPropErr << " " <<
	missile.yawFinComm << " " <<
	missile.rollFinDeflection << " " <<
	missile.pitchFinDeflection << " " <<
	missile.yawFinDeflection << " " <<
	missile.FIN1DEFL << " " <<
	missile.FIN2DEFL << " " <<
	missile.FIN3DEFL << " " <<
	missile.FIN4DEFL << " " <<
	missile.alphaPrimeRadians << " " <<
	missile.alphaPrimeDegrees << " " <<
	missile.sinPhiPrime << " " <<
	missile.cosPhiPrime << " " <<
	missile.rollFinDeflectionDegrees << " " <<
	missile.pitchAeroBallisticFinDeflectionDegrees << " " <<
	missile.yawAeroBallisticFinDeflectionDegrees << " " <<
	missile.totalFinDeflectionDegrees << " " <<
	missile.pitchAeroBallisticBodyRateDegrees << " " <<
	missile.yawAeroBallisticBodyRateDegrees << " " <<
	missile.rollRateDegrees << " " <<
	missile.sinOfFourTimesPhiPrime << " " <<
	missile.squaredSinOfTwoTimesPhiPrime << " " <<
	missile.CA0 << " " <<
	missile.CAA << " " <<
	missile.CAD << " " <<
	missile.CA_POWER_CORRECTION << " " <<
	missile.CYP << " " <<
	missile.CYDR << " " <<
	missile.CN0 << " " <<
	missile.CNP << " " <<
	missile.CNDQ << " " <<
	missile.CLLAP << " " <<
	missile.CLLP << " " <<
	missile.CLLDP << " " <<
	missile.CLM0 << " " <<
	missile.CLMP << " " <<
	missile.CLMQ << " " <<
	missile.CLMDQ << " " <<
	missile.CLNP << " " <<
	missile.mass << " " <<
	missile.unadjustedThrust << " " <<
	missile.transverseMomentOfInertia << " " <<
	missile.axialMomentOfInertia << " " <<
	missile.centerOfGravityFromNose << " " <<
	missile.thrust << " " <<
	missile.CX << " " <<
	missile.CY << " " <<
	missile.CZ << " " <<
	missile.CL << " " <<
	missile.CM << " " <<
	missile.CN << " " <<
	missile.CNA << " " <<
	missile.CMA << " " <<
	missile.CND << " " <<
	missile.CMD << " " <<
	missile.CMQ << " " <<
	missile.CLP << " " <<
	missile.CLD << " " <<
	missile.staticMargin << " " <<
	missile.missDistance << " " <<
	missile.lethality << " " <<
	missile.isLaunched <<
	"\n";

}

void sixDofFly(Missile &missile, string flyOutID, bool writeData, bool consoleReport, double flyForThisLong)
{

	// For console report if requested.
	double lastTime = missile.tof;

	// For log file if requested.
	ofstream logFile;

	if (writeData)
	{

		logFile.open("CPP_6DOF_SRAAM_V2/output/" + flyOutID + "_6DOF.txt");
		writeLogFileHeader(logFile);

	}

	if (consoleReport)
	{

		cout << "\n6DOF " + flyOutID + "\n";
		cout << "\n";

	}

	while (missile.lethality == "FLYING")
	{

		atmosphere(missile);
		seeker(missile);
		guidance(missile);
		control(missile);
		actuators(missile);
		aerodynamicAnglesAndConversions(missile);
		tableLookUps(missile);
		accelerationLimit(missile);
		propulsion(missile);
		aerodynamics(missile);
		aerodynamicDerivatives(missile);
		missileMotion(missile);

		if (missile.INTEGRATION_PASS == 0)
		{
			performanceAndTerminationCheck(missile, flyForThisLong);

			if (writeData)
			{

				logData(missile, logFile);

			}
			
			if (consoleReport)
			{

				auto print_it = static_cast<int>(round(missile.tof * 10000.0)) % 10000;
				if (print_it == 0)
				{
					cout
					<< setprecision(6)
					<< missile.tof
					<< " E "
					<< missile.enuPos[0]
					<< " N "
					<< missile.enuPos[1]
					<< " U "
					<< missile.enuPos[2]
					<< " MACH "
					<< missile.mach
					<< endl;
					lastTime = missile.tof;
				}

			}

		}

	}

	if (consoleReport)
	{

		cout << "\n";
		cout << "6DOF " + flyOutID + " REPORT" << endl;
		cout << setprecision(2) << "FINAL POSITION AT " << missile.tof << " E " << missile.enuPos[0] << " N " << missile.enuPos[1] << " U " << missile.enuPos[2] << " MACH " << missile.mach << endl;
		cout << setprecision(2) << "MISS DISTANCE " << missile.missDistance << " FORWARD, LEFT, UP, MISS DISTANCE " << missile.mslToWaypoint[0] << " " << missile.mslToWaypoint[1] << " " << missile.mslToWaypoint[2] << endl;
		cout << "SIMULATION RESULT: " << missile.lethality << endl;

	}

}

void threeDofFly(Missile &missile, string flyOutID, bool writeData, bool consoleReport, double flyForThisLong)
{

	const double TIME_STEP = 0.01; // Seconds.

	double missileAzimuth = missile.enuAttitude[2];
	double missileElevation = missile.enuAttitude[1];

	ofstream logFile;

	if (writeData)
	{
		logFile.open("CPP_6DOF_SRAAM_V2/output/" + flyOutID + "_3DOF.txt");
		logFile << fixed << setprecision(10) << "tof posE posN posU tgtE tgtN tgtU alpha beta lethality" << endl;
	}

	if (consoleReport)
	{
		cout << "\n3DOF " + flyOutID + "\n";
		cout << "\n";
	}
	
	double lastTime = 0.0;

	while (missile.lethality == "FLYING")
	{

		// Common.
		int index;
		double TEMP;

		// Time of flight.
		missile.tof += TIME_STEP;

		// Orientation.
		azAndElFromVector(missileAzimuth, missileElevation, missile.enuVel);
		flightPathAnglesToLocalOrientation(missileAzimuth, -1.0 * missileElevation, missile.enuToFlu);

		// Atmosphere.
		magnitude(missile.enuVel, missile.spd);
		auto ATM = atm1976_metric::update(missile.enuPos[2], missile.spd);
		missile.grav = ATM.g;
		missile.p = ATM.p;
		missile.q = ATM.q;
		missile.mach = ATM.mach;

		// Aero ballistic angles.
		aerodynamicAnglesAndConversions(missile);

		// Lookups.
		index = missile.tableNameIndexPairs["MASS"];
		missile.mass = linearInterpolationWithBoundedEnds(missile.tables[index], missile.tof);
		index = missile.tableNameIndexPairs["THRUST"];
		missile.unadjustedThrust = linearInterpolationWithBoundedEnds(missile.tables[index], missile.tof);
		index = missile.tableNameIndexPairs["CA0"];
		missile.CA0 = linearInterpolationWithBoundedEnds(missile.tables[index], missile.mach);
		index = missile.tableNameIndexPairs["CAA"];
		missile.CAA = linearInterpolationWithBoundedEnds(missile.tables[index], missile.mach);
		if (missile.tof <= ROCKET_BURN_OUT_TIME)
		{
			missile.CA_POWER_CORRECTION = 0.0;
		}
		else
		{
			index = missile.tableNameIndexPairs["CAOFF"];
			missile.CA_POWER_CORRECTION = linearInterpolationWithBoundedEnds(missile.tables[index], missile.mach);
		}
		index = missile.tableNameIndexPairs["CYP"];
		missile.CYP = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.mach, missile.alphaPrimeDegrees);
		double CYP_Max = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.mach, ALPHA_PRIME_MAX);
		index = missile.tableNameIndexPairs["CN0"];
		missile.CN0 = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.mach, missile.alphaPrimeDegrees);
		double CN0_Max = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.mach, ALPHA_PRIME_MAX);
		index = missile.tableNameIndexPairs["CNP"];
		missile.CNP = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.mach, missile.alphaPrimeDegrees);
		double CNP_Max = biLinearInterpolationWithBoundedBorders(missile.tables[index], missile.mach, ALPHA_PRIME_MAX);


		// Propulsion.
		if (missile.tof <= ROCKET_BURN_OUT_TIME)
		{
			missile.thrust = missile.unadjustedThrust + (SEA_LEVEL_PRESSURE - missile.p) * THRUST_EXIT_AREA;
		}
		else
		{
			missile.thrust = 0.0;
		}

		// Aerodynamics
		double CYAERO_Actual = missile.CYP * missile.sinOfFourTimesPhiPrime;
		double CYAERO_Max = CYP_Max * missile.sinOfFourTimesPhiPrime;
		double CZAERO_Actual = missile.CN0 + missile.CNP * missile.squaredSinOfTwoTimesPhiPrime;
		double CZAERO_Max = CN0_Max + CNP_Max * missile.squaredSinOfTwoTimesPhiPrime;
		double CY_Max = CYAERO_Max * missile.cosPhiPrime - CZAERO_Max * missile.sinPhiPrime;
		double CZ_Max = CYAERO_Max * missile.sinPhiPrime + CZAERO_Max * missile.cosPhiPrime;
		missile.CX = missile.CA0 + missile.CAA * missile.alphaPrimeDegrees + missile.CA_POWER_CORRECTION;
		missile.CZ = CYAERO_Actual * missile.sinPhiPrime + CZAERO_Actual * missile.cosPhiPrime;
		missile.CY = CYAERO_Actual * missile.cosPhiPrime - CZAERO_Actual * missile.sinPhiPrime;

		// Guidance.
		if (!missile.isBallistic)
		{
			double relPos[3];
			subtractTwoVectors(missile.enuPos, missile.waypoint, relPos);
			double mslToIntercept[3];
			threeByThreeTimesThreeByOne(missile.enuToFlu, relPos, mslToIntercept);
			setArrayEquivalentToReference(missile.mslToWaypoint, mslToIntercept);
			double maxNormalAccelerationAllowed = abs((CZ_Max * missile.q * REFERENCE_AREA) / missile.mass);
			double maxSideAccelerationAllowed = abs((CY_Max * missile.q * REFERENCE_AREA) / missile.mass);
			missile.commLimit = sqrt(maxNormalAccelerationAllowed * maxNormalAccelerationAllowed + maxSideAccelerationAllowed * maxSideAccelerationAllowed);
			guidance(missile);
		}
		else
		{
			missile.normComm = 0;
			missile.sideComm = 0;
		}

		// Forces
		double axialForce = missile.thrust - missile.CX * missile.q * REFERENCE_AREA + missile.fluGrav[0] * missile.mass;
		double sideForce = missile.CY * missile.q * REFERENCE_AREA + missile.fluGrav[1] * missile.mass;
		double normalForce = missile.CZ * missile.q * REFERENCE_AREA + missile.fluGrav[2] * missile.mass;

		// Specific force.
		missile.spcfForce[0] = axialForce / missile.mass;
		missile.spcfForce[1] = sideForce / missile.mass + missile.sideComm;
		missile.spcfForce[2] = normalForce / missile.mass + missile.normComm;
		oneByThreeTimesThreeByThree(missile.spcfForce, missile.enuToFlu, missile.enuAcc);

		// Motion integration.
		double deltaPos[3];
		multiplyVectorTimesScalar(TIME_STEP, missile.enuVel, deltaPos);
		double newMissileENUPosition[3];
		addTwoVectors(missile.enuPos, deltaPos, newMissileENUPosition);
		setArrayEquivalentToReference(missile.enuPos, newMissileENUPosition);

		double deltaVel[3];
		multiplyVectorTimesScalar(TIME_STEP, missile.enuAcc, deltaVel);
		double newMissileENUVelocity[3];
		addTwoVectors(missile.enuVel, deltaVel, newMissileENUVelocity);
		setArrayEquivalentToReference(missile.enuVel, newMissileENUVelocity);

		// Alpha and beta.
		threeByThreeTimesThreeByOne(missile.enuToFlu, missile.enuVel, missile.fluVel);
		missile.alphaRadians = -1.0 * atan2_0(missile.fluVel[2], missile.fluVel[0]);
		missile.betaRadians = atan2_0(missile.fluVel[1], missile.fluVel[0]);

		// Performance and termination check.
		performanceAndTerminationCheck(missile, flyForThisLong);

		// Log data.
		if (writeData)
		{
			logFile << fixed << setprecision(10) <<
			missile.tof << " " <<
			missile.enuPos[0] << " " <<
			missile.enuPos[1] << " " <<
			missile.enuPos[2] << " " <<
			missile.waypoint[0] << " " <<
			missile.waypoint[1] << " " <<
			missile.waypoint[2] << " " <<
			missile.alphaRadians << " " <<
			missile.betaRadians << " " <<
			missile.lethality << endl;
		}

		if (consoleReport)
		{
			auto print_it = static_cast<int>(round(missile.tof * 10000.0)) % 10000;
			if (print_it == 0)
			{
				cout << setprecision(6) << missile.tof << " E " << missile.enuPos[0] << " N " << missile.enuPos[1] << " U " << missile.enuPos[2] << " MACH " << missile.mach << endl;
				lastTime = missile.tof;
			}
		}

	}

	if (consoleReport)
	{
		cout << "\n";
		cout << "3DOF " + flyOutID + " REPORT" << endl;
		cout << setprecision(6) << "FINAL POSITION AT " << missile.tof << " E " << missile.enuPos[0] << " N " << missile.enuPos[1] << " U " << missile.enuPos[2] << " MACH " << missile.mach << endl;
		cout << setprecision(6) << "MISS DISTANCE " << missile.missDistance << " FORWARD, LEFT, UP, MISS DISTANCE " << missile.mslToWaypoint[0] << " " << missile.mslToWaypoint[1] << " " << missile.mslToWaypoint[2] << endl;
		cout << "SIMULATION RESULT: " << missile.lethality << endl;
		cout << "\n";
	}

}


