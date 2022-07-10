// STANDARD LIBRARY
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <math.h>
#include <vector>
#include <map>
#include <algorithm>

// SOURCE CODE
#include "src/util.h"
#include "src/flightComputer.h"
#include "src/environment.h"
#include "src/IMU.h"
#include "src/seeker.h"
#include "src/SRAAMEngine/SRAAMEngine.h"
#include "src/translationalDynamics.h"
#include "src/rotationalDynamics.h"
#include "src/translationalKinematics.h"
#include "src/rotationalKinematics.h"
#include "src/intercept.h"
#include "src/endCheck.h"
#include "src/logData.h"

using namespace std;

/*
#
# AUTHOR - WILSON GRAHAM BEECH
# ANY MODULES NOT WRITTEN BY GRAHAM BEECH ARE DENOTED AS SUCH
# REFERENCE - MODELING AND SIMULATION OF AEROSPACE VEHICLE DYNAMICS, SECOND EDITON - PETER H. ZIPFEL
#
# EAST, NORTH, UP COORDINATE SYSTEM
#
# INTERCEPTOR LOCAL ORIENTATION
# ARRAY 0 >>> LOOKING DOWN THE NOZZLE OF THE INTERCEPTOR
# ARRAY 1 >>> LOOKING DOWN THE NOZZLE OF THE INTERCEPTOR, THIS POINTS OUT THE LEFT HAND SIDE
# ARRAY 2 >>> LOOKING DOWN THE NOZZLE OF THE INTERCEPTOR, THIS POINTS OUT THE TOP SIDE OF THE INTERCEPTOR
#
#                           POSITIVE NORMAL
#                                         |
#                                         |
#                                         |
#  POSITIVE SIDE -----------O----------- NEGATIVE SIDE
#                                         |
#                                         |
#                                         |
#                          NEGATIVE NORMAL
#
# NEGATIVE AXIS IS COMING OUT OF THE SCREEN STRAIGHT AT YOU
# POSITIVE AXIS IS POINTING INTO THE SCREEN DIRECTLY AWAY FROM YOU
#
# POSITIVE ALPHA INDICATES NOSE BELOW FREE STREAM VELOCITY
# POSITIVE BETA INDICATES NOSE LEFT FREE STREAM VELOCITY
# POSITIVE ROLL INDICATES NORMAL AXIS CLOCKWISELY ROTATED FROM TWELVE O'CLOCK
#
# FIN ORIENTATION
# LOOKING DOWN THE NOZZLE OF THE MISSILE
#
#                              FIN 4       FIN 1
#                                         X
#                              FIN 3       FIN 2
#
*/

void fly()
{
	// REAL TIME START
	auto wallClockStart = chrono::high_resolution_clock::now();

	// DATA PACKET. THIS IS A STRUCT DEFINED IN "UTIL.H" THAT IS PASSED TO EVERY OBJECT, INIT AND UPDATE. IT CONTAINS INFORMATION WHICH IS NEEDED TO BE ACCESSED GLOBALLY.
	mslDataPacket dataPacket;

	// INITIALIZE AND OPEN INPUT FILE
	std::ifstream inPut;
	inPut.open("input.txt");

	// INITIALIZE INPUTS
	double timeStep, integrationStep, maxTime, phi, tht, psi, tgtE, tgtN, tgtU;

	// READ INPUTS FROM INPUT FILE
	inPut >> timeStep >> integrationStep >> maxTime >> phi >> tht >> psi >> tgtE >> tgtN >> tgtU;
	phi *= degToRad;
	tht *= degToRad;
	psi *= degToRad;
	
	cout << "\n" << endl;
	// INITIALIZE MSL COMPONENTS OR "BLOCKS." THESE OBJECTS PERFORM THE VARIOUS COMPUTATIONS REQUIRED TO FLY THE MISSILE.
	flightComputer flightComputerObj(&dataPacket, timeStep, integrationStep, maxTime);
	environment environmentObj(&dataPacket);
	IMU IMUObj(&dataPacket, phi, tht, psi);
	seeker seekerObj(&dataPacket, tgtE, tgtN, tgtU);
	SRAAMEngine SRAAMEngineObj(&dataPacket);
	translationalDynamics translationalDynamicsObj(&dataPacket);
	rotationalDynamics rotationalDynamicsObj(&dataPacket);
	translationalKinematics translationalKinematicsObj(&dataPacket);
	rotationalKinematics rotationalKinematicsObj(&dataPacket);
	intercept interceptObj(&dataPacket);
	endCheck endCheckObj(&dataPacket);
	logData logDataObj(&dataPacket);

	// LOG FIRST LINE OF DATA
	logDataObj.update(&dataPacket);

	// FLY LOOP
	cout << "\n" << endl;
	cout << "FLIGHT" << endl;
	double lastTime = 0;
	while (dataPacket.fly)
	{

		// OBJECT UPDATES
		flightComputerObj.update(&dataPacket);
		environmentObj.update(&dataPacket);
		IMUObj.update(&dataPacket);
		seekerObj.update(&dataPacket);
		SRAAMEngineObj.update(&dataPacket);	
		translationalDynamicsObj.update(&dataPacket);
		rotationalDynamicsObj.update(&dataPacket);
		translationalKinematicsObj.update(&dataPacket);
		rotationalKinematicsObj.update(&dataPacket);
		interceptObj.update(&dataPacket);
		endCheckObj.update(&dataPacket);
		logDataObj.update(&dataPacket);

		// CONSOLE OUTPUT
		auto print_it = static_cast<int>(round(dataPacket.mslTof * 10000.0)) % 10000;
		if (print_it == 0)
		{
			cout << setprecision(6) << dataPacket.mslTof << " E " << dataPacket.mslPos[0] << " N " << dataPacket.mslPos[1] << " U " << dataPacket.mslPos[2] << " RANGE " << dataPacket.mslRange << " MACH " << dataPacket.mslMach << endl;
			lastTime = dataPacket.mslTof;
		}

	}

	// MISSION REPORT
	cout << "\n" << endl;
	cout << "MISSION REPORT" << endl;
	cout << setprecision(6) << "FINAL POSITION AT " << dataPacket.mslTof << " E " << dataPacket.mslPos[0] << " N " << dataPacket.mslPos[1] << " U " << dataPacket.mslPos[2] << " RANGE " << dataPacket.mslRange << " MACH " << dataPacket.mslMach << endl;
	cout << setprecision(6) << "MISS DISTANCE " << dataPacket.missDistance << " >>> FORWARD, LEFT, UP, MISS DISTANCE " << dataPacket.forwardLeftUpMslToIntercept[0] << " " << dataPacket.forwardLeftUpMslToIntercept[1] << " " << dataPacket.forwardLeftUpMslToIntercept[2] << endl;
	cout << "SIMULATION RESULT: " << dataPacket.lethality << endl;
	auto wallClockEnd = chrono::high_resolution_clock::now();
	auto simRealRunTime = chrono::duration_cast<chrono::milliseconds>(wallClockEnd - wallClockStart);
	cout << "SIMULATION RUN TIME :" << simRealRunTime.count() / 1000.0 << " SECONDS" << endl;
	cout << "\n" << endl;

}

int main()
{
	fly();
	return 0;
}