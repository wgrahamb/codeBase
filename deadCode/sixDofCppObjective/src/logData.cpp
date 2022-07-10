#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <math.h>
#include <vector>
#include <map>
#include <algorithm>

#include "logData.h"
#include "util.h"

using namespace std;

logData::logData(mslDataPacket *dataPacket)
{
	std::cout << "DATA LOG INITIATED" << std::endl;

	logFile.open("log.txt");
	logFile << fixed << setprecision(10) << "tof posE posN posU tgtE tgtN tgtU normAch pitchRate thetaRate alpha theta sideAch yawRate psiRate beta psi rollAngleComm rollRate phiRate phi" << endl;

}

void logData::update(mslDataPacket *dataPacket)
{
	logFile << fixed << setprecision(10) <<
	dataPacket->mslTof << " " <<
	dataPacket->mslPos[0] << " " <<
	dataPacket->mslPos[1] << " " <<
	dataPacket->mslPos[2] << " " <<
	dataPacket->mslWayPoint[0] << " " <<
	dataPacket->mslWayPoint[1] << " " <<
	dataPacket->mslWayPoint[2] << " " <<
	dataPacket->mslBodyAcc[2] / grav << " " <<
	dataPacket->mslRate[1] << " " <<
	dataPacket->mslEulerDot[1] << " " <<
	dataPacket->mslAlpha << " " <<
	dataPacket->mslEuler[1] << " " <<
	dataPacket->mslBodyAcc[1] / grav << " " <<
	dataPacket->mslRate[2] << " " <<
	dataPacket->mslEulerDot[2] << " " <<
	dataPacket->mslBeta << " " <<
	dataPacket->mslEuler[2] << " " <<
	0.0 << " " <<
	dataPacket->mslRate[0] << " " <<
	dataPacket->mslEulerDot[0] << " " <<
	dataPacket->mslEuler[0] << "\n";
}