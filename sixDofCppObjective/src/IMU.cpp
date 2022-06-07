#include "iostream"
#include "math.h"

#include "IMU.h"
#include "util.h"

IMU::IMU(mslDataPacket *dataPacket, double phi, double tht, double psi)
{
	std::cout << "INERTIAL MEASUREMENT UNIT INITIATED" << std::endl;
	eulerAnglesToLocalOrientation(phi, -tht, psi, dataPacket->mslLocalOrient);
	dataPacket->mslPos[0] = 0.0;
	dataPacket->mslPos[1] = 0.0;
	dataPacket->mslPos[2] = 0.0;
	dataPacket->mslRange = 0.0;
	dataPacket->mslVel[0] = dataPacket->mslLocalOrient[0][0];
	dataPacket->mslVel[1] = dataPacket->mslLocalOrient[0][1];
	dataPacket->mslVel[2] = dataPacket->mslLocalOrient[0][2];
	dataPacket->mslBodyVel[0] = 1.0;
	dataPacket->mslBodyVel[1] = 0.0;
	dataPacket->mslBodyVel[2] = 0.0;
	magnitude(dataPacket->mslBodyVel, dataPacket->mslSpeed);
	dataPacket->mslMach = dataPacket->mslSpeed / dataPacket->mslSpeedOfSound;
	dataPacket->mslAcc[0] = 0.0;
	dataPacket->mslAcc[1] = 0.0;
	dataPacket->mslAcc[2] = 0.0;
	dataPacket->mslBodyAcc[0] = 0.0;
	dataPacket->mslBodyAcc[1] = 0.0;
	dataPacket->mslBodyAcc[2] = 0.0;
	dataPacket->mslAlpha = 0.0;
	dataPacket->mslBeta = 0.0;
	dataPacket->mslEuler[0] = phi;
	dataPacket->mslEuler[1] = tht;
	dataPacket->mslEuler[2] = psi;
	dataPacket->mslEulerDot[0] = 0.0;
	dataPacket->mslEulerDot[1] = 0.0;
	dataPacket->mslEulerDot[2] = 0.0;
	dataPacket->mslRate[0] = 0.0;
	dataPacket->mslRate[1] = 0.0;
	dataPacket->mslRate[2] = 0.0;
	dataPacket->mslRateDot[0] = 0.0;
	dataPacket->mslRateDot[1] = 0.0;
	dataPacket->mslRateDot[2] = 0.0;

}

void IMU::update(mslDataPacket *dataPacket)
{
	threeByThreeTimesThreeByOne(dataPacket->mslLocalOrient, dataPacket->mslVel, dataPacket->mslBodyVel);
	magnitude(dataPacket->mslBodyVel, dataPacket->mslSpeed);
	dataPacket->mslAlpha = -1 * atan2(dataPacket->mslBodyVel[2], dataPacket->mslBodyVel[0]);
	dataPacket->mslBeta = atan2(dataPacket->mslBodyVel[1], dataPacket->mslBodyVel[0]);
	dataPacket->mslMach = dataPacket->mslSpeed / dataPacket->mslSpeedOfSound;
}