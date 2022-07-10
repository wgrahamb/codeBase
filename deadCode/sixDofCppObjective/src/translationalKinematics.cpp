#include "iostream"
#include "translationalKinematics.h"
#include "util.h"

translationalKinematics::translationalKinematics(mslDataPacket *dataPacket)
{
	std::cout << "TRANSLATIONAL KINEMATICS INITIATED" << std::endl;
}

void translationalKinematics::update(mslDataPacket *dataPacket)
{

	double deltaVel[3];
	multiplyVectorTimesScalar(dataPacket->mslTimeStep, dataPacket->mslAcc, deltaVel);
	double newMslVel[3];
	addTwoVectors(dataPacket->mslVel, deltaVel, newMslVel);
	dataPacket->mslVel[0] = newMslVel[0];
	dataPacket->mslVel[1] = newMslVel[1];
	dataPacket->mslVel[2] = newMslVel[2];
	double deltaPos[3];
	multiplyVectorTimesScalar(dataPacket->mslTimeStep, dataPacket->mslVel, deltaPos);
	double newMslPos[3];
	addTwoVectors(dataPacket->mslPos, deltaPos, newMslPos);
	dataPacket->mslPos[0] = newMslPos[0];
	dataPacket->mslPos[1] = newMslPos[1];
	dataPacket->mslPos[2] = newMslPos[2];
	double distanceTravelled;
	magnitude(deltaPos, distanceTravelled);
	dataPacket->mslRange += distanceTravelled;

}