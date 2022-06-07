#include "iostream"
#include "math.h"

#include "guidance.h"
#include "util.h"

guidance::guidance(mslDataPacket *dataPacket, double pro_nav_gain, double max_accel_allow)
{
	std::cout << "GUIDANCE INITIATED" << std::endl;
	proNavGain = pro_nav_gain;
	maxAccelAllow = max_accel_allow;
	maxAccel = maxAccelAllow;
	dataPacket->mslMaxAccel = maxAccel;
	dataPacket->mslMaxAccelAllow = max_accel_allow;
	dataPacket->normCommand = 0.0;
	dataPacket->sideCommand = 0.0;
}

void guidance::update(mslDataPacket *dataPacket)
{

	forwardLeftUpMslToIntercept[0] = dataPacket->forwardLeftUpMslToIntercept[0];
	forwardLeftUpMslToIntercept[1] = dataPacket->forwardLeftUpMslToIntercept[1];
	forwardLeftUpMslToIntercept[2] = dataPacket->forwardLeftUpMslToIntercept[2];
	maxAccel = dataPacket->mslMaxAccel;

	double forwardLeftUpMslToInterceptU[3];
	double forwardLeftUpMslToInterceptMag;
	unitVec(forwardLeftUpMslToIntercept, forwardLeftUpMslToInterceptU);
	magnitude(forwardLeftUpMslToIntercept, forwardLeftUpMslToInterceptMag);
	double relVel[3];
	relVel[0] = dataPacket->mslVel[0] * -1;
	relVel[1] = dataPacket->mslVel[1] * -1;
	relVel[2] = dataPacket->mslVel[2] * -1;
	double relVelMag;
	magnitude(relVel, relVelMag);
	double closingVel[3];
	threeByThreeTimesThreeByOne(dataPacket->mslLocalOrient, relVel, closingVel);
	double TEMP1[3], TEMP2;
	crossProductTwoVectors(forwardLeftUpMslToIntercept, closingVel, TEMP1);
	dotProductTwoVectors(forwardLeftUpMslToIntercept, forwardLeftUpMslToIntercept, TEMP2);
	double lineOfSightRate[3];
	lineOfSightRate[0] = TEMP1[0] / TEMP2;
	lineOfSightRate[1] = TEMP1[1] / TEMP2;
	lineOfSightRate[2] = TEMP1[2] / TEMP2;
	double command[3];
	double TEMP3[3];
	TEMP3[0] = -1 * proNavGain * relVelMag * forwardLeftUpMslToInterceptU[0];
	TEMP3[1] = -1 * proNavGain * relVelMag * forwardLeftUpMslToInterceptU[1];
	TEMP3[2] = -1 * proNavGain * relVelMag * forwardLeftUpMslToInterceptU[2];
	crossProductTwoVectors(TEMP3, lineOfSightRate, command);
	normCommand = command[2];
	sideCommand = command[1];
	double trigRatio = atan2(normCommand, sideCommand);
	double accMag = sqrt(normCommand * normCommand + sideCommand * sideCommand);
	if (accMag > maxAccel)
	{
		accMag = maxAccel;
	}
	sideCommand = accMag * cos(trigRatio);
	normCommand = accMag * sin(trigRatio);

	dataPacket->normCommand = normCommand;
	dataPacket->sideCommand = sideCommand;
}