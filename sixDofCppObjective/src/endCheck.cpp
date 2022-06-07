#include "iostream"
#include "math.h"
#include "algorithm"
#include "string"

#include "endCheck.h"
#include "util.h"

endCheck::endCheck(mslDataPacket *dataPacket)
{
	std::cout << "END CHECK INITIATED" << std::endl;

	// END CHECK
	ec.successfulIntercept = "SUCCESSFUL INTERCEPT";
	ec.flying = "IN FLIGHT";
	ec.groundCollision = "BELOW GROUND";
	ec.pointOfClosestApproachPassed = "POINT OF CLOSEST APPROACH PASSED";
	ec.maxTimeExceeded = "MAX TIME EXCEEDED";
	ec.notANumber = "NAN";
	ec.forcedSimTermination = "FORCED SIM TERMINATION";
	dataPacket->lethality = ec.flying; // STATUS
	dataPacket->fly = true;
}

void endCheck::update(mslDataPacket *dataPacket)
{
	if (dataPacket->mslPos[2] < 0)
	{
		dataPacket->lethality = ec.groundCollision;
		dataPacket->fly = false;
	}
	else if (dataPacket->missDistance < 2.0)
	{
		dataPacket->lethality = ec.successfulIntercept;
		dataPacket->fly = false;
	}
	else if (dataPacket->forwardLeftUpMslToIntercept[0] < 0.0)
	{
		dataPacket->lethality = ec.pointOfClosestApproachPassed;
		dataPacket->fly = false;
	}
	else if (isnan(dataPacket->mslPos[0]))
	{
		dataPacket->lethality = ec.notANumber;
		dataPacket->fly = false;
	}
	else if (dataPacket->mslTof > dataPacket->mslMaxTime)
	{
		dataPacket->lethality = ec.maxTimeExceeded;
		dataPacket->fly = false;
	}
	else if (dataPacket->lethality == ec.forcedSimTermination)
	{
		dataPacket->fly = false;
	}
}