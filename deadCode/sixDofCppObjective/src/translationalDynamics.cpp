#include "iostream"
#include "translationalDynamics.h"
#include "util.h"

translationalDynamics::translationalDynamics(mslDataPacket *dataPacket)
{
	std::cout << "TRANSLATIONAL DYNAMICS INITIATED" << std::endl;
}

void translationalDynamics::update(mslDataPacket *dataPacket)
{

	double gravLocalVec[3], gravBodyVec[3];
	gravLocalVec[0] = 0.0;
	gravLocalVec[1] = 0.0;
	gravLocalVec[2] = -1 * grav;
	threeByThreeTimesThreeByOne(dataPacket->mslLocalOrient, gravLocalVec, gravBodyVec);

	double axialForce = dataPacket->mslThrust - dataPacket->CX * dataPacket->mslDynamicPressure * dataPacket->mslRefArea + gravBodyVec[0] * dataPacket->mslMass;
	double sideForce = dataPacket->CY * dataPacket->mslDynamicPressure * dataPacket->mslRefArea + gravBodyVec[1] * dataPacket->mslMass;
	double normalForce = dataPacket->CZ * dataPacket->mslDynamicPressure * dataPacket->mslRefArea + gravBodyVec[2] * dataPacket->mslMass;

	dataPacket->mslBodyAcc[0] = axialForce / dataPacket->mslMass - (dataPacket->mslRate[1] * dataPacket->mslBodyVel[2] - dataPacket->mslRate[2] * dataPacket->mslBodyVel[1]);
	dataPacket->mslBodyAcc[1] = sideForce / dataPacket->mslMass - (dataPacket->mslRate[2] * dataPacket->mslBodyVel[0] - dataPacket->mslRate[0] * dataPacket->mslBodyVel[2]);
	dataPacket->mslBodyAcc[2] = normalForce / dataPacket->mslMass - (dataPacket->mslRate[0] * dataPacket->mslBodyVel[1] - dataPacket->mslRate[1] * dataPacket->mslBodyVel[0]);

	oneByThreeTimesThreeByThree(dataPacket->mslBodyAcc, dataPacket->mslLocalOrient, dataPacket->mslAcc);
}