#include "iostream"
#include "math.h"

#include "rotationalKinematics.h"
#include "util.h"

rotationalKinematics::rotationalKinematics(mslDataPacket *dataPacket)
{
	std::cout << "ROTATIONAL KINEMATICS INITIATED" << std::endl;
}

void rotationalKinematics::update(mslDataPacket *dataPacket)
{
	double newPhiDot = dataPacket->mslRate[0] + (dataPacket->mslRate[1] * sin(dataPacket->mslEuler[0]) + dataPacket->mslRate[2] * cos(dataPacket->mslEuler[0])) * tan(dataPacket->mslEuler[1]);
	double newPhi = integrate(newPhiDot, dataPacket->mslEulerDot[0], dataPacket->mslEuler[0], dataPacket->mslIntegrationStep);
	double phiDelta = dataPacket->mslIntegrationStep * (dataPacket->mslEulerDot[0] + newPhiDot) / 2;
	// double newPhi = rk4(dataPacket->mslEuler[0], phiDelta, dataPacket->mslIntegrationStep);
	dataPacket->mslEulerDot[0] = newPhiDot;



	double newThetaDot = dataPacket->mslRate[1] * cos(dataPacket->mslEuler[0]) - dataPacket->mslRate[2] * sin(dataPacket->mslEuler[0]);
	double newTheta = integrate(newThetaDot, dataPacket->mslEulerDot[1], dataPacket->mslEuler[1], dataPacket->mslIntegrationStep);
	double thetaDelta = dataPacket->mslIntegrationStep * (dataPacket->mslEulerDot[1] + newThetaDot) / 2;
	// double newTheta = rk4(dataPacket->mslEuler[1], thetaDelta, dataPacket->mslIntegrationStep);
	dataPacket->mslEulerDot[1] = newThetaDot;



	double newPsiDot = -1 * (dataPacket->mslRate[1] * sin(dataPacket->mslEuler[0]) + dataPacket->mslRate[2] * cos(dataPacket->mslEuler[0])) / cos(dataPacket->mslEuler[1]);
	double newPsi = integrate(newPsiDot, dataPacket->mslEulerDot[2], dataPacket->mslEuler[2], dataPacket->mslIntegrationStep);
	double psiDelta = dataPacket->mslIntegrationStep * (dataPacket->mslEulerDot[2] + newPsiDot) / 2;
	// double newPsi = rk4(dataPacket->mslEuler[2], psiDelta, dataPacket->mslIntegrationStep);
	dataPacket->mslEulerDot[2] = newPsiDot;



	dataPacket->mslEuler[0] = newPhi;
	dataPacket->mslEuler[1] = newTheta;
	dataPacket->mslEuler[2] = newPsi;

	eulerAnglesToLocalOrientation(newPhi, -newTheta, newPsi, dataPacket->mslLocalOrient);
}