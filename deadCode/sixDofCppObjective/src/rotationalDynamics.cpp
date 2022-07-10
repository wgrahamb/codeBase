#include "iostream"
#include "rotationalDynamics.h"
#include "util.h"

rotationalDynamics::rotationalDynamics(mslDataPacket *dataPacket)
{
	std::cout << "ROTATIONAL DYNAMICS INITIATED" << std::endl;
}

void rotationalDynamics::update(mslDataPacket *dataPacket)
{
	double rollMoment = dataPacket->CL * dataPacket->mslDynamicPressure * dataPacket->mslRefArea * dataPacket->mslRefDiam;
	double pitchMoment = dataPacket->CM * dataPacket->mslDynamicPressure * dataPacket->mslRefArea * dataPacket->mslRefDiam;
	double yawMoment = dataPacket->CN * dataPacket->mslDynamicPressure * dataPacket->mslRefArea * dataPacket->mslRefDiam;



	double newRollRateDot = rollMoment / dataPacket->mslAMoi;
	double newRollRate = integrate(newRollRateDot, dataPacket->mslRateDot[0], dataPacket->mslRate[0], dataPacket->mslIntegrationStep);
	// double rollRateDelta = dataPacket->mslIntegrationStep * (newRollRateDot + dataPacket->mslRateDot[0]) / 2.0;
	// double newRollRate = rk4(dataPacket->mslRate[0], rollRateDelta, dataPacket->mslIntegrationStep);
	dataPacket->mslRateDot[0] = newRollRateDot;



	double newPitchRateDot = (1 / dataPacket->mslTMoi) * ((dataPacket->mslTMoi - dataPacket->mslAMoi) * dataPacket->mslRate[0] * dataPacket->mslRate[2] + pitchMoment);
	double newPitchRate = integrate(newPitchRateDot, dataPacket->mslRateDot[1], dataPacket->mslRate[1], dataPacket->mslIntegrationStep);
	// double pitchRateDelta = dataPacket->mslIntegrationStep * (newPitchRateDot + dataPacket->mslRateDot[1]) / 2.0;
	// double newPitchRate = rk4(dataPacket->mslRate[1], pitchRateDelta, dataPacket->mslIntegrationStep);
	dataPacket->mslRateDot[1] = newPitchRateDot;



	double newYawRateDot = (1 / dataPacket->mslTMoi) * ((dataPacket->mslAMoi - dataPacket->mslTMoi) * dataPacket->mslRate[0] * dataPacket->mslRate[1] + yawMoment);
	double newYawRate = integrate(newYawRateDot, dataPacket->mslRateDot[2], dataPacket->mslRate[2], dataPacket->mslIntegrationStep);
	// double yawRateDelta = dataPacket->mslIntegrationStep * (newYawRateDot + dataPacket->mslRateDot[2]) / 2.0;
	// double newYawRate = rk4(dataPacket->mslRate[2], yawRateDelta, dataPacket->mslIntegrationStep);
	dataPacket->mslRateDot[2] = newYawRateDot;



	dataPacket->mslRate[0] = newRollRate;
	dataPacket->mslRate[1] = newPitchRate;
	dataPacket->mslRate[2] = newYawRate;
}