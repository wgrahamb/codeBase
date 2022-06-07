#include "iostream"
#include "flightComputer.h"
#include "util.h"

flightComputer::flightComputer(mslDataPacket *dataPacket, double time_step, double integration_step, double max_time)
{
	std::cout << "FLIGHT COMPUTER INITIATED" << std::endl;
	timeStep = time_step;
	integrationStep = integration_step;
	maxTime = max_time;
	dataPacket->mslTimeStep = timeStep;
	dataPacket->mslIntegrationStep = integrationStep;
	dataPacket->mslMaxTime = maxTime;
	dataPacket->mslTof = 0.0;
}

void flightComputer::update(mslDataPacket *dataPacket)
{
	dataPacket->mslTof += timeStep;
}