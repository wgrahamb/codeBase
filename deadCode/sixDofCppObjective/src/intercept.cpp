#include "iostream"
#include "intercept.h"
#include "util.h"

intercept::intercept(mslDataPacket *dataPacket)
{
	std::cout << "INTERCEPT INITIATED" << std::endl;
	magnitude(dataPacket->forwardLeftUpMslToIntercept, dataPacket->missDistance);
}

void intercept::update(mslDataPacket *dataPacket)
{
	magnitude(dataPacket->forwardLeftUpMslToIntercept, dataPacket->missDistance);
}