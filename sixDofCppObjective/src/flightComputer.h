#include "util.h"

class flightComputer
{

	public:

		// CONSTRUCTOR
		flightComputer(mslDataPacket *dataPacket, double time_step, double integration_step, double max_time);
		double timeStep;
		double integrationStep;
		double maxTime;

		// UPDATE FUNCTION
		void update(mslDataPacket *dataPacket);

};