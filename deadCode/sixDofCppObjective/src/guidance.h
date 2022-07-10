#include "util.h"

class guidance
{

	public:

		// CONSTRUCTOR
		guidance(mslDataPacket *dataPacket, double pro_nav_gain, double max_accel_allow);
		double proNavGain;
		double maxAccelAllow;
		double maxAccel;

		// UPDATE FUNCTION
		void update(mslDataPacket *dataPacket);
		double forwardLeftUpMslToIntercept[3], normCommand, sideCommand;

};