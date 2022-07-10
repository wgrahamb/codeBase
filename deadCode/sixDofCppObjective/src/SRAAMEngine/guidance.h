#include "../util.h"

class guidance
{

	public:

		// CONSTRUCTOR
		guidance(mslDataPacket *dataPacket);
		double proNavGain;
		double maxAccelAllow;
		double maxAccel;

		// UPDATE FUNCTION
		void update(mslDataPacket *dataPacket, double max_accel);
		double forwardLeftUpMslToIntercept[3], normCommand, sideCommand;

};