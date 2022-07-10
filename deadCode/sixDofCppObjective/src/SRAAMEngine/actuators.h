#include "../util.h"

class actuators
{

	public:

		// CONSTRUCTOR
		actuators(mslDataPacket *dataPacket);

		double pitchFinDefl, yawFinDefl, rollFinDefl;
		double DEL1, DEL1D, DEL1DOT, DEL1DOTDOT;
		double DEL2, DEL2D, DEL2DOT, DEL2DOTDOT;
		double DEL3, DEL3D, DEL3DOT, DEL3DOTDOT;
		double DEL4, DEL4D, DEL4DOT, DEL4DOTDOT;
		double finRadianLimit, finRateRadianLimit, WNACT, ZETACT;

		// UPDATE FUNCTION
		void update(
			mslDataPacket *dataPacket,
			double rollFinComm,
			double pitchFinComm,
			double yawFinComm
		);

};