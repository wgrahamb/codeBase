#include "util.h"

class seeker
{

	public:

		// CONSTRUCTOR
		seeker(mslDataPacket *dataPacket, double tgtE, double tgtN, double tgtU);

		double seekerPitch, seekerYaw;
		double seekerLocalOrient[3][3];
		double seekerPitchErr, seekerYawErr;
		double forwardLeftUpMslToIntercept[3];
		double gk, zetak, wnk;
		double wlr, wlrd, wlr1, wlr1d, wlr2, wlr2d;
		double wlq, wlqd, wlq1, wlq1d, wlq2, wlq2d;

		// UPDATE FUNCTION
		void update(mslDataPacket *dataPacket);

};