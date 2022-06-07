#include "util.h"

class control
{

	public:

		// CONSTRUCTOR
		control(mslDataPacket *dataPacket);
		double zetlagr, wrcl, zrcl, yy, yyd, zz, zzd, maxDefl, pitchFinComm, yawFinComm, rollFinComm, rollAngleComm;

		// UPDATE FUNCTION
		void update(mslDataPacket *dataPacket);

};