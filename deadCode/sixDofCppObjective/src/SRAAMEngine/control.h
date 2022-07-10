#include "../util.h"

class control
{

	public:

		// CONSTRUCTOR
		control(mslDataPacket *dataPacket);
		double zetlagr, wrcl, zrcl, yy, yyd, zz, zzd, maxDefl, pitchFinComm, yawFinComm, rollFinComm, rollAngleComm;

		// UPDATE FUNCTION
		void update(
			mslDataPacket *dataPacket,
			double CNA,
			double CND,
			double CMA,
			double CMD,
			double CMQ,
			double CLP,
			double CLD,
			double normCommand,
			double sideCommand
		);

};