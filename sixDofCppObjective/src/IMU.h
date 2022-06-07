#include "util.h"

class IMU
{

	public:

		// CONSTRUCTOR
		IMU(mslDataPacket *dataPacket, double phi, double tht, double psi);

		// UPDATE FUNCTION
		void update(mslDataPacket *dataPacket);

};