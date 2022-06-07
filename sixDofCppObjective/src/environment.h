#include "util.h"

class environment
{

	public:

		// CONSTRUCTOR
		environment(mslDataPacket *dataPacket);

		// UPDATE FUNCTION
		void update(mslDataPacket *dataPacket);

		double rho;
		double speedOfSound;
		double pressure;
		double temperature;

};