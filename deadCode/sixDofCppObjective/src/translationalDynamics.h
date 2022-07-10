#include "util.h"

class translationalDynamics
{

	public:

		// CONSTRUCTOR
		translationalDynamics(mslDataPacket *dataPacket);

		// UPDATE FUNCTION
		void update(mslDataPacket *dataPacket);

};