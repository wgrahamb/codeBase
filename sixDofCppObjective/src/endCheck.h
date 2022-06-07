#include "util.h"
#include "string"

class endCheck
{

	public:

		// CONSTRUCTOR
		endCheck(mslDataPacket *dataPacket);

		// STRUCT FOR END CHECKS
		struct endChecks {
			std::string successfulIntercept;
			std::string flying;
			std::string groundCollision;
			std::string pointOfClosestApproachPassed;
			std::string notANumber;
			std::string maxTimeExceeded;
			std::string forcedSimTermination;
		};
		endChecks ec;

		// UPDATE FUNCTION
		void update(mslDataPacket *dataPacket);

};