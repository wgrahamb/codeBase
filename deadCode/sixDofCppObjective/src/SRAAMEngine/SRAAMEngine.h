#include "../util.h"

#include "guidance.h"
#include "control.h"
#include "actuators.h"
#include "aerodynamics.h"
#include "massAndMotor.h"

class SRAAMEngine
{
	public:
	
		SRAAMEngine(mslDataPacket *dp);
		void update(mslDataPacket *dp);

		guidance *guidanceObj;
		double normCommand, sideCommand;

		control *controlObj;
		double rollFinComm, pitchFinComm, yawFinComm;

		actuators *actuatorsObj;
		double rollFinDefl, pitchFinDefl, yawFinDefl;

		aerodynamics *aerodynamicsObj;
		double CNA, CND, CMA, CMD, CMQ, CLP, CLD, maxAccel;

		massAndMotor *massAndMotorObj;
		double launchCg, cgFromNose;

};