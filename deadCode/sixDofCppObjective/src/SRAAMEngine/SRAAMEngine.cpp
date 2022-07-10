#include "../util.h"
#include "SRAAMEngine.h"

SRAAMEngine::SRAAMEngine(mslDataPacket *dp)
{

	guidanceObj = new guidance(dp);
	controlObj = new control(dp);
	actuatorsObj = new actuators(dp);
	aerodynamicsObj = new aerodynamics(dp);
	massAndMotorObj = new massAndMotor(dp);

}

void SRAAMEngine::update(mslDataPacket *dp)
{
	guidanceObj->update(dp, maxAccel);
	normCommand = guidanceObj->normCommand;
	sideCommand = guidanceObj->sideCommand;

	controlObj->update(dp, CNA, CND, CMA, CMD, CMQ, CLP, CLD, normCommand, sideCommand);
	rollFinComm = controlObj->rollFinComm;
	pitchFinComm = controlObj->pitchFinComm;
	yawFinComm = controlObj->yawFinComm;

	actuatorsObj->update(dp, rollFinComm, pitchFinComm, yawFinComm);
	rollFinDefl = actuatorsObj->rollFinDefl;
	pitchFinDefl = actuatorsObj->pitchFinDefl;
	yawFinDefl = actuatorsObj->yawFinDefl;

	aerodynamicsObj->update(dp, rollFinDefl, pitchFinDefl, yawFinDefl, launchCg, cgFromNose, guidanceObj->maxAccelAllow);
	CNA = aerodynamicsObj->CNA;
	CND = aerodynamicsObj->CND;
	CMA = aerodynamicsObj->CMA;
	CMD = aerodynamicsObj->CMD;
	CMQ = aerodynamicsObj->CMQ;
	CLP = aerodynamicsObj->CLP;
	CLD = aerodynamicsObj->CLD;
	maxAccel = aerodynamicsObj->maxAccel;

	massAndMotorObj->update(dp);
	launchCg = massAndMotorObj->launchCg;
	cgFromNose = massAndMotorObj->cgFromNose;

}