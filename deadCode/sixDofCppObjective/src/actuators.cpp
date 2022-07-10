#include "iostream"
#include "actuators.h"
#include "util.h"

actuators::actuators(mslDataPacket *dataPacket)
{

	std::cout << "ACTUATORS INITIATED" << std::endl;

	// FIN ACTUATION
	pitchFinDefl = 0.0; // RADIANS
	yawFinDefl = 0.0; // RADIANS
	rollFinDefl = 0.0; // RADIANS
	DEL1 = 0.0; // RADIANS >>> FIN DEFLECTION
	DEL1D = 0.0; // RADIANS >>> FIN POSITION DERIVED
	DEL1DOT = 0.0; // RADIANS PER SECOND >>> FIN RATE
	DEL1DOTDOT = 0.0; // RADIANS PER S^2 >>> FIN RATE DERIVED
	DEL2 = 0.0; // RADIANS >>> FIN DEFLECTION
	DEL2D = 0.0; // RADIANS >>> FIN POSITION DERIVED
	DEL2DOT = 0.0; // RADIANS PER SECOND >>> FIN RATE
	DEL2DOTDOT = 0.0; // RADIANS PER S^2 >>> FIN RATE DERIVED
	DEL3 = 0.0; // RADIANS >>> FIN DEFLECTION
	DEL3D = 0.0; // RADIANS >>> FIN POSITION DERIVED
	DEL3DOT = 0.0; // RADIANS PER SECOND >>> FIN RATE
	DEL3DOTDOT = 0.0; // RADIANS PER S^2 >>> FIN RATE DERIVED
	DEL4 = 0.0; // RADIANS >>> FIN DEFLECTION
	DEL4D = 0.0; // RADIANS >>> FIN POSITION DERIVED
	DEL4DOT = 0.0; // RADIANS PER SECOND >>> FIN RATE
	DEL4DOTDOT = 0.0; // RADIANS PER S^2 >>> FIN RATE DERIVED
	finRadianLimit = 0.4887; // RADIANS
	finRateRadianLimit = 10.472; // RADIANS PER SECOND
	WNACT = 100; // NATURAL FREQUENCY OF ACTUATOR >>> RADIANS PER SECOND
	ZETACT = 0.7; // DAMPING OF ACTUATOR

}

void actuators::update(mslDataPacket *dataPacket)
{
	double DEL1C = -dataPacket->rollFinComm + dataPacket->pitchFinComm - dataPacket->yawFinComm;
	double DEL2C = -dataPacket->rollFinComm + dataPacket->pitchFinComm + dataPacket->yawFinComm;
	double DEL3C = dataPacket->rollFinComm + dataPacket->pitchFinComm - dataPacket->yawFinComm;
	double DEL4C = dataPacket->rollFinComm + dataPacket->pitchFinComm + dataPacket->yawFinComm;
	int flag;

	// FIN ONE
	flag = 0;
	if (abs(DEL1) > finRadianLimit) {
		if (DEL1 < 0) {
			DEL1 = -1 * finRadianLimit;
		}
		else if (DEL1 > 0) {
			DEL1 = finRadianLimit;
		}
		if ((DEL1 * DEL1DOT) > 0) {
			DEL1DOT = 0;
		}
	}
	if (abs(DEL1DOT) > finRateRadianLimit) {
		flag = 1;
		if (DEL1DOT < 0) {
			DEL1DOT = -1 * finRateRadianLimit;
		}
		else if (DEL1DOT > 0) {
			DEL1DOT = finRateRadianLimit;
		}
	}
	double DEL1D_NEW = DEL1DOT;
	double DEL1_NEW = integrate(DEL1D_NEW, DEL1D, DEL1, dataPacket->mslIntegrationStep);
	DEL1 = DEL1_NEW;
	DEL1D = DEL1D_NEW;
	double EDX1 = DEL1C - DEL1;
	double DEL1DOTDOT_NEW = WNACT * WNACT * EDX1 - 2 * ZETACT * WNACT * DEL1D;
	double DEL1DOT_NEW = integrate(DEL1DOTDOT_NEW, DEL1DOTDOT, DEL1DOT, dataPacket->mslIntegrationStep);
	DEL1DOT = DEL1DOT_NEW;
	DEL1DOTDOT = DEL1DOTDOT_NEW;
	if (flag == 1 and (DEL1DOT * DEL1DOTDOT) > 0) {
		DEL1DOTDOT = 0.0;
	}

	// FIN TWO
	flag = 0;
	if (abs(DEL2) > finRadianLimit) {
		if (DEL2 < 0) {
			DEL2 = -1 * finRadianLimit;
		}
		else if (DEL2 > 0) {
			DEL2 = finRadianLimit;
		}
		if ((DEL2 * DEL2DOT) > 0) {
			DEL2DOT = 0;
		}
	}
	if (abs(DEL2DOT) > finRateRadianLimit) {
		flag = 1;
		if (DEL2DOT < 0) {
			DEL2DOT = -1 * finRateRadianLimit;
		}
		else if (DEL2DOT > 0) {
			DEL2DOT = finRateRadianLimit;
		}
	}
	double DEL2D_NEW = DEL2DOT;
	double DEL2_NEW = integrate(DEL2D_NEW, DEL2D, DEL2, dataPacket->mslIntegrationStep);
	DEL2 = DEL2_NEW;
	DEL2D = DEL2D_NEW;
	double EDX2 = DEL2C - DEL2;
	double DEL2DOTDOT_NEW = WNACT * WNACT * EDX2 - 2 * ZETACT * WNACT * DEL2D;
	double DEL2DOT_NEW = integrate(DEL2DOTDOT_NEW, DEL2DOTDOT, DEL2DOT, dataPacket->mslIntegrationStep);
	DEL2DOT = DEL2DOT_NEW;
	DEL2DOTDOT = DEL2DOTDOT_NEW;
	if (flag == 1 and (DEL2DOT * DEL2DOTDOT) > 0) {
		DEL2DOTDOT = 0.0;
	}

	// FIN THREE
	flag = 0;
	if (abs(DEL3) > finRadianLimit) {
		if (DEL3 < 0) {
			DEL3 = -1 * finRadianLimit;
		}
		else if (DEL3 > 0) {
			DEL3 = finRadianLimit;
		}
		if ((DEL3 * DEL3DOT) > 0) {
			DEL3DOT = 0;
		}
	}
	if (abs(DEL3DOT) > finRateRadianLimit) {
		flag = 1;
		if (DEL3DOT < 0) {
			DEL3DOT = -1 * finRateRadianLimit;
		}
		else if (DEL3DOT > 0) {
			DEL3DOT = finRateRadianLimit;
		}
	}
	double DEL3D_NEW = DEL3DOT;
	double DEL3_NEW = integrate(DEL3D_NEW, DEL3D, DEL3, dataPacket->mslIntegrationStep);
	DEL3 = DEL3_NEW;
	DEL3D = DEL3D_NEW;
	double EDX3 = DEL3C - DEL3;
	double DEL3DOTDOT_NEW = WNACT * WNACT * EDX3 - 2 * ZETACT * WNACT * DEL3D;
	double DEL3DOT_NEW = integrate(DEL3DOTDOT_NEW, DEL3DOTDOT, DEL3DOT, dataPacket->mslIntegrationStep);
	DEL3DOT = DEL3DOT_NEW;
	DEL3DOTDOT = DEL3DOTDOT_NEW;
	if (flag == 1 and (DEL3DOT * DEL3DOTDOT) > 0) {
		DEL3DOTDOT = 0.0;
	}

	// FIN FOUR
	flag = 0;
	if (abs(DEL4) > finRadianLimit) {
		if (DEL4 < 0) {
			DEL4 = -1 * finRadianLimit;
		}
		else if (DEL4 > 0) {
			DEL4 = finRadianLimit;
		}
		if ((DEL4 * DEL4DOT) > 0) {
			DEL4DOT = 0;
		}
	}
	if (abs(DEL4DOT) > finRateRadianLimit) {
		flag = 1;
		if (DEL4DOT < 0) {
			DEL4DOT = -1 * finRateRadianLimit;
		}
		else if (DEL4DOT > 0) {
			DEL4DOT = finRateRadianLimit;
		}
	}
	double DEL4D_NEW = DEL4DOT;
	double DEL4_NEW = integrate(DEL4D_NEW, DEL4D, DEL4, dataPacket->mslIntegrationStep);
	DEL4 = DEL4_NEW;
	DEL4D = DEL4D_NEW;
	double EDX4 = DEL4C - DEL4;
	double DEL4DOTDOT_NEW = WNACT * WNACT * EDX4 - 2 * ZETACT * WNACT * DEL4D;
	double DEL4DOT_NEW = integrate(DEL4DOTDOT_NEW, DEL4DOTDOT, DEL4DOT, dataPacket->mslIntegrationStep);
	DEL4DOT = DEL4DOT_NEW;
	DEL4DOTDOT = DEL4DOTDOT_NEW;
	if (flag == 1 and (DEL4DOT * DEL4DOTDOT) > 0) {
		DEL4DOTDOT = 0.0;
	}

	rollFinDefl = (-DEL1 - DEL2 + DEL3 + DEL4) / 4;
	pitchFinDefl = (DEL1 + DEL2 + DEL3 + DEL4) / 4;
	yawFinDefl = (-DEL1 + DEL2 - DEL3 + DEL4) / 4;

	dataPacket->pitchFinDefl = pitchFinDefl;
	dataPacket->yawFinDefl = yawFinDefl;
	dataPacket->rollFinDefl = rollFinDefl;

}