#include "iostream"
#include "math.h"

#include "control.h"
#include "util.h"

control::control(mslDataPacket *dataPacket)
{

	std::cout << "CONTROL INITIATED" << std::endl;

	// CONTROL
	zetlagr = 0.6; // DAMPING OF CLOSED RATE LOOP >>> ND
	wrcl = 20; // FREQUENCY OF ROLL CLOSED LOOP COMPLEX POLE >>> RADIANS PER SECOND
	zrcl = 0.9; //DAMPING OF ROLL CLOSED LOOP POLE >>> ND
	yy = 0.0; // METERS PER SECOND >>> YAW FEED FORWARD INTEGRATION
	yyd = 0.0; // METERS PER SECOND >>> YAW FEED FORWARD DERIVATIVE
	zz = 0.0; // METERS PER SECOND >>> PITCH FEED FORWARD INTEGRATION
	zzd = 0.0; // METERS PER SECOND >>> PITCH FEED FORWARD DERIVATIVE
	maxDefl = 28.0; // DEGREES
	pitchFinComm = 0.0; // RADIANS
	yawFinComm = 0.0; // RADIANS
	rollFinComm = 0.0; // RADIANS
	rollAngleComm = 0.0; // RADIANS

	dataPacket->pitchFinComm = pitchFinComm;
	dataPacket->yawFinComm = yawFinComm;
	dataPacket->rollFinComm = rollFinComm;

}

void control::update(mslDataPacket *dataPacket)
{
	if (dataPacket->mslMach > 0.6) {
		
		double DNA = dataPacket->CNA * (dataPacket->mslDynamicPressure * dataPacket->mslRefArea / dataPacket->mslMass); // METERS PER SECOND^2
		double DMA = dataPacket->CMA * (dataPacket->mslDynamicPressure * dataPacket->mslRefArea * dataPacket->mslRefDiam / dataPacket->mslTMoi); // PER SECOND^2
		double DMD = dataPacket->CMD * (dataPacket->mslDynamicPressure * dataPacket->mslRefArea * dataPacket->mslRefDiam / dataPacket->mslTMoi); // PER SECOND^2
		double DMQ = dataPacket->CMQ * (dataPacket->mslRefDiam / (2 * dataPacket->mslSpeed)) * (dataPacket->mslDynamicPressure * dataPacket->mslRefArea * dataPacket->mslRefDiam / dataPacket->mslTMoi); // PER SECOND
		double DLP = dataPacket->CLP * (dataPacket->mslRefDiam / (2 * dataPacket->mslSpeed)) * (dataPacket->mslDynamicPressure * dataPacket->mslRefArea * dataPacket->mslRefDiam / dataPacket->mslAMoi); // PER SECOND
		double DLD = dataPacket->CLD * (dataPacket->mslDynamicPressure * dataPacket->mslRefArea * dataPacket->mslRefDiam / dataPacket->mslAMoi); // PER SECOND^2

		double WACL = 0.013 * sqrt(dataPacket->mslDynamicPressure) + 7.1;
		double ZACL = 0.000559 * sqrt(dataPacket->mslDynamicPressure) + 0.232;
		double PACL = 14;

		// FEEDBACK GAINS
		double GAINFB3 = WACL * WACL * PACL / (DNA * DMD);
		double GAINFB2 = (2 * ZACL * WACL + PACL + DMQ - DNA / dataPacket->mslSpeed) / DMD;
		double GAINFB1 = (
			WACL * WACL +
			2 * ZACL * WACL * PACL +
			DMA +
			DMQ * DNA / dataPacket->mslSpeed -
			GAINFB2 * DMD * DNA / dataPacket->mslSpeed
		) / (DNA * DMD);

		// ROLL
		double GKP = (2 * wrcl * zrcl + DLP) / DLD;
		double GKPHI = wrcl * wrcl / DLD;
		double EPHI = GKPHI * (rollAngleComm - dataPacket->mslEuler[0]);
		rollFinComm = EPHI - GKP * dataPacket->mslEulerDot[0];

		// PITCH
		double zzdNew = dataPacket->normCommand - dataPacket->mslBodyAcc[2];
		double zzNew = integrate(zzdNew, zzd, zz, dataPacket->mslIntegrationStep);
		zz = zzNew;
		zzd = zzdNew;
		double deflPitch = -1 * GAINFB1 * dataPacket->mslBodyAcc[2] - GAINFB2 * dataPacket->mslEulerDot[1] + GAINFB3 * zz;
		if (abs(deflPitch) > maxDefl) {
			if (deflPitch > 0) {
				deflPitch = maxDefl;
			}
			else if (deflPitch < 0) {
				deflPitch = -1 * maxDefl;
			}
		}
		pitchFinComm = deflPitch * degToRad;

		// YAW
		double yydNew = dataPacket->mslBodyAcc[1] - dataPacket->sideCommand;
		double yyNew = integrate(yydNew, yyd, yy, dataPacket->mslIntegrationStep);
		yy = yyNew;
		yyd = yydNew;
		double deflYaw = GAINFB1 * dataPacket->mslBodyAcc[1] - GAINFB2 * dataPacket->mslEulerDot[2] + GAINFB3 * yy;
		if (abs(deflYaw) > maxDefl) {
			if (deflYaw > 0) {
				deflYaw = maxDefl;
			}
			else if (deflYaw < 0) {
				deflYaw = -1 * maxDefl;
			}
		}
		yawFinComm = deflYaw * degToRad;

	}
	else if (dataPacket->mslMach > 0.01) {

		double DNA = dataPacket->CNA * (dataPacket->mslDynamicPressure * dataPacket->mslRefArea / dataPacket->mslMass); // METERS PER SECOND^2
		double DND = dataPacket->CND * (dataPacket->mslDynamicPressure * dataPacket->mslRefArea / dataPacket->mslMass); // METERS PER SECOND^2
		double DMA = dataPacket->CMA * (dataPacket->mslDynamicPressure * dataPacket->mslRefArea * dataPacket->mslRefDiam / dataPacket->mslTMoi); // PER SECOND^2
		double DMD = dataPacket->CMD * (dataPacket->mslDynamicPressure * dataPacket->mslRefArea * dataPacket->mslRefDiam / dataPacket->mslTMoi); // PER SECOND^2
		double DMQ = dataPacket->CMQ * (dataPacket->mslRefDiam / (2 * dataPacket->mslSpeed)) * (dataPacket->mslDynamicPressure * dataPacket->mslRefArea * dataPacket->mslRefDiam / dataPacket->mslTMoi); // PER SECOND
		double DLP = dataPacket->CLP * (dataPacket->mslRefDiam / (2 * dataPacket->mslSpeed)) * (dataPacket->mslDynamicPressure * dataPacket->mslRefArea * dataPacket->mslRefDiam / dataPacket->mslAMoi); // PER SECOND
		double DLD = dataPacket->CLD * (dataPacket->mslDynamicPressure * dataPacket->mslRefArea * dataPacket->mslRefDiam / dataPacket->mslAMoi); // PER SECOND^2

		// ROLL
		double GKP = (2 * wrcl * zrcl + DLP) / DLD;
		double GKPHI = wrcl * wrcl / DLD;
		double EPHI = GKPHI * (rollAngleComm - dataPacket->mslEuler[0]);
		rollFinComm = EPHI - GKP * dataPacket->mslEulerDot[0];

		// RATE CONTROL
		double ZRATE = DNA / dataPacket->mslSpeed - DMA * DND / (dataPacket->mslSpeed * DMD); // ND
		double AA = DNA / dataPacket->mslSpeed - DMQ; // ND
		double BB = -1 * DMA - DMQ * DNA / dataPacket->mslSpeed; // ND
		double TEMP1 = AA - 2 * zetlagr * zetlagr * ZRATE; // ND
		double TEMP2 = AA * AA - 4 * zetlagr * zetlagr * BB; // ND
		double RADIX = TEMP1 * TEMP1 - TEMP2; // ND
		double GRATE = (-1 * TEMP1 + sqrt(RADIX)) / (-1 * DMD); // ND

		// PITCH
		pitchFinComm = GRATE * dataPacket->mslEulerDot[1]; // RADIANS

		// YAW
		yawFinComm = GRATE * dataPacket->mslEulerDot[2]; // RADIANS


	}
	else {
		rollFinComm = 0.0;
		pitchFinComm = 0.0;
		yawFinComm = 0.0;
	}

	dataPacket->pitchFinComm = pitchFinComm;
	dataPacket->yawFinComm = yawFinComm;
	dataPacket->rollFinComm = rollFinComm;

}