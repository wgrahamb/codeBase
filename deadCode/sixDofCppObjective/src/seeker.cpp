#include "iostream"
#include "seeker.h"
#include "util.h"

seeker::seeker(mslDataPacket *dataPacket, double tgtE, double tgtN, double tgtU)
{
	std::cout << "SEEKER INITIATED" << std::endl;

	gk = 10; // KALMAN FILTER GAIN >>> PER SECOND
	zetak = 0.9; // KALMAN FILTER DAMPING
	wnk = 60; // KALMAN FILTER NATURAL FREQUENCY >>> RADIANS PER SECOND

	dataPacket->mslWayPoint[0] = tgtE;
	dataPacket->mslWayPoint[1] = tgtN;
	dataPacket->mslWayPoint[2] = tgtU;

	double localRelPos[3];
	subtractTwoVectors(dataPacket->mslPos, dataPacket->mslWayPoint, localRelPos);
	double localRelPosU[3];
	unitVec(localRelPos, localRelPosU);
	double mslToInterceptU[3];
	threeByThreeTimesThreeByOne(dataPacket->mslLocalOrient, localRelPos, forwardLeftUpMslToIntercept);
	threeByThreeTimesThreeByOne(dataPacket->mslLocalOrient, localRelPosU, mslToInterceptU);
	azAndElFromVector(seekerYaw, seekerPitch, mslToInterceptU);

	dataPacket->forwardLeftUpMslToIntercept[0] = forwardLeftUpMslToIntercept[0];
	dataPacket->forwardLeftUpMslToIntercept[1] = forwardLeftUpMslToIntercept[1];
	dataPacket->forwardLeftUpMslToIntercept[2] = forwardLeftUpMslToIntercept[2];

	double seekerAttitudeToLocalTM[3][3]; // ND
	eulerAnglesToLocalOrientation(0.0, -1 * seekerPitch, seekerYaw, seekerAttitudeToLocalTM); // ND
	threeByThreeTimesThreeByThree(seekerAttitudeToLocalTM, dataPacket->mslLocalOrient, seekerLocalOrient); // ND
	wlr = seekerYaw;
	wlq = seekerPitch;

}

void seeker::update(mslDataPacket *dataPacket)
{

	double wsq = wnk * wnk;
	double gg = gk * wsq;

	// YAW CHANNEL
	double wlr1d_new = wlr2;
	double wlr1_new = integrate(wlr1d_new, wlr1d, wlr1, dataPacket->mslIntegrationStep);
	wlr1 = wlr1_new;
	wlr1d = wlr1d_new;
	double wlr2d_new = gg * seekerYawErr - 2 * zetak * wnk * wlr1d - wsq * wlr1;
	double wlr2_new = integrate(wlr2d_new, wlr2d, wlr2, dataPacket->mslIntegrationStep);
	wlr2 = wlr2_new;
	wlr2d = wlr2d_new;

	// PITCH CHANNEL
	double wlq1d_new = wlq2;
	double wlq1_new = integrate(wlq1d_new, wlq1d, wlq1, dataPacket->mslIntegrationStep);
	wlq1 = wlq1_new;
	wlq1d = wlq1d_new;
	double wlq2d_new = gg * seekerPitchErr - 2 * zetak * wnk * wlq1d - wsq * wlq1;
	double wlq2_new = integrate(wlq2d_new, wlq2d, wlq2, dataPacket->mslIntegrationStep);
	wlq2 = wlq2_new;
	wlq2d = wlq2d_new;

	// YAW CONTROL
	double wlrd_new = wlr1 - dataPacket->mslEulerDot[2];
	double wlr_new = integrate(wlrd_new, wlrd, wlr, dataPacket->mslIntegrationStep);
	wlr = wlr_new;
	wlrd = wlrd_new;
	seekerYaw = wlr;

	// PITCH CONTROL
	double wlqd_new = wlq1 - dataPacket->mslEulerDot[1];
	double wlq_new = integrate(wlqd_new, wlqd, wlq, dataPacket->mslIntegrationStep);
	wlq = wlq_new;
	wlqd = wlqd_new;
	seekerPitch = wlq;

	double localRelPos[3];
	subtractTwoVectors(dataPacket->mslPos, dataPacket->mslWayPoint, localRelPos);
	double seekerAttitudeToLocalTM[3][3];
	eulerAnglesToLocalOrientation(0.0, -1 * seekerPitch, seekerYaw, seekerAttitudeToLocalTM);
	threeByThreeTimesThreeByThree(seekerAttitudeToLocalTM, dataPacket->mslLocalOrient, seekerLocalOrient);
	double seekerToInterceptRelPos[3];
	threeByThreeTimesThreeByOne(seekerLocalOrient, localRelPos, seekerToInterceptRelPos);
	double inducedErr[3] = {1.0, 0.5, 0.2};
	double seekerToInterceptRelPosWithErr[3];
	multiplyTwoVectors(seekerToInterceptRelPos, inducedErr, seekerToInterceptRelPosWithErr);
	azAndElFromVector(seekerYawErr, seekerPitchErr, seekerToInterceptRelPosWithErr);
	oneByThreeTimesThreeByThree(seekerToInterceptRelPosWithErr, seekerAttitudeToLocalTM, forwardLeftUpMslToIntercept);

	dataPacket->forwardLeftUpMslToIntercept[0] = forwardLeftUpMslToIntercept[0];
	dataPacket->forwardLeftUpMslToIntercept[1] = forwardLeftUpMslToIntercept[1];
	dataPacket->forwardLeftUpMslToIntercept[2] = forwardLeftUpMslToIntercept[2];

}