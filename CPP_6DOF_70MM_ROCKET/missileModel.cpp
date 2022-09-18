
// Standard.
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <math.h>
#include <vector>
#include <map>
#include <algorithm>
#include <string>
#include <dirent.h>

// Tables.
#include "missileData.h"

// Utility.
#include "util.h"
#include "utility_header.hpp"

// Namespace.
using namespace std;

// End checks.
enum class Lethality
{
	loitering,
	flying,
	ground,
	nan,
	maxTime,
	intercept
};

// Some constants.
const double SEA_LEVEL_PRESSURE = 2116.8; // psf.
const auto WALL_CLOCK_START = chrono::high_resolution_clock::now();
const double PA_TO_PSF = 0.020885;
const double M_TO_FT = 3.28084;
const double KGPM3_TO_LBMPFT3 = 0.062428;
const double STD_GRAVITY = 32.2; // Feet per sec^2

struct Rocket
{

	private:

	// Overhead.
	double simTime;

	public:

	// Methods.
	void updateSimTime(double DT) {simTime += DT;}
	double getSimTime() {return simTime;}

	// Set a timestep limit.
	const double DT_LIM = 1.0 / 500.0;

	// Body.
	Lethality lethality = Lethality::loitering;
	double tof; // Seconds.
	double spd; // Feet per sec.
	double velB[3]; // Feet per sec.
	double alpha; // Rads.
	double beta; // Rads.
	double specificForce[3]; // Feet per sec^2.
	double rate[3]; // Rads per sec.

	// Frame.
	double fluToEnuMatrix[3][3]; // Non dimensional.
	double enuPos[3]; // Feet.
	double enuVel[3]; // Feet per sec.
	double enuEuler[3]; // Rads.

	// Atmosphere.
	double rho; // lbm per ft^3.
	double p; // psf.
	double temperature; // Farenheit.
	double q; // psf.
	double a; // feet per sec.
	double g; // feet per sec^2.
	double mach; // Non dimensional.
	double FLUg[3]; // feet per sec^2.

	// Table look ups. Targets for monte carlo.
	// Includes mass, motor, and aerodynamic coefficients.
	double mass; // Lbm.
	double xcg; // From base.
	double amoi; // Lbf-in^2.
	double tmoi; // Lbf-in^2.
	double vacThrust; // Lbf.
	double torque; // Lbf-in. Due to motor.
	double CNA; // Per rad.
	double xcp; // Inches from nose.
	double CMQ; // Per rad.
	double CR; // Non dimensional. Roll moment coefficient.
	double CLD; // Per rad.
	double CLP; // Per rad.
	double CD; // Non dimension.

	// Propulsion.
	double thrust; // lbf.

	// Aerodynamics.
	double CX; // Non dimensional.
	double CY; // Non dimensional.
	double CZ; // Non dimensional.
	double CL; // Non dimensional.
	double CM; // Non dimensional.
	double CN; // Non dimensional.
	double force[3]; // lbf.
	double moment[3]; // lbf-in.

};

void writeHeader(ofstream &logFile)
{

	logFile <<
	"tof " <<
	"spd " <<
	"u " <<
	"v " <<
	"w " <<
	"alpha " <<
	"beta " <<
	"udot " <<
	"vdot " <<
	"wdot " <<
	"p " <<
	"q " <<
	"r " <<
	"enuPosX " <<
	"enuPosY " <<
	"enuPosZ " <<
	"enuVelX " <<
	"enuVelY " <<
	"enuVelZ " <<
	"enuPhi " <<
	"enuTht " <<
	"enuPsi" <<
	"\n";

}

void writeData(Rocket &rocket, ofstream &logFile)
{

	logFile << setprecision(10) << fixed <<
	rocket.tof << " " <<
	rocket.spd << " " <<
	rocket.velB[0] << " " <<
	rocket.velB[1] << " " <<
	rocket.velB[2] << " " <<
	rocket.alpha << " " <<
	rocket.beta << " " <<
	rocket.specificForce[0] << " " <<
	rocket.specificForce[1] << " " <<
	rocket.specificForce[2] << " " <<
	rocket.rate[0] << " " <<
	rocket.rate[1] << " " <<
	rocket.rate[2] << " " <<
	rocket.enuPos[0] << " " <<
	rocket.enuPos[1] << " " <<
	rocket.enuPos[2] << " " <<
	rocket.enuVel[0] << " " <<
	rocket.enuVel[1] << " " <<
	rocket.enuVel[2] << " " <<
	rocket.enuEuler[0] << " " <<
	rocket.enuEuler[1] << " " <<
	rocket.enuEuler[2] << " " <<
	"\n";

}

void emplace(Rocket &rocket, double phiDeg, double thtDeg, double psiDeg)
{

	// Input.
	double phi = phiDeg * degToRad;
	double tht = thtDeg * degToRad;
	double psi = psiDeg * degToRad;

	// Frame.
	eulerAnglesToLocalOrientation(phi, -1.0 * tht, psi, rocket.fluToEnuMatrix);
	setArrayEquivalentToZero(rocket.enuPos);
	setArrayEquivalentToZero(rocket.enuVel);
	setArray(rocket.enuEuler, phi, tht, psi);

	// Body.
	rocket.tof = 0.0;
	rocket.spd = 0.0;
	setArrayEquivalentToZero(rocket.velB);
	rocket.alpha = 0.0;
	rocket.beta = 0.0;
	setArrayEquivalentToZero(rocket.specificForce);

	// Tipoff. Zero for now.
	setArrayEquivalentToZero(rocket.rate);

};

void launch(Rocket &rocket) {rocket.lethality = Lethality::flying;}

void fly(Rocket &rocket, double DT)
{

	// Input.
	double tempTimeStep = DT;
	if (tempTimeStep > rocket.DT_LIM)
	{
		tempTimeStep = rocket.DT_LIM;
	}
	const double theTimeStep = tempTimeStep;

	// Set max time.
	const double MAX_TIME = rocket.tof + DT;

	// Data.
	ofstream logFile;
	logFile.open("CPP_6DOF_70MM_ROCKET/visual/log.txt");
	writeHeader(logFile);
	writeData(rocket, logFile);

	// Loop.
	while (rocket.lethality == Lethality::flying)
	{

		rocket.tof += theTimeStep;

		// Atmosphere.
		double tempRho; // kg per m^3.
		double tempPressure; // pascals.
		double tempTemperature; // Kelvin.
		double tempA; // meters per second.
		us76_nasa2002(rocket.enuPos[2] * (1.0 / M_TO_FT), &tempRho,
			&tempPressure, &tempTemperature, &tempA);

		rocket.rho = tempRho * KGPM3_TO_LBMPFT3; // lbm per ft^3.
		rocket.p = tempPressure * PA_TO_PSF; // psf.
		rocket.temperature = (tempTemperature - 273.15) * (9.0/5.0) + 32; // farenheit.
		rocket.q = (0.5 * rocket.rho * rocket.spd * rocket.spd) / STD_GRAVITY; // psf.
		rocket.a = tempA * M_TO_FT; // feet per second.
		rocket.g = STD_GRAVITY; // feet per second squared.
		rocket.mach = rocket.spd / rocket.a;
		double ENUg[3] = {0.0, 0.0, -1.0 * rocket.g};
		threeByThreeTimesThreeByOne(rocket.fluToEnuMatrix, ENUg, rocket.FLUg);

		// Look ups.
		rocket.mass = lin1DInterp(WEIGHT_V_TIME, rocket.tof); // lbm.
		rocket.xcg = lin1DInterp(XCG_V_TIME, rocket.tof); // inches from base.
		rocket.amoi = lin1DInterp(AMOI_V_TIME, rocket.tof); // lbf-in^2.
		rocket.tmoi = lin1DInterp(TMOI_V_TIME, rocket.tof); // lbf-in^2.
		rocket.vacThrust = lin1DInterp(THRUST_V_TIME, rocket.tof); // lbf.
		rocket.torque = lin1DInterp(TORQUE_V_TIME, rocket.tof); // lbf-in.
		rocket.CNA = lin1DInterp(CNA_V_MACH, rocket.mach); // per rad
		rocket.xcp = lin1DInterp(XCP_V_MACH, rocket.mach); // inches from nose.
		rocket.CMQ = lin1DInterp(CMQ_V_MACH, rocket.mach); // per rad.
		rocket.CR = lin1DInterp(
			CL_V_MACH, rocket.mach); // non dimensional. rolling moment coefficient.
		rocket.CLD = lin1DInterp(CLD_V_MACH, rocket.mach); // per rad.
		rocket.CLP = lin1DInterp(CLP_V_MACH, rocket.mach);
		if (rocket.tof <= ROCKET_BURNOUT)
		{
			rocket.CD = lin1DInterp(CD_ON_V_MACH, rocket.mach);
		}
		else
		{
			rocket.CD = lin1DInterp(CD_OFF_V_MACH, rocket.mach);
		}

		// Propulsion.
		if (rocket.tof <= ROCKET_BURNOUT)
		{
			rocket.thrust = rocket.vacThrust + (SEA_LEVEL_PRESSURE - rocket.p) * REF_AREA;
		}
		else
		{
			rocket.thrust = 0.0;
		}

		// Aerodynamics.
		double windCD[3] = {-1.0 * rocket.CD, 0.0, 0.0};
		double windToBody[3][3];
		flightPathAnglesToLocalOrientation(rocket.beta, rocket.alpha, windToBody);
		double bodyCD[3];
		threeByThreeTimesThreeByOne(windToBody, windCD, bodyCD);

		rocket.CX = bodyCD[0];
		rocket.CY = rocket.CNA * rocket.beta + bodyCD[1];
		rocket.CZ = rocket.CNA * rocket.alpha + bodyCD[2];

		double CLTOTAL = rocket.CR;
		double CLDEFL = rocket.CLD * (10.0 * degToRad);
		double CLDAMP = (REF_DIAM / 2 * rocket.spd) * (rocket.CLP * rocket.rate[0]);
		rocket.CL = CLTOTAL + CLDEFL + CLDAMP;

		double CMREF = 0.0;
		double CMFORCE = -1.0 * rocket.CZ *
			((REF_LENGTH - rocket.xcg) - rocket.xcp) / REF_DIAM; // moment due to force
		double CMDAMP = (REF_DIAM / 2 * rocket.spd) *
			(rocket.CMQ + 0.0) * rocket.rate[1]; // damping
		rocket.CM = CMREF + CMFORCE + CMDAMP;

		double CNREF = 0.0;
		double CNFORCE = -1.0 * rocket.CY *
			((REF_LENGTH - rocket.xcg) - rocket.xcp) / REF_DIAM; // moment due to force
		double CNDAMP = (REF_DIAM / 2 * rocket.spd) *
			(rocket.CMQ + 0.0) * rocket.rate[2]; // damping
		rocket.CN = CNREF + CNFORCE + CNDAMP;

		rocket.force[0] = rocket.thrust + rocket.CX * rocket.q * REF_AREA +
			(rocket.FLUg[0] * rocket.mass) / STD_GRAVITY;
		rocket.force[1] = rocket.CY * rocket.q * REF_AREA +
			(rocket.FLUg[1] * rocket.mass) / STD_GRAVITY;
		rocket.force[2] = rocket.CZ * rocket.q * REF_AREA +
			(rocket.FLUg[2] * rocket.mass) / STD_GRAVITY;

		rocket.moment[0] = rocket.torque + rocket.CL * rocket.q * REF_AREA * REF_DIAM;
		rocket.moment[1] = rocket.CM * rocket.q * REF_AREA * REF_DIAM;
		rocket.moment[2] = rocket.CN * rocket.q * REF_AREA * REF_DIAM;

		// Derivatives.
		double enuVel[3];
		double enuAcc[3];
		double enuEulerDot[3];
		double rateDot[3];

		// Vel.
		setArrayEquivalentToReference(enuVel, rocket.enuVel);

		// Acc.
		rocket.specificForce[0] = rocket.force[0] / rocket.mass;
		rocket.specificForce[1] = rocket.force[1] / rocket.mass;
		rocket.specificForce[2] = rocket.force[2] / rocket.mass;
		oneByThreeTimesThreeByThree(rocket.specificForce, rocket.fluToEnuMatrix, enuAcc);

		// Euler dot.
		enuEulerDot[0] = rocket.rate[0] + (rocket.rate[1] * sin(rocket.enuEuler[0]) + 
			rocket.rate[2] * cos(rocket.enuEuler[0])) * tan(rocket.enuEuler[1]);
		enuEulerDot[1] = rocket.rate[1] * cos(rocket.enuEuler[0]) - rocket.rate[2]
			* sin(rocket.enuEuler[0]);
		enuEulerDot[2] = (rocket.rate[1] * sin(rocket.enuEuler[0]) +
			rocket.rate[2] * cos(rocket.enuEuler[0]))
			/ cos(rocket.enuEuler[1]);

		// Rate dot.
		rateDot[0] = rocket.moment[0] / rocket.amoi;
		rateDot[1] = (1 / rocket.tmoi) * ((rocket.tmoi - rocket.amoi) *
			rocket.rate[0] * rocket.rate[2] + rocket.moment[1]);
		rateDot[2] = (1 / rocket.tmoi) * ((rocket.amoi - rocket.tmoi) *
			rocket.rate[0] * rocket.rate[1] + rocket.moment[2]);

		// State.
		double deltaPos[3];
		double deltaVel[3];
		double deltaEuler[3];
		double deltaRate[3];

		multiplyVectorTimesScalar(theTimeStep, enuVel, deltaPos);
		multiplyVectorTimesScalar(theTimeStep, enuAcc, deltaVel);
		multiplyVectorTimesScalar(theTimeStep, deltaEuler, enuEulerDot);
		multiplyVectorTimesScalar(theTimeStep, deltaRate, rateDot);

		double newEnuPos[3];
		double newEnuVel[3];
		double newEnuEuler[3];
		double newRate[3];

		addTwoVectors(rocket.enuPos, deltaPos, newEnuPos);
		addTwoVectors(rocket.enuVel, deltaVel, newEnuVel);
		addTwoVectors(rocket.enuEuler, deltaEuler, newEnuEuler);
		addTwoVectors(rocket.rate, deltaRate, newRate);

		setArrayEquivalentToReference(rocket.enuPos, newEnuPos);
		setArrayEquivalentToReference(rocket.enuVel, newEnuVel);
		setArrayEquivalentToReference(rocket.enuEuler, newEnuEuler);
		setArrayEquivalentToReference(rocket.rate, newRate);

		// Attitude.
		eulerAnglesToLocalOrientation(
			rocket.enuEuler[0],
			-1.0 * rocket.enuEuler[1],
			rocket.enuEuler[2],
			rocket.fluToEnuMatrix
		);
		threeByThreeTimesThreeByOne(rocket.fluToEnuMatrix, rocket.enuVel, rocket.velB);
		magnitude(rocket.velB, rocket.spd);
		rocket.alpha = -1.0 * atan2_0(rocket.velB[2], rocket.velB[0]);
		rocket.beta = -1.0 * atan2_0(rocket.velB[1], rocket.velB[0]);

		// Report.
		cout << rocket.tof << "\n";
		consolePrintArray("ENU", rocket.enuPos);
		cout << "\n";

		// Data.
		writeData(rocket, logFile);

		// Implicit end checks.
		if (rocket.enuPos[2] < 0.0)
		{
			rocket.lethality = Lethality::ground;
		}
		else if (isnan(rocket.enuPos[2]))
		{
			rocket.lethality = Lethality::nan;
		}
		else if (rocket.tof > MAX_TIME)
		{
			rocket.lethality = Lethality::maxTime;
		}

		// break;

	}

};

// Only updates if lethality is set to "flying."
void update(Rocket &rocket, double DT)
{

	if (rocket.lethality == Lethality::flying)
	{
		fly(rocket, DT);
		rocket.updateSimTime(DT);
	}
	else
	{
		rocket.updateSimTime(DT);
		return;
	}

}

int main()
{

	cout << "HOWDY!\n";

	Rocket rocket0;
	emplace(rocket0, 0.0, 45.0, 0.0);
	launch(rocket0);
	update(rocket0, double(100.0));

	return 0;

}