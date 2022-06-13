//-----------------------------------------------------------//
// File: RollAuto.h
// 
// This class provides roll commands and moments to the actuator
// as part of the roll autopilot functionality.
//                                                           
// Developer: Dennis Strickland
//      Date: 30 September 2019
//-----------------------------------------------------------//

#ifndef ROLLAUTO_H
#define ROLLAUTO_H
#include "main.h"
#include "NavProc.h"

class Output;
class System;

#ifdef SIXDOF 
class RollAuto : public Block {
	public:
	ofstream *fout0, *fout1;
#else
class RollAuto : public Block {
#endif
	public:
	int kt;
	double tick, sdtHz, sdt;

	#ifdef SIXDOF 
	RollAuto( string infile, Output *outp, System *sysp);
	#else
	RollAuto();
	#endif
	void init();

	// Inputs.

	bool navSolutionAvailable;
	double missileTimeOfFlight;
	Vecff missileNonRolledBodyRate;
	double machSpeed;
	double dynamicPressure;
	double rollAngle;

	void handleInput(NavigationState const &navigationState, double mach, double q);

	void update();
	#ifdef SIXDOF 
	void rpt();
	#endif
	
	int kp;

	int auto_Flag;
	float ajx, amach;
	float xki, xka, xkg;
	float phi_hat, phi_hat_deg, phi_u;
	float phi_c, phi0;
	float t0;
	float d, mom_c;
	float phi1, phi1x;
	float mom_c1, xkm, mom_icar;
	float wx_nr;
	float delr_cmd_lim;
	float phi_hi;
	float phi1_in, phi1_in_p, wx_in, wx_p;
	float cld, q, sref, dia, qsd;
	float ldelta_est;
	float aaLim;
	float kpr_gain, tdc_gain;

	float A_ap, wn_ap, zeta_ap;
	float c0, c1, c2, tn;
	float knr, kpr, kar;

	float rolldd, rolldot, disRoll, delRolldis;

	Vecff w_nr;

	Vecff vzb;

	Table1ff *xA_table, *xwn_table, *xzeta_table, *ajx_table, *cld_table;

	Integrator wx_integ;

	double rollFinCommandDegrees;

	private:
		Output     *out;
		System     *sys;

};

#endif
