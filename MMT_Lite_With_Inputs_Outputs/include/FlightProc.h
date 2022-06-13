//-----------------------------------------------------------//
// File: FlightProc.h
// 
// This class determines estimates of atmospherics, aero and
// missile mass properties for use by the autopilot.
//                                                           
// Developer: Dennis Strickland
//      Date: 30 September 2019
//-----------------------------------------------------------//

#ifndef FLIGHTPROC_H
#define FLIGHTPROC_H
#include "main.h"

class Output;
class System;

#ifdef SIXDOF 
class FlightProc : public Block {

	public:
	ofstream *fout0, *fout1;
#else
class FlightProc : public Block {
#endif
	public:
	int kt;
	double t1000, xd, sdt;

	#ifdef SIXDOF 
	FlightProc(string infile, Output *outp, System *sysp);
	#else
	FlightProc();
	#endif
	void init();
	void handleInput(NavigationState const &navigationState);

	// Inputs.
	bool navSolutionAvailable;
	double currentAltitude;
	Vecff missileBodyVelocity;
	Vec missileBodyRate;
	Vecff missileBodyGravityEstimate;
	Vecff missileBodyAcceleration;
	double missileRollAngle;

	void update();

	#ifdef SIXDOF 
	void rpt();
	#endif

	float q_est;
	float sref_est, dia_est;
	float cna_est, cnd_est, cld_est, clp_est;
	float cma0_est, cmd0_est;
	float cmq_est;
	float ajx_est, ajy_est;
	float amass_est;
	float v_est;
	float amach;
	float alphp_est, alphy_est, alph_est;
	float xcp_est;
	float xcg_est, ycg_est, zcg_est;
	float ximu_est, yimu_est, zimu_est;
	float hm_est, rho, vs;

	Matff BodyRollToNR_DCM;
	Vecff w_est, gb_est, ab_est, vb_est;
	Vecff w_est_nr, gb_est_nr, ab_est_nr, abg_est_nr, vb_est_nr;

	float n_phi_hat_nr;
	float wx_est_nr, wy_est_nr, wz_est_nr;
	float axb_est_nr, ayb_est_nr, azb_est_nr;
	float gxb_est_nr, gyb_est_nr, gzb_est_nr;
	float vxb_est_nr, vyb_est_nr, vzb_est_nr;
	float alpha_est_nr, beta_est_nr, alphaTot_est_nr;

	float cnt,cnt1;

	Table1ff *rho_vs_hm, *vs_vs_hm;
	Table1ff *cna_table, *ajx_table, *ajy_table, *amass_table;
	Table1ff *xcp_table;
	Table1ff *xcg_table, *ycg_table, *zcg_table;
	Table1ff *cnd_table;
	Table1ff *cld_table, *clp_table;
	Table1ff *cmq_table;
	Table1ff *cma0_table, *cmd0_table;

	void angle_of_attack(Vecff v_b, float &alphp, float &alphy, float &alpha);

	private:
		Output   *out;
		System   *sys;

};

#endif
