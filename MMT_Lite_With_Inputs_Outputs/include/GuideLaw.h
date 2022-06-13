//-----------------------------------------------------------//
// File: GuideLaw.h
// 
// This class determines pitch/yaw guidance commands.
//                                                           
// Developer: Dennis Strickland
//      Date: 30 September 2019
//-----------------------------------------------------------//

#ifndef GUIDELAW_H
#define GUIDELAW_H
#include "main.h"
#include "NavProc.h"

class Output;
class System;

#ifdef SIXDOF 
class GuideLaw : public Block {

	public:
	ofstream *fout0, *fout1;
#else
class GuideLaw : public Block {
#endif
	public:
	int kt;
	int cnt;
	float t1000, xd;
	float t_eps;

	#ifdef SIXDOF 
	GuideLaw(string infile, Output *outp, System *sysp);
	#else
	GuideLaw();
	#endif

	void init();

	// Inputs.
	bool navSolutionAvailable;
	Vecff wayPoint;
	Vecff missileLTFPosition;
	Vecff missileLTFVelocity;
	bool navSolution200HZAvailable;
	double rollAngle;
	Matff missileNavigationDCM;
	Matff missileRolledToNonRolledDCM;
	double alpha;
	double beta;

	void handleInput(NavigationState const &navigationState);
	void update();
	#ifdef SIXDOF 
	void rpt();
	#endif

	double sdtHz, sdt;
	float t_fin_lock_off;
	float phi_c;
	float tguide;
	float acomy, acomz;
	float acomy_t, acomz_t;
	float t_kf;

	int gamdot_Type;
	int guide_Flag, xkn_flg;

	Vecff mtpos, mtvel, mtpos_b, mtvel_b;
	Vecff pm_n, vm_n;
	Vecff prev_pm_n, prev_vm_n;
	Vecff rXv;
	float vm_mag;
	float gamP_T, gamY_T;
	float rngsq;
	float wx, wy, wz;
	float phi;

	float impAz, impTht;
	Vecff wp1;
	double alt;
	//
	float gamY_d, gamP_d;
	Vecff velTan_d;

	Vecff rngToGo, rngToGo_u;
	float rngToGo_mag;
	Vecff vm_b;
	Vecff vm_b_NR;
	float vm_b_mag;
	float alpha_NR, beta_NR;
	float bhe, sbhe, cbhe, salp, calp;
	Vecff velTan_i_u;
	Vecff sbeta_i_u, sbeta_b_u, sbeta_b_nr, sbeta_vel_nr;
	Vecff vXvd_i_u, vXvd_b_u, vXvd_b_nr, vXvd_vel_nr;
	float acomy1, acomz1;
	float gamd_q, gamd_r;
	float gamd_q1, gamd_r1;
	float gamd_q_t, gamd_r_t;
	float xks, xkn, xkni;
	float t_del, t1;
	float del_pm, del_pm_Lim;
	float gamdot_Lim;

	float tol;
	float gamV_n, gamH_n;

	Matff BodyRollToNR_DCM;
	Vecff rtg_nr, w_nr;

	Table1ff *xkn1_vs_t, *xkn2_vs_t, *phi_c_vs_t;
	Table1ff *gamd_q_vs_t, *gamd_r_vs_t;
	Table1ff *xks_vs_t;

	private:
		Output     *out;
		System     *sys;

};

#endif
