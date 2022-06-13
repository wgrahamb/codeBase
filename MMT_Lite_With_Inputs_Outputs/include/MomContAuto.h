//-----------------------------------------------------------//
// File: MomContAuto.h
// 
// This class determines the moment command for use in the 
// actuator.
//                                                           
// Developer: Dennis Strickland
//      Date: 30 September 2019
//-----------------------------------------------------------//

#ifndef MOMCONTAUTO_H
#define MOMCONTAUTO_H
#include "main.h"
#include "NavProc.h"

class Output;
class System;

#ifdef SIXDOF 
class MomContAuto : public Block {
	public:
	ofstream *fout0, *fout1;
#else
class MomContAuto : public Block {
#endif
	public:
	int t1;
	int kt, cnt;
	double sdtHz, sdt;
	float tick, t0;
	float amach, q_est, sref, cna, cnd, cmd0, ajy, amass;
	float xk, xki, xka, xkg;
	float acomy, acomz;
	float acomy0, acomz0;
	float mcp, mcy;
	float wn;
	float wx, wy, wz;
	float alph_max, alphp, alphy;
	float amax;
	float cmq, xmq;
	float a0, a1, a2;
	float delp, dely;
	float mcp_icar, mcy_icar, xkm;
	float mcp0, mcy0;
	float aaLim;
	float maxG;
	float gamdot_Lim, gamdot_Scale;
	float gamdot_Mag;
	float gamd_q, gamd_r;
	float gamd_q_nr, gamd_r_nr;
	float del_max;


	float phi_hat_nr;
	float gamd_qc_nr, gamd_rc_nr;

	float n_phi_hat_nr;
	float wx_nr, wy_nr, wz_nr;
	float axb_nr, ayb_nr, azb_nr;
	float gxb_nr, gyb_nr, gzb_nr;
	float vxb_nr, vyb_nr, vzb_nr;
	float alpha_hat_nr;
	float beta_hat_nr;
	float alphaTot_hat_nr;
	float qdot_est_nr, rdot_est_nr;
	float wyold_nr, wzold_nr;

	Vecff w_nr, gb_nr, abg_nr;

	int auto_Flag;

	float A_ap, wn_ap, zeta_ap;
	float c0, c1, c2, tn;
	float kn, kq, ka;

	float gdot_eq, gdot_er;
	float gdy_err, gdz_err, gdy_err0, gdz_err0;
	float y1y, y1z, x2y, x2z;
	float delta_qc, delta_rc;
	float dqdes, drdes;
	float theta_ddot, psi_ddot, theta_dot, psi_dot;
	float theta_ddotp, psi_ddotp;
	float dis_thtd, dis_psid;
	float delp_dis, dely_dis;
	float tdcp_gain, tdcy_gain;

	float cna_est, cnd_est, cma_est, cmd_est, xms_cg, xms_cgx;
	float cma2_est, aoaTot_est;
	float thr_est, xmom_arm;
	float xcg, ximu, dia, qsd, qs;
	float vm, vxb, vyb, vzb, axb, ayb, azb;
	float gxb, gyb, gzb;
	float alpha_hat, beta_hat, alphaTot_hat;
	float bhe, sbhe, cbhe, salp, calp;

	float qdot_est, rdot_est, wyold, wzold, qdotx, rdotx, xndomd;

	float malpha_est, mdelta_est, nalpha_est, ndelta_est;
	float nn_est;

	Table1ff *cna_a1_table, *cna_a2_table, *cma_b1_table, *cma_b2_table;
	float c_a1, c_a2, c_b1, c_b2, dx;
	float xmrp;

	float gamd_ramp;
	Table1ff *gamdramp_vs_t;

	Table1ff *xA_table, *xwn_table, *xzeta_table;
	Table1ff *cna_table, *cnd_table, *cmd0_table, *ajy_table;

	double yawFinCommandDegrees, pitchFinCommandDegrees;

	#ifdef SIXDOF 
	MomContAuto( string infile, Output *outp, System *sysp);
	#else
	MomContAuto();
	#endif
	void init();

	// Inputs.
	bool navSolutionAvailable; // hard code
	double pitchGuidanceCommand; // seperate from state
	double yawGuidanceCommand; // seperate from state
	double gammaDotLimit; // seperate from state
	double machSpeed; // seperate from state
	double dynamicPressure; // seperate from state
	double mass; // seperate from state
	double centerOfGravity; // seperate from state
	double missileSpeed;
	Vecff nonRolledBodyRate;
	Vecff missileBodyGravityEstimate;
	Vecff missileBodyAcceleration;
	double alpha;
	double beta;
	double angleOfAttack;

	void handleInput(NavigationState const &navigationState);
	void update(
		double normGuidanceCommand,
		double sideGuidanceCommand,
		double gamDotLimit,
		bool processExecuting,
		double rollAngle,
		double mach,
		double dynamicPressure,
		double referenceArea,
		double CNA_in,
		double CND_in,
		double CMD0_in,
		double AJY_in,
		double mass,
		double centerOfGrav,
		double XIMU_in,
		double referenceDiameter,
		double speed,
		Vecff nonRolledBodyRate,
		Vecff bodyGravEst,
		Vecff bodyAccelEst,
		double alpha,
		double beta,
		double aoa
	);

	#ifdef SIXDOF 
	void rpt();
	#endif

	private:
		Output     *out;
		System     *sys;

};

#endif
