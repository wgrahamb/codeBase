//------------------------------------------------------//
// File: Motion.h
//
// This class implements the equations of motion for   
// an unsymmetric missile and solves for the missile's     
// translational/rotational dynamics.                    
//                                                      
// Developer: Dennis Strickland                          
///------------------------------------------------------//

#ifndef MOTION_H
#define MOTION_H
#include "main.h"

class Output;
class System;
class reFactoredGeometry;

class Motion : public Block
{

	public:

		int kt;
		double t1000, xd, tick, t;
		double t_detent_release;

		int errWind_Flag, gcnt;
		double wndSpd_sig, wndDir_sig;
		double gustSpd_sig, gustDir_sig;
		double wpsiDir, gpsiDir;
		double vw_mag, vg_mag;
		Vec vwind;
		Vec vwindb;

		double rng, tol;
		Vec euler, euler_deg;
		Quat quat, quat_d;
		double phi, tht, psi;
		double phi_deg, theta_deg, psi_deg;
		Vec eulerNbtoB, eulerNbtoB_deg;
		Vec eulerLTF, eulerLTF_deg;

		double Nx, Ny, Nz, Fdetent, breakwireDelay;
		double omegaB_d_Mag;
		Vec omegaI, omegaB, omegaB_d, acc_eci;
		Vec angMom;
		Vec force, moment;

		int tip_Flag, rail_Flag;
		int pos_pitch_Flag, neg_pitch_Flag, pitch_tumble_Flag;
		int pos_yaw_Flag, neg_yaw_Flag, yaw_tumble_Flag;
		int pitch_yaw_tumble_Flag;
		int DOF3_pass_flag;
		double deg_tol;
		double wpo_mean, wqo_mean, wro_mean;
		double wpo_err, wqo_err, wro_err;
		Vec tipoff;

		Vec epos, epos_i, evel, ipos, ivel, nvel, npos, npos_i, npos_0;
		Vec ltf_pos, ltf_vel;
		Vec XYZ_pos;

		Vec egrav, igrav, bgrav;

		Vec sf_b;
		Vec sumF_ECI;
		Vec accl_b;
		Vec ipos_d;
		Vec ivel_d;
		Vec evel_d;
		Vec uvw;
		Vec vb;
		Vec delPos;
		Vec g_load;
		double g_load_yz_mag, g_load_yz_max;
		double vrel;
		double vmag;
		double vb_yzmag;
		double qnr;
	
		double roll, pitch, yaw;
		double NEDbtoBODY_yaw, NEDbtoBODY_pitch, NEDbtoBODY_roll;
		double NEDbtoBODY_yaw_rad, NEDbtoBODY_pitch_rad, NEDbtoBODY_roll_rad;
		double LTFtoBODY_yaw, LTFtoBODY_pitch, LTFtoBODY_roll;
		double LTFtoBODY_yaw_r, LTFtoBODY_pitch_r, LTFtoBODY_roll_r;
		double gamV, gamH;
		double conAng, precRate, cangTol;
		double roll_d, pitch_d, yaw_d, precRate2;
		double alphp, alphy, alph, aphi;
		double alphp_r, alphy_r;

		Motion(string infile, Output *outp, System *sysp);
		double vecAng(Vec x, Vec y);
		void angle_of_attack(Vec v_b, double &alphp, double &alphy, double &alpha, double &aphi);
		double atan2_0(double y, double x);
		void SumForcesMoments();
		void Calc_6DOF();
		void UpdateTruth(Vec LTFWindVel);
		void checkTumble();
		void initECI(Vec epos, Vec evel, Mat i2e, Vec &ipos, Vec &ivel);
		void update_ECEF(Vec ipos, Vec ivel, Mat i2e, Vec &epos, Vec &evel);
		double inducedMoment();
		void init();
		void update(
			Vec LTFWindVel,
			Vec aeroForce,
			Vec aeroMoment,
			Vec motorForce,
			Vec motorMoment,
			int spinFlag,
			double railLength,
			double pScale,
			double mass,
			Mat inertiaTensor
		);
		Vec aeroForceIn, aeroMomentIn, motorForceIn, motorMomentIn;
		int spinFlagIn;
		double railLengthIn, pScaleIn;
		double massIn;
		Mat inertiaTensorIn;
		void rpt();

	private:
		Output   *out;
		System   *sys;
		reFactoredGeometry *re_factored_geo;

};

#endif
