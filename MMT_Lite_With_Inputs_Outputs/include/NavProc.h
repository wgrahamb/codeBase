//-----------------------------------------------------------//
// File: NavProc.h
// 
// This class processes the navigation input to provide
// navigated position/velocity/attitude output.
//                                                           
// Developer: Dennis Strickland
//      Date: 30 September 2019
//-----------------------------------------------------------//

#ifndef NAVPROC_H
#define NAVPROC_H
#include "main.h"

class Output;
class System;

struct Integrator
{
	int k;
	float sdt, y0, y, u0;
	Integrator();
	void init( float y, float sdt );
	float update( float u );
}; 

#ifdef SIXDOF 
	class NavProc : public Block {
	public:
	ofstream *fout0, *fout1;
	bool executing;
#else
	class NavProc : public Block {
#endif
	public:
		int kt,kp,cnt,cnt10Hz;
		double t1000, xd, sdtHz, sdt;

		#ifdef SIXDOF 
			NavProc(string infile, Output *outp, System *sysp);
		#else
			NavProc();
		#endif
		void init();

		// Inputs.
		bool navSolutionAvailable;
		double missileTimeOfFlight;
		Vec missileLTFPosition;
		Vec missileLTFVelocity;
		Vec missileLTFEulerAngles;
		Vec missileBodyAcceleration;
		Vec missileBodyRate;

		void handleInput(NavigationState const &navigationState);
		void update();
		#ifdef SIXDOF 
		void rpt();
		#endif

		int nav_valid;
		int first10_valid;
		int nav10_valid;
		int nav200_valid;
		int burnout_flg;
		float g0;
		float wx_nav, wy_nav, wz_nav;
		float phi0;
		float count_sf_v, count_sf_w, count_sf_w_x;
		float count_sf_sf, count_sf_omega_X, count_sf_omega;
		float delt, navt;
		float delt_error;

		float tol;
		float gamV_nav, gamH_nav;

		Integrator wx_int;
		double phiInt; 
		Vec acc_b_NR;

		bool nav_solution_available;
		float nav_sol_latency;

		double orig_Lat, orig_Lon, orig_Altg;
		double tgt_Rng, tgt_Az, tgt_Hgt;

		Vecff pt, vt;
		Vecff vw, rr, a_w;
		Vecff vw0, vwd;
		Vecff acc_bf0, acc_bf1, omegaf0, omegaf1;
		Vecff acc_eci;
		Vecff acc_b, acc_b1, omega;
		Vecff vb, vm, pm;
		Vecff euler, euler_deg;
		Vecff delta_v, delta_theta;
		Vecff sf_imu, omega_imu;

		float t0, prev_tnav, burnout_time;
		Vecff pm_nav, vm_nav, vb_nav, vm_nav_p;
		Vecff accb_nav;
		Vecff gravn_nav, gravb_nav;
		float phi_nav, theta_nav, psi_nav;
		float phi_nav_deg, theta_nav_deg, psi_nav_deg;
		Matff mDCM_nav;
		Quatff quat_nav;
		Quatff quat_d;

		Matff mDCM_IMU;

		float phi_hat;
		float wx_nav_p;

		Vecff acc_grav_sum, acc_grav_sum_p;
		Vecff accm_nav;
		float qsd_p, qxd_p, qyd_p, qzd_p;
		float t_eps;

		float phi_tmp, theta_tmp, psi_tmp;
		float erx_p, ery_p, erz_p;

	private:
		int count_600Hz;
		Output     *out;
		System     *sys;

};

#endif
