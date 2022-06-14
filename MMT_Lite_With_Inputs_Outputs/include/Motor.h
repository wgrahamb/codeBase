//------------------------------------------------------//
// File: Motor.h
//
// This class computes the forces and moments due to thrust
// during flight.
//                                                      
// Developer: Dennis Strickland                          
//------------------------------------------------------//

#ifndef MOTOR_H
#define MOTOR_H
#include "main.h"

class Output;
class System;

class Motor : public Block {

	public:
		int kt;
		double tick;

		Motor( string infile, Output *outp, System *sysp);

		double m0, mf;
		double sdt;
		double te;
		double masspd, massp;

		double thrust_vac, thrust;
		double tma_tht_sig, tma_psi_sig;
		double tma_tht, tma_psi;
		double tux, tuy, tuz;

		double time_param, thr_param;
		double time_fac, thr_fac;
		double thrust_TempScale;

		double rthxb, rthyb, rthzb;
		double rthxb_sig, rthyb_sig, rthzb_sig;
		double rthxb_Err, rthyb_Err, rthzb_Err;

		int    errMotor_Flag, burnout_flg;
		double tma_tht_Err, tma_psi_Err;
		double jitterY, jitterY_d;
		double jitterZ, jitterZ_d;
		double forceY, forceZ, forceYZ, forceAng;

		double tm, tmotor;
	
		Vec force, moment;

		Table1 *thrust_vs_t;

		double itot_Amb;
		double isp_Amb;

		bool after_launch;

		void init();

		// Inputs.

		double XCenterOfGravity;
		double YCenterOfGravity;
		double ZCenterOfGravity;
		double airTemperature;
		double airTemperatureNominal;
		double currentAirPressure;

		void handleInput(
			NavigationState const &navigationState,
			double xcg,
			double ycg,
			double zcg,
			double airTemp,
			double airTempNominal,
			double airPressure
		);
		void update();
		void rpt();

	private:
		Output   *out;
		System   *sys;
	
};

#endif
