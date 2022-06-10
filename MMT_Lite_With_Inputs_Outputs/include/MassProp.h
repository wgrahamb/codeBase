//------------------------------------------------------//
// File: MassProp.h
//
// This class determines the mass properties of the 
// missile during flight.
//                                                      
// Developer: Dennis Strickland                          
//------------------------------------------------------//

#ifndef MASSPROP_H
#define MASSPROP_H
#include "main.h"

class Output;
class System;

class MassProp : public Block {

	public:
		int kt;
		double t1000, xd, tick, t;

		MassProp( string infile, Output *outp, System *sysp);

		double pct;
		double m0,mf;
		double xcg0, xcg1;
		double ycg0, ycg1;
		double zcg0, zcg1;
		double ajx0, ajx1;
		double ajy0, ajy1;
		double ajz0, ajz1;
		double ajxy0, ajxy1;
		double ajxz0, ajxz1;
		double ajyz0, ajyz1;
		//
		double mass;
		double xcg, ycg, zcg;
		double ajx, ajy, ajz;
		double ajxy, ajxz, ajyz;
		//
		double m_sig;
		double xcg_sig, ycg_sig, zcg_sig;
		double ajx_sig, ajy_sig, ajz_sig;
		double ajxy_sig, ajxz_sig, ajyz_sig;
		//
		int    errMassProp_Flag;
		double mass_unc;
		double xcg_unc, ycg_unc, zcg_unc;
		double ajx_unc, ajy_unc, ajz_unc;
		double ajxy_unc, ajxz_unc, ajyz_unc;
		//
		Mat iten;

		void init();
		void update(double currentMassEstimate);
		void rpt();
		double unituni();
		double gaussianDistribution(double mean, double sig);

	protected:


	private:
		Output   *out;
		System   *sys;
	
};

#endif
