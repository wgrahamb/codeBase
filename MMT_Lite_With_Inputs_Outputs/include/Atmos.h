//-----------------------------------------------------------//
// File: Atmos.h
// 
// This class implements the 1962 Atmospheric model.       
// It provides:                                              
//    density, speed-of-sound, pressure, dynamic pressure    
//    and mach number vs altitude.                           
//                                                           
// Wind and Gust magnitude and direction are determined.     
// Pitch/Yaw Angle of Attack values are calculated as well.  
//                                                           
// Developer: Dennis Strickland                              
//-----------------------------------------------------------//

#ifndef ATMOS_H
#define ATMOS_H
#include "main.h"

class Output;
class System;
class TFAtm62;

class Atmos : public Block {

	public:
		int kt;
		double t1000, xd, tick, t;

		Atmos( string infile, Output *outp, System *sysp);

		double rho, vs, p, q, amach;

		double airTemp, airTemp_nom, airTemp_sig, airTemp_Err;
		double rho_sig, vs_sig, p_sig;
		double wndSpd_sig, wndDir_sig;
		double gustSpd_sig, gustDir_sig;
		//
		int    errWind_Flag;
		int    gcnt;

		double wpsiDir, gpsiDir;
		double vw_mag, vg_mag;
		Vec    vwind, vgust;

		int    errAtmos_Flag;
		double rho_Err, vs_Err, p_Err;

		TFAtm62 *atm62;

		void init();
		void update(double altitude, double speed);
		void rpt();

		double uniformDistribution(double min,double max);
		double unituni();
		double gaussianDistribution(double mean, double sig);

	private:
		Output   *out;
		System   *sys;

	protected:


	
};

#endif
