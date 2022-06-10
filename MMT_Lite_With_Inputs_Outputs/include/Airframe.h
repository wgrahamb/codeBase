//-----------------------------------------------------------//
// File: Airframe.h
// 
// This class provides information about the airframe properties.
//                                                           
// Developer: Dennis Strickland                              
//-----------------------------------------------------------//

#ifndef AIRFRAME_H
#define AIRFRAME_H
#include "main.h"

class Output;
class System;

class Airframe : public Block
{
 
	public:
	int kt;
	double t1000, xd, tick, t;

	Airframe(string infile, Output *outp, System *sysp);

	double dtHz, sdt;
	double xfin;
	double ximu;

	int    spin_Flag;
	int    errTipoff_Flag;
	double pScale;
	double sref, dia, anoz, isp;
	double xrail;
	double fin_sig, fin1, fin2, fin3, fin4;
	double wpo_mean, wqo_mean, wro_mean;
	double wpo_sig, wqo_sig, wro_sig;
	double altg_max;
 
	void init();
	void update();
	void rpt();

	private:
		Output   *out;
		System   *sys;

};

#endif
