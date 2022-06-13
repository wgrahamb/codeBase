//------------------------------------------------------//
// File: MassProp.cpp
//
// This class determines the mass properties of the 
// missile during flight.
//                                                      
// Developer: Dennis Strickland                          
//------------------------------------------------------//

#include "MassProp.h"
#include "System.h"
#include "Output.h"


MassProp::MassProp( string infile, Output *outp, System *sysp) {

	cout << "MASS PROPERTIES CONSTRUCTED" << endl;

	//Local access
	out   = outp;
	sys   = sysp;

	//Inputs
	Filer *ff = new Filer( "./input/"+infile);
	m0 = ff->getDouble( "m0");
	mf = ff->getDouble( "mf");
	//
	xcg0 = ff->getDouble( "xcg0");
	xcg1 = ff->getDouble( "xcg1");
	ycg0 = ff->getDouble( "ycg0");
	ycg1 = ff->getDouble( "ycg1");
	zcg0 = ff->getDouble( "zcg0");
	zcg1 = ff->getDouble( "zcg1");
	//
	ajx0 = ff->getDouble( "ajx0");
	ajx1 = ff->getDouble( "ajx1");
	ajy0 = ff->getDouble( "ajy0");
	ajy1 = ff->getDouble( "ajy1");
	ajz0 = ff->getDouble( "ajz0");
	ajz1 = ff->getDouble( "ajz1");
	//
	ajxy0 = ff->getDouble( "ajxy0");
	ajxy1 = ff->getDouble( "ajxy1");
	ajxz0 = ff->getDouble( "ajxz0");
	ajxz1 = ff->getDouble( "ajxz1");
	ajyz0 = ff->getDouble( "ajyz0");
	ajyz1 = ff->getDouble( "ajyz1");
	//
	m_sig   = ff->getDouble( "mass_sig");
	xcg_sig = ff->getDouble( "xcg_sig");
	ycg_sig = ff->getDouble( "ycg_sig");
	zcg_sig = ff->getDouble( "zcg_sig");
	ajx_sig = ff->getDouble( "ajx_sig");
	ajy_sig = ff->getDouble( "ajy_sig");
	ajz_sig = ff->getDouble( "ajz_sig");
	//
	ajxy_sig = ff->getDouble( "ajxy_sig");
	ajxz_sig = ff->getDouble( "ajxz_sig");
	ajyz_sig = ff->getDouble( "ajyz_sig");
	//
	errMassProp_Flag = ff->getInt( "errMassProp_Flag");
	delete ff;

}

void MassProp::init()
{
	tick = 0.0;
	kt = 0;

	// //Errors
	// //Determine Errors (if any)
	// bool disable = true;
	// if(errMassProp_Flag == 1)
	// {
	// 	disable = false;
	// }

	// mass_unc = gaussianDistribution(0.0, m_sig);
	// //
	// xcg_unc = gaussianDistribution(0.0, xcg_sig);
	// ycg_unc = gaussianDistribution(0.0, ycg_sig);
	// zcg_unc = gaussianDistribution(0.0, zcg_sig);
	// //
	// ajx_unc = gaussianDistribution(0.0, ajx_sig);
	// ajy_unc = gaussianDistribution(0.0, ajy_sig);
	// ajz_unc = gaussianDistribution(0.0, ajz_sig);
	// //
	// ajxy_unc = gaussianDistribution(0.0, ajxy_sig);
	// ajxz_unc = gaussianDistribution(0.0, ajxz_sig);
	// ajyz_unc = gaussianDistribution(0.0, ajyz_sig);

	//Determine Mass Properties
	mass = m0;
	xcg = xcg0;
	ycg = ycg0;
	zcg = zcg0;
	ajx  = ajx0;
	ajy  = ajy0;
	ajz  = ajz0;
	ajxy = ajxy0;
	ajxz = ajxz0;
	ajyz = ajyz0;

	//Load Inertia Tensor
	iten[0][0] = ajx0;
	iten[0][1] = ajxy0;
	iten[0][2] = ajxz0;
	iten[1][0] = ajxy0;
	iten[1][1] = ajy0;
	iten[1][2] = ajyz0;
	iten[2][0] = ajxz0;
	iten[2][1] = ajyz0;
	iten[2][2] = ajz0;
}

void MassProp::handleInput(NavigationState const &navigationState)
{
	missileTimeOfFlight = navigationState.missileTimeOfFlight_;
}

void MassProp::update()
{

	if( State::sample())
	{
		tick = ( double)kt++;
	}
	//
	if( State::sample( State::EVENT, 0.0)) {}

	//Determine Mass Properties
	
	if (missileTimeOfFlight < rocketBurnOut)
	{
		mass = linearInterpolation(missileTimeOfFlight, 0.0, rocketBurnOut, mf, m0);
		xcg = linearInterpolation(missileTimeOfFlight, 0.0, rocketBurnOut, xcg0, xcg1);
		ycg = linearInterpolation(missileTimeOfFlight, 0.0, rocketBurnOut, ycg0, ycg1);
		zcg = linearInterpolation(missileTimeOfFlight, 0.0, rocketBurnOut, zcg0, zcg1);
		ajx  = linearInterpolation(missileTimeOfFlight, 0.0, rocketBurnOut, ajx0, ajx1);
		ajy  = linearInterpolation(missileTimeOfFlight, 0.0, rocketBurnOut, ajy0, ajy1);
		ajz  = linearInterpolation(missileTimeOfFlight, 0.0, rocketBurnOut, ajz0, ajz1);
		ajxy  = linearInterpolation(missileTimeOfFlight, 0.0, rocketBurnOut, ajxy0, ajxy1);
		ajxz  = linearInterpolation(missileTimeOfFlight, 0.0, rocketBurnOut, ajxz0, ajxz1);
		ajyz  = linearInterpolation(missileTimeOfFlight, 0.0, rocketBurnOut, ajyz0, ajyz1);
	}
	else
	{
		mass = mf;
		xcg = xcg1;
		ycg = ycg1;
		zcg = zcg1;
		ajx  = ajx1;
		ajy  = ajy1;
		ajz  = ajz1;
		ajxy  = ajxy1;
		ajxz  = ajxz1;
		ajyz  = ajyz1;
	}


	//Load Inertia Tensor
	iten[0][0] = ajx;
	iten[0][1] = ajxy;
	iten[0][2] = ajxz;
	iten[1][0] = ajxy;
	iten[1][1] = ajy;
	iten[1][2] = ajyz;
	iten[2][0] = ajxz;
	iten[2][1] = ajyz;
	iten[2][2] = ajz;

}

void MassProp::rpt() {

	if( State::sample( out->pt_console)) {

		if(out->displayInput == 1) {
			printf( "MassProp  %10.4f\n", sys->t_sys);
		}

	}
}

double MassProp::unituni()
{
	double value;
	value=(double)rand()/RAND_MAX;
	return value;
}

double MassProp::gaussianDistribution(double mean,double sig)
{
	static int iset=0;
	static double gset;
	double fac,rsq,v1,v2,value;

	if(iset==0){
		do{
			v1=2.*unituni()-1.;
			v2=2.*unituni()-1.;
			rsq=v1*v1+v2*v2;
		}while(rsq>=1.0||rsq==0);

		fac=sqrt(-2.*log(rsq)/rsq);
		gset=v1*fac;
		iset=1;
		value=v2*fac;
	}
	else{
		iset=0;
		value=gset;
	}
	return value*sig+mean;
}

double MassProp::linearInterpolation(double x, double x1, double x2, double y1, double y2)
{
	return y1 + ((x - x1) * (y2 - y1) / (x2 - x1));
}