//-----------------------------------------------------------//
// File: Atmos.cpp
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

#include "Atmos.h"
#include "System.h"
#include "TFAtm62.h"
#include "Output.h"
#include "Util.h"


Atmos::Atmos( string infile, Output *outp, System *sysp) {

	cout << "ATMOSHPERE CONSTRUCTED" << endl;

	//Local access
	out   = outp;
	sys   = sysp;
 
	atm62 = new TFAtm62();

	//Input
	Filer *ff = new Filer( "./input/"+infile);
	errAtmos_Flag = ff->getInt( "errAtmos_Flag");
	airTemp_nom   = ff->getDouble( "airTemp_nom");
	airTemp_sig   = ff->getDouble( "airTemp_sig");
	rho_sig       = ff->getDouble( "rho_sig");
	vs_sig        = ff->getDouble( "vs_sig");
	p_sig         = ff->getDouble( "p_sig");
	
	//
	errWind_Flag  = ff->getInt( "errWind_Flag");
	wndSpd_sig    = ff->getDouble( "wndSpd_sig");
	wndDir_sig    = ff->getDouble( "wndDir_sig");
	gustSpd_sig   = ff->getDouble( "gustSpd_sig");
	gustDir_sig   = ff->getDouble( "gustDir_sig");
	//
	delete ff;

}

void Atmos::init()
{ 

	tick = 0.0;
	kt = 0;
	gcnt = -1;  //gust counter
 
	//
	bool disable = true;
	if(errWind_Flag == 1)
	{
		disable = false;
	}

	vw_mag = gaussianDistribution(0.0, wndSpd_sig);
	vg_mag = gaussianDistribution(0.0, gustSpd_sig);
	//
	wpsiDir = uniformDistribution(-1 * wndDir_sig, wndDir_sig);
	gpsiDir = uniformDistribution(-1 * gustDir_sig, gustDir_sig);

	//Wind and Gust
	vwind.x = vw_mag*cos(wpsiDir*dtr) + vg_mag*cos(gpsiDir*dtr);
	vwind.y = vw_mag*sin(wpsiDir*dtr) + vg_mag*sin(gpsiDir*dtr);
	vwind.z = 0.0;

	//Atmospheric Erros
	//Determine Errors (if any)
	disable = true;
	if(errAtmos_Flag == 1)
	{
		disable = false;
	}
	rho_Err = gaussianDistribution(0.0, rho_sig);
	vs_Err = gaussianDistribution(0.0, vs_sig);
	p_Err = gaussianDistribution(0.0, p_sig);
	airTemp_Err = uniformDistribution(-1 * airTemp_sig, airTemp_sig);

	//Atmospheric
	airTemp = airTemp_nom + airTemp_Err;
	rho     = atm62->rho_() * (1.0+rho_Err);
	vs      = atm62->vs_()  * (1.0+vs_Err);
	p       = atm62->p_()   * (1.0+p_Err);
	q       = 0.5 * rho  * 1 * 1;
	amach   = 1 / vs;

}

void Atmos::update(double altitude, double speed)
{

	if( State::sample())
	{
		tick = ( double)kt++;
	}
	if( State::sample( State::EVENT, 0.0)) {}
	
	//Altitude
	if(altitude >= 0.0)
	{
		atm62->update(altitude);
	}
	else
	{
		atm62->update(0.0);
	} 

	/////
	//Gust (change mag and dir of gust every sec)
	if(errWind_Flag == 1)
	{
		gcnt = gcnt + 1;
		if(fmod((double)gcnt, 1000.0) == 0)
		{
			// wind gusts are turned off for a nominal run
			vg_mag = gaussianDistribution(0.0, gustSpd_sig);
			gpsiDir = uniformDistribution(-1 * gustDir_sig, gustDir_sig);
			gcnt = 0;
		}
	}
	//
	vwind.x = vw_mag*cos(wpsiDir*dtr) + vg_mag*cos(gpsiDir*dtr);
	vwind.y = vw_mag*sin(wpsiDir*dtr) + vg_mag*sin(gpsiDir*dtr);
	vwind.z = 0.0;
	/////

	//Atmospheric
	rho   = atm62->rho_() * (1.0+rho_Err);
	vs    = atm62->vs_()  * (1.0+vs_Err);
	p     = atm62->p_()   * (1.0+p_Err);
	q     = 0.5 * rho * speed * speed;
	amach = speed / vs;

}

void Atmos::rpt()
{

	if( State::sample( out->pt_console))
	{
		if(out->displayInput == 1)
		{
			printf( "Atmos  %10.4f\n", sys->t_sys);
		}
	}
}

double Atmos::unituni()
{
	double value;
	value=(double)rand()/RAND_MAX;
	return value;
}

double Atmos::uniformDistribution(double min,double max)
{
	double value;
	value=min+(max-min)*unituni();
	return value;
}

double Atmos::gaussianDistribution(double mean,double sig)
{

	static int iset=0;
	static double gset;
	double fac,rsq,v1,v2,value;

	if(iset==0)
	{
		do
		{
			v1=2.*unituni()-1.;
			v2=2.*unituni()-1.;
			rsq=v1*v1+v2*v2;
		} while(rsq>=1.0||rsq==0);

		fac=sqrt(-2.*log(rsq)/rsq);
		gset=v1*fac;
		iset=1;
		value=v2*fac;
	}
	else
	{
		iset=0;
		value=gset;
	}
	return value*sig+mean;
}