//-----------------------------------------------------------//
// File: Airframe.cpp
// 
// This class provides information about the airframe properties.
//                                                           
// Developer: Dennis Strickland                              
//-----------------------------------------------------------//

#include "Airframe.h"
#include "System.h"
#include "Output.h"

Airframe::Airframe( string infile, Output *outp, System *sysp) {

	cout << "AIRFRAME CONSTRUCTED" << endl;

	//Local access
	out   = outp;
	sys   = sysp;

	addIntegrator(t1000, xd);

	//Get Inputs
	Filer *ff = new Filer( "./input/"+infile);
	dtHz      = ff->getDouble( "dtHz");
	spin_Flag = ff->getInt( "spin_Flag");
	pScale    = ff->getDouble( "pScale");
	xrail     = ff->getDouble( "xrail");
	errTipoff_Flag = ff->getInt( "errTipoff_Flag" );
	wpo_mean = ff->getDouble( "wpo_mean");
	wqo_mean = ff->getDouble( "wqo_mean");
	wro_mean = ff->getDouble( "wro_mean");
	wpo_sig  = ff->getDouble( "wpo_sig");
	wqo_sig  = ff->getDouble( "wqo_sig");
	wro_sig  = ff->getDouble( "wro_sig");
	sref     = ff->getDouble( "sref");
	dia      = ff->getDouble( "dia");
	anoz     = ff->getDouble( "anoz");
	ximu     = ff->getDouble( "ximu");
	delete ff;

}

void Airframe::init()
{
	t1000 = 0.0;
	tick = 0.0;
	kt = 0;
	altg_max = -999999.99;
	sdt = 1.0/dtHz;
}

void Airframe::update() {

	if( State::sample())
	{
		tick = (double)kt++;
	}
	if( State::sample( State::EVENT, 0.0)) {}
	if( State::sample(sdt)) {}
}

void Airframe::rpt()
{
	if( State::sample( out->pt_console))
	{
		if(out->displayInput == 1)
		{
			printf( "Airframe  %10.4f\n", sys->t_sys);
		}
	}
}
