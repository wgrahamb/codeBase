//------------------------------------------------------//
// File: Motor.cpp
//
// This class computes the forces and moments due to thrust
// during flight.
//                                                      
// Developer: Dennis Strickland                          
//------------------------------------------------------//

#include "Motor.h"
#include "System.h"
#include "Output.h"
#include "SimConstants.h"


Motor::Motor( string infile, Output *outp, System *sysp )
{

	cout << "MOTOR CONSTRUCTED" << endl;

	//Local access
	out   = outp;
	sys   = sysp;

	string motorFile = "./input/"+infile;

	//Inputs
	Filer *ff = new Filer( motorFile);
	tma_tht_sig   = ff->getDouble( "tma_tht_sig");
	tma_psi_sig   = ff->getDouble( "tma_psi_sig");
	//
	rthxb_sig     = ff->getDouble( "rthxb_sig");
	rthyb_sig     = ff->getDouble( "rthyb_sig");
	rthzb_sig     = ff->getDouble( "rthzb_sig");
	errMotor_Flag = ff->getInt( "errMotor_Flag");
	delete ff;

	//Read Thrust Curve (Ambient)
	thrust_vs_t = new Table1( motorFile.c_str());
	thrust_vs_t->read( "thrust_vs_t", true);

	//States and Derivatives
	addIntegrator(massp, masspd);

	//Determine Total Impulse of Ambient Motor
	double a1,a2;
	int thrPts_Amb = 2138;
	double thr_dt_Amb = 0.0005;
	itot_Amb = 0.0;
	for (int i=0; i<=thrPts_Amb; i++)
	{
		double thrVal1 = thrust_vs_t->interp( i*thr_dt_Amb);
		double thrVal2 = thrust_vs_t->interp( (i+1)*thr_dt_Amb);
		if (thrVal1 < thrVal2) 
		{
			a1 = thrVal1*thr_dt_Amb;
			a2 = 0.5*(thrVal2-thrVal1)*thr_dt_Amb;
			itot_Amb = itot_Amb + a1 + a2;
		}
		else if (thrVal2 <= thrVal1)
		{
			a1 = thrVal2*thr_dt_Amb;
			a2 = 0.5*(thrVal1-thrVal2)*thr_dt_Amb;
			itot_Amb = itot_Amb + a1 + a2;
		}
	}
	//Specific Impulse
	double fullMass = 9.653;
	double emptyMass = 7.197;
	isp_Amb = itot_Amb / ((fullMass - emptyMass) * G);

}

void Motor::init()
{
	tick = 0.0;
	kt = 0;
	after_launch = false;

	double anoz = 0.000426642;

	force    = Vec( 0.0, 0.0, 0.0);
	moment   = Vec( 0.0, 0.0, 0.0);

	//Errors
	//Determine Errors (if any)
	bool disable = true;
	if(errMotor_Flag == 1)
	{
		disable = false;
	}

	tma_tht_Err = 0.0008;
	tma_psi_Err = 0.0016;
	//
	rthxb_Err   = 0.0;
	rthyb_Err   = -7.62e-5;
	rthzb_Err   = 1.4e-3;

	//Initialize Propellant Mass
	double fullMass = 9.653;
	double emptyMass = 7.197;
	massp = fullMass - emptyMass;

	//Thrust Misalignment
	//Pitch-Yaw
	tma_tht = tma_tht_Err;
	tma_psi = tma_psi_Err;

	tux = cos(tma_tht) * cos(tma_psi);
	tuy = sin(tma_psi);
	tuz = -1 * cos(tma_psi) * sin(tma_tht);

	//Hot/Cold Motor Tuning Parameters
	time_param = 0.001;
	thr_param  = 0.00005;

	thrust = 0.0;
	masspd = 0.0;

	burnout_flg = -1;
}

void Motor::update(
	double XCenterOfGravity,
	double YCenterOfGravity,
	double ZCenterOfGravity,
	double airTemp,
	double airTempNominal,
	double pressure
) {

	if( State::sample()) {tick = ( double)kt++;}
	if( State::sample( State::EVENT, 0.0)) {}

	//Return if not at least Launch Mode
	if(!sys->launch)
	{ 
		tm = sys->t_sys;
		return; 
	}

	// This is required for HWIL due to t_sys discontinuity at launch
	if (!after_launch)
	{
		after_launch = true;
		tm = 0 - sys->last_sim_dt;
		sys->traceEvent("Ignition (Launch)");
	}

	//Hot/Cold Motor Scale Factors
	time_fac = 1.0 - time_param * (airTemp - airTempNominal);
	thr_fac  = 1.0 + thr_param  * (airTemp - airTempNominal);
	
	//Hot/Cold Motor Scale Factor (Combination time and thrust)
	thrust_TempScale = 1.0/(time_fac*thr_fac);

	//Motor time (scale burn time based on Hot/Cold motor)
	tmotor = (sys->t_sys - tm - sys->sys_dt) / time_fac;

	//Thrust (scale thrust based on Hot/Cold motor)
	thrust_vac = thrust_vs_t->interp( tmotor ) * thrust_TempScale;
	thrust = thrust_vac - anoz * pressure;
	if( thrust < 0.0)
	{
		thrust = 0.0;
	}

	//Mass flow rate
	masspd = -thrust_vac / isp_Amb / G;

	//Thrust Force (vector based on mis-alignment angles)
	force.x = thrust*tux;
	force.y = thrust*tuy;
	force.z = thrust*tuz;

	//Thrust Moment Arms (force vector offset from cg)
	double rxarm = rthxb_Err - XCenterOfGravity;
	double ryarm = rthyb_Err - YCenterOfGravity;
	double rzarm = rthzb_Err - ZCenterOfGravity;

	//Thrust Moments (based on angular mis-aligment and offset from cg)
	moment.x = force.z*ryarm - force.y*rzarm;
	moment.y = force.x*rzarm - force.z*rxarm;
	moment.z = force.y*rxarm - force.x*ryarm;

	//Total Magnitude Side Force due to Thrust (includes all thrust errors)
	forceY   = moment.y/XCenterOfGravity;
	forceZ   = moment.z/XCenterOfGravity;
	forceYZ  = sqrt(forceY*forceY + forceZ*forceZ);
	if (forceZ == 0.0 && forceY == 0.0)
	{
		forceAng = 0.0;
	}
	else
	{
		forceAng = atan2(forceZ, forceY) * rtd;
	}

	//Set Burnout Mode
	if((force.mag() <= 0.0) && (tmotor > 0.0) && (burnout_flg == -1))
	{
		sys->mode = MODE_BURNOUT;
		sys->t_bo = sys->t_sys;
		burnout_flg = 1;
		sys->traceEvent("Burnout");
	}

}

void Motor::rpt()
{
	if( State::sample( out->pt_console))
	{
		if(out->displayInput == 1)
		{
			printf( "Motor  %10.4f\n", sys->t_sys);
		}
	}
}
