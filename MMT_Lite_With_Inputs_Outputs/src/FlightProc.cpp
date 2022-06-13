//-----------------------------------------------------------//
// File: FlightProc.cpp
// 
// This class determines estimates of atmospherics, aero and
// missile mass properties for use by the autopilot.
//                                                           
// Developers: Dennis Strickland
//      Date: 30 September 2019
//-----------------------------------------------------------//

#include "FlightProc.h"
#include "System.h"
#include "Output.h"

#ifdef SIXDOF 
FlightProc::FlightProc( string infile, Output *outp, System *sysp) {

	cout << "FLIGHT PROCESS CONSTRUCTED" << endl;

	//Local access
	out     = outp;
	sys     = sysp;

	string fltFile = "input/"+infile;
	
	Filer *ff = new Filer( fltFile);
	ff->setLine0( "FlightProc");
	sdt      = ff->getDouble( "sdt");
	sref_est = ff->getDouble( "sref");
	dia_est  = ff->getDouble( "dia");
	ximu_est = ff->getDouble( "ximu");
	yimu_est = ff->getDouble( "yimu");
	zimu_est = ff->getDouble( "zimu");
	delete ff;

	rho_vs_hm = new Table1ff( fltFile.c_str());
	rho_vs_hm->read( "rho_vs_hm", true);
	vs_vs_hm = new Table1ff( fltFile.c_str());
	vs_vs_hm->read( "vs_vs_hm", true);
	cna_table = new Table1ff( fltFile.c_str());
	cna_table->read( "cna_table", true);
	ajy_table = new Table1ff( fltFile.c_str());
	ajy_table->read( "ajy_table", true);
	amass_table = new Table1ff( fltFile.c_str());
	amass_table->read( "amass_table", true);

	xcg_table = new Table1ff( fltFile.c_str());
	xcg_table->read( "xcg_table", true);
	//
	ycg_table = new Table1ff( fltFile.c_str());
	ycg_table->read( "ycg_table", true);
	//
	zcg_table = new Table1ff( fltFile.c_str());
	zcg_table->read( "zcg_table", true);

	cnd_table = new Table1ff( fltFile.c_str());
	cnd_table->read( "cnd_table", true);
	cmq_table = new Table1ff( fltFile.c_str());
	cmq_table->read( "cmq_table", true);
	cma0_table = new Table1ff( fltFile.c_str());
	cma0_table->read( "cma0_table", true);
	cmd0_table = new Table1ff( fltFile.c_str());
	cmd0_table->read( "cmd0_table", true);

	ajx_table = new Table1ff( fltFile.c_str());
	ajx_table->read( "ajx_table", true);
	cld_table = new Table1ff( fltFile.c_str());
	cld_table->read( "cld_table", true);
	clp_table = new Table1ff( fltFile.c_str());
	clp_table->read( "clp_table", true);

}
#else
FlightProc::FlightProc()
{
	// embedded SW loads parameters here
}
#endif

void FlightProc::init() {

	t1000 = 0.0;
	kt = 0;
	hm_est = 0.0;
	rho = 0.0;
	vs = 0.0;
	vb_est.x = 0.0;
	vb_est.y = 0.0;
	vb_est.z = 0.0;
	v_est = 0.0;
	q_est = 0.0;
	amach = 0.0;
	cna_est = 0.0;
	ajx_est = 0.0;
	ajy_est =  0.0;
	amass_est =  0.0;
	xcg_est = 0.0;
	ycg_est = 0.0;
	zcg_est = 0.0;
	vb_est = 0.0;
	cnd_est = 0.0;
	cld_est = 0.0;
	clp_est = 0.0;
	cmq_est = 0.0;
	cma0_est = 0.0;
	cmd0_est = 0.0;

	w_est     = Vecff(0.0, 0.0, 0.0);
	ab_est    = Vecff(0.0, 0.0, 0.0);
	gb_est    = Vecff(0.0, 0.0, 0.0);
	vb_est    = Vecff(0.0, 0.0, 0.0);
	//
	w_est_nr  = Vecff(0.0, 0.0, 0.0);
	ab_est_nr = Vecff(0.0, 0.0, 0.0);
	gb_est_nr = Vecff(0.0, 0.0, 0.0);
	vb_est_nr = Vecff(0.0, 0.0, 0.0);
	alpha_est_nr = beta_est_nr = alphaTot_est_nr = 0.0;

	cnt = 0;
	cnt1 = 0;

}

void FlightProc::handleInput(NavigationState const &navigationState)
{

	navSolutionAvailable = true;
	currentAltitude = navigationState.missileLTFPosition_[2];

	Vecff missileLocalVelocity;
	missileLocalVelocity.x = navigationState.missileLTFVelocity_[0];
	missileLocalVelocity.y = navigationState.missileLTFVelocity_[1];
	missileLocalVelocity.z = navigationState.missileLTFVelocity_[2];

	Vecff eulerAngles;
	eulerAngles.x = navigationState.missileLTFEulerAngles_[0];
	eulerAngles.y = navigationState.missileLTFEulerAngles_[1];
	eulerAngles.z = navigationState.missileLTFEulerAngles_[2];

	Matff dcm = eulerAngles.getDCM();

	missileBodyVelocity = dcm * missileLocalVelocity;

	missileBodyRate.x = navigationState.missileBodyRate_[0];
	missileBodyRate.y = navigationState.missileBodyRate_[1];
	missileBodyRate.z = navigationState.missileBodyRate_[2];

	Vecff localGravityEstimate;
	localGravityEstimate.x = 0.0;
	localGravityEstimate.y = 0.0;
	localGravityEstimate.z = 9.81;

	missileBodyGravityEstimate = dcm * localGravityEstimate;

	missileBodyAcceleration.x = navigationState.missileBodyAcceleration_[0];
	missileBodyAcceleration.y = navigationState.missileBodyAcceleration_[1];
	missileBodyAcceleration.z = navigationState.missileBodyAcceleration_[2];

	missileRollAngle = navigationState.missileLTFEulerAngles_[0];

}

void FlightProc::update()
{
	#ifdef SIXDOF
	if (navSolutionAvailable)
	{
	#else
	{
	#endif

	//Autopilot does not run until launch
	if(sys->brk_Flag != 1) { return; }

	hm_est = -1 * currentAltitude;
	rho    =  rho_vs_hm->interp( hm_est);
	vs     =  vs_vs_hm->interp( hm_est);

	ajx_est   = ajx_table->interp( sys->t_flight);
	ajy_est   = ajy_table->interp( sys->t_flight);
	amass_est = amass_table->interp( sys->t_flight);

	xcg_est   = xcg_table->interp( sys->t_flight);
	ycg_est   = ycg_table->interp( sys->t_flight);
	zcg_est   = zcg_table->interp( sys->t_flight);

	//body-velocity estimates
	vb_est.x = missileBodyVelocity.x;
	vb_est.y = missileBodyVelocity.y;
	vb_est.z = missileBodyVelocity.z;
	v_est    = vb_est.mag();
	q_est    = rho * v_est * v_est / 2.0;

	amach    = v_est / vs;
	cna_est  = cna_table->interp( amach);
	cnd_est  = cnd_table->interp( amach);
	cld_est  = cld_table->interp( amach);
	clp_est  = clp_table->interp( amach);
	cmq_est  = cmq_table->interp( amach);
	cma0_est = cma0_table->interp( amach);
	cmd0_est = cmd0_table->interp( amach);

	//body-rate esimtates
	w_est.x = missileBodyRate.x;
	w_est.y = missileBodyRate.y;
	w_est.z = missileBodyRate.z;

	//Gravity estimates
	gb_est.x = missileBodyGravityEstimate.x;
	gb_est.y =missileBodyGravityEstimate.y;
	gb_est.z =missileBodyGravityEstimate.z;

	//body-acceleration estimates
	ab_est.x = missileBodyAcceleration.x;
	ab_est.y = missileBodyAcceleration.y;
	ab_est.z = missileBodyAcceleration.z;

	//Compute in Non-Rolling Frame
	n_phi_hat_nr = -1 * missileRollAngle;

	//Rolling to Non-Rolling Frame DCM
	BodyRollToNR_DCM = Vecff(n_phi_hat_nr, 0.0, 0.0).getDCM(); // I think this may be a part of what I am missing

	//Body-Rates non-rolling
	w_est_nr = BodyRollToNR_DCM * w_est;
		
	//Gravity non-rolling
	gb_est_nr = BodyRollToNR_DCM * gb_est;

	//Body-Acceleration non rolling
	ab_est_nr = BodyRollToNR_DCM * ab_est;

	//Body-Acceleration + Gravity non-rolling
	abg_est_nr = ab_est_nr + gb_est_nr;

	//Body-Velocity non-rolling
	vb_est_nr = BodyRollToNR_DCM * vb_est;

	//Angle of Attack (deg)
	angle_of_attack(vb_est_nr, alpha_est_nr, beta_est_nr, alphaTot_est_nr); 

	}
}

#ifdef SIXDOF 
void FlightProc::rpt()
{
	if (State::sample(out->pt_console))
	{
		if(out->displayInput == 1)
		{
			printf( "FlightP   %10.4f\n", sys->t_flight);
		}
	}
}
#endif

void FlightProc::angle_of_attack(Vecff v_b, float &alphp, float &alphy, float &alpha)
{

	if (v_b.z == 0.0 && v_b.x == 0.0)
	{
		alphp = 0.0;
	}
	else
	{
		alphp = atan2(v_b.z, v_b.x) * rtd;
	}

	if( v_b.mag() == 0.0)
	{
		alphy = 0.0;
	}
	else
	{
		alphy = asin( v_b.y / v_b.mag()) * rtd;
	}

	float yzVel = sqrt(v_b.y * v_b.y + v_b.z * v_b.z);
	if(v_b.x == 0.0)
	{
		alpha = 0.0;
	}
	else
	{
		alpha = atan2(yzVel, v_b.x) * rtd;
	}

} 

