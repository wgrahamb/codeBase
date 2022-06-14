//------------------------------------------------------//
// File: Motion.cpp
//
// This class implements the equations of motion for
// an unsymmetric missile and solves for the missile's
// translational/rotational dynamics.
//
//
// Developer: Dennis Strickland
//------------------------------------------------------//

#include "Motion.h"
#include "System.h"
#include "Airframe.h"
#include "MassProp.h"
#include "reFactoredGeometry.h"
#include "Output.h"

//Complex Aero Constructor
Motion::Motion(string infile, Output *outp, System *sysp) {

	cout << "MOTION CONSTRUCTED" << endl;

	//Input
	Filer *ff = new Filer( "./input/"+infile);
	ff->setLine0( "Motion");
	xd = ff->getDouble( "xd");
	//Detent Force to constrain Missile while on Launcher
	spinFlagIn = ff->getInt("spin_Flag");
	pScaleIn = ff->getDouble("pScale");
	Fdetent = ff->getDouble( "Fdetent");
	breakwireDelay = ff->getDouble( "breakwireDelay");
	deg_tol = ff->getDouble( "deg_tol");
	delete ff;

	//Local access
	out   = outp;
	sys   = sysp;
	re_factored_geo = new reFactoredGeometry();

	//Integration (States, Derivatives)
	addIntegrator( ipos.x, ivel.x);
	addIntegrator( ipos.y, ivel.y);
	addIntegrator( ipos.z, ivel.z);
	//
	addIntegrator( ivel.x, ivel_d.x);
	addIntegrator( ivel.y, ivel_d.y);
	addIntegrator( ivel.z, ivel_d.z);
	//
	addIntegrator( omegaB.x, omegaB_d.x);
	addIntegrator( omegaB.y, omegaB_d.y);
	addIntegrator( omegaB.z, omegaB_d.z);
	//
	addIntegrator( quat[0], quat_d[0]);
	addIntegrator( quat[1], quat_d[1]);
	addIntegrator( quat[2], quat_d[2]);
	addIntegrator( quat[3], quat_d[3]);

	out->add( "phi",  &LTFtoBODY_roll_r);
	out->add( "theta", &LTFtoBODY_pitch_r);
	out->add( "psi",   &LTFtoBODY_yaw_r);

	out->add( "rollRate", &omegaB.x);
	out->add( "pitchRate", &omegaB.y);
	out->add( "yawRate", &omegaB.z);

	out->add( "sideAch", &g_load.y); 
	out->add( "normAch", &g_load.z); 

	out->add( "alpha", &alphp_r); 
	out->add( "beta", &alphy_r);

	out->add( "posX", &XYZ_pos.x);   
	out->add( "posY", &XYZ_pos.y);
	out->add( "posZ", &XYZ_pos.z);

	//Zero Values
	omegaB_d = Vec( 0.0, 0.0, 0.0);
	quat_d   = Quat( 0.0, 0.0, 0.0, 0.0);
	vb       = Vec( 0.0, 0.0, 0.0);

};

void Motion::init()
{

	re_factored_geo->init(
		sys->phi0,
		sys->tht0,
		sys->psi0,
		sys->latg0,
		sys->lon0,
		sys->altg0,
		sys->azRef0
	);

	t1000 = 0.0;
	tick = 0.0;
	kt = 0;
	cout.precision(12);

	//Initialize
	force    = Vec( 0.0, 0.0, 0.0);
	moment   = Vec( 0.0, 0.0, 0.0);
	omegaB   = Vec( 0.0, 0.0, 0.0);
	omegaB_d = Vec( 0.0, 0.0, 0.0);
	tipoff   = Vec( 0.0, 0.0, 0.0);
	sf_b     = Vec( 0.0, 0.0, 0.0);
	ipos_d   = Vec( 0.0, 0.0, 0.0);
	ivel_d   = Vec( 0.0, 0.0, 0.0);
	ivel     = Vec( 0.0, 0.0, 0.0);
	quat_d   = Quat( 0.0, 0.0, 0.0, 0.0);
	angMom   = Vec( 0.0, 0.0, 0.0);
	uvw      = Vec( 0.0, 0.0, 0.0);
	t_detent_release = -1.0;

	aphi = 0.0;
	alph  = 0.0;
	alphp = 0.0;
	alphy = 0.0;

	g_load_yz_mag = 0.0;
	g_load_yz_max = 0.0;

	//Initial Euler Angles from NEDgtoBODY
	euler = re_factored_geo->NEDgtoBODY.getEuler();
	Vec euler_deg = euler * rtd;
	euler_deg.extract( roll, pitch, yaw );
	//
	//Initial Euler Angles from NEDbtoBODY
	eulerNbtoB = re_factored_geo->NEDbtoBODY.getEuler();
	Vec eulerNbtoB_deg = eulerNbtoB * rtd;
	eulerNbtoB_deg.extract( NEDbtoBODY_roll, NEDbtoBODY_pitch, NEDbtoBODY_yaw );
	//
	//Initial Euler Angles from LTFtoBODY
	eulerLTF = re_factored_geo->LTFtoBODY.getEuler();
	Vec eulerLTF_deg = eulerLTF * rtd;
	eulerLTF_deg.extract( LTFtoBODY_roll, LTFtoBODY_pitch, LTFtoBODY_yaw );
	eulerLTF.extract( LTFtoBODY_roll_r, LTFtoBODY_pitch_r, LTFtoBODY_yaw_r );

	//Initial ECEF Position/Velocity
	epos = re_factored_geo->lla2ecef(re_factored_geo->latg, re_factored_geo->lon, re_factored_geo->altg);
	//
	epos_i = re_factored_geo->lla2ecef(sys->latg0*dtr, sys->lon0*dtr, sys->altg0); // ECEF position of LTF origin
	//
	evel = Vec(0.0, 0.0, 0.0);

	//Initial NEDg Position/Velocity
	npos_i = Vec(0.0, 0.0, -sys->altg0); // NED position of LTF origin

	//Calculate NEDg Position Relative to LTF origin
	delPos = (epos - epos_i);
	//
	npos   = (re_factored_geo->ECEFtoNEDg * delPos) + npos_i;
	npos_0 = npos;
	//
	nvel = re_factored_geo->ECEFtoNEDg * evel;

	// Initialize LTF Position/Velocity
	ltf_pos = re_factored_geo->NEDtoLTF * (npos - npos_i);
	XYZ_pos.x = ltf_pos.x;
	XYZ_pos.y = ltf_pos.y;
	XYZ_pos.z = -1 * ltf_pos.z;
	ltf_vel = re_factored_geo->NEDtoLTF * nvel;
	
	//Initial ECI Position/Velocity
	initECI(epos, evel, re_factored_geo->ECItoECEF, ipos, ivel);

	//Initialize Body Rotation States to Earth Rotation
	omegaB = re_factored_geo->ECItoBODY * Vec(0.0, 0.0, omegae);

	//Initial Range
	rng += delPos.mag(); 

	//Initial Attitude Quaternion
	quat = re_factored_geo->ECItoBODY.getQuat();

	//Initial Body-Relative Velocity
	vb   = re_factored_geo->NEDgtoBODY * nvel;
	vrel = vb.mag();

	//Update ECEF Position
	epos  = re_factored_geo->ECItoECEF * ipos;
	
	//Calculate Gravity in ECEF Frame
	egrav = re_factored_geo->gravity(epos);
	
	//Calculate Gravity in ECI Frame
	igrav = re_factored_geo->ECItoECEF.transpose() * egrav;
	
	//Calculate Gravity in Body Frame
	bgrav = re_factored_geo->ECItoBODY * igrav;

	//Normal Force while Missile on Launcher
	massIn = 9.65; // Rough estimate of starting mass.
	Nx =  -1 * massIn * bgrav.x;
	Ny = -1 * massIn * bgrav.y;
	Nz = -1 * massIn * bgrav.z;

	//Initialize G-Load
	g_load = Vec(0.0, 0.0, 0.0);

	//Tolerance
	tol = 1.0e-3;

	//Tipoff Flag
	tip_Flag = -1;

	//Rail Flag
	rail_Flag = -1;

	//Pitch-tumble Flag
	pos_pitch_Flag = -1;
	neg_pitch_Flag = -1;
	pitch_tumble_Flag = -1;

	//Yaw-tumble Flag
	pos_yaw_Flag = -1;
	neg_yaw_Flag = -1;
	yaw_tumble_Flag = -1;

	//Pitch-Yaw-Tumble Flag
	pitch_yaw_tumble_Flag = -1;

	// Error zero for now.
	wpo_err = 0.0;
	wqo_err = 0.0;
	wro_err = 0.0;

	railLengthIn = launchRailLength;

}

void Motion::update(
	Vec LTFWindVel,
	Vec aeroForce,
	Vec aeroMoment,
	Vec motorForce,
	Vec motorMoment,
	double mass,
	Mat inertiaTensor
)
{

	if(State::sample())
	{
		tick = (double)kt++;
	}
	//
	if(State::sample( State::EVENT, 0.0)) {}

	// Populate variables.
	aeroForceIn = aeroForce;
	aeroMomentIn = aeroMoment;
	motorForceIn = motorForce;
	motorMomentIn = motorMoment;
	massIn = mass;
	inertiaTensorIn = inertiaTensor;

	// Update geometry.
	double totalSysTime = sys->t_sys - sys->pretime;
	re_factored_geo->update(totalSysTime, quat);

	//Sum Forces and Moments

	SumForcesMoments();

	//Calculate 6DOF States
	Calc_6DOF();

	//Update Truth States
	UpdateTruth(LTFWindVel);

}

void Motion::rpt()
{
	if( State::sample(out->pt_console))
	{
		if(out->displayInput == 1)
		{
			printf( "Motion     t: %10.4f\n", sys->t_sys);
			printf( "          npos: %10.2f %10.2f %10.2f\n", npos.x, npos.y, npos.z);
			printf( "          nvel: %10.3f %10.3f %10.3f\n", nvel.x, nvel.y, nvel.z);
		}
	}
}

//--------------------------------------------------------//

void Motion::SumForcesMoments()
{

	//Assign Induced Roll
	if(spinFlagIn == 1)
	{ //Induced Roll from Spin Vanes
		if((rail_Flag == 1) && (sys->t_sys <= sys->t_bo))
		{
			motorMomentIn.x = inducedMoment();
		}
	}

	//Sum Forces
	force.x  = aeroForceIn.x + motorForceIn.x;
	force.y  = aeroForceIn.y + motorForceIn.y;
	force.z  = aeroForceIn.z + motorForceIn.z;
	
	//Sum Moments
	moment.x = aeroMomentIn.x + motorMomentIn.x;
	moment.y = aeroMomentIn.y + motorMomentIn.y;
	moment.z = aeroMomentIn.z + motorMomentIn.z;

}

//--------------------------------------------------------//

void Motion::Calc_6DOF()
{

	//Update ECEF Position
	epos  = re_factored_geo->ECItoECEF * ipos;

	//Calculate Gravity in ECEF Frame
	egrav = re_factored_geo->gravity(epos);

	//Calculate Gravity in ECI Frame
	igrav = re_factored_geo->ECItoECEF.transpose() * egrav;
	
	//Calculate Gravity in Body Frame
	bgrav = re_factored_geo->ECItoBODY * igrav;

	//Compute ECI Linear Velocity (Position Derivative)
	ipos_d = ivel;

	//Transform Summed Forces into ECI Frame
	sumF_ECI = re_factored_geo->ECItoBODY.transpose() * force;

	//Range from Launch
	rng = Vec(epos - epos_i).mag();

	//Constraints while missile in tube
	double rng_xy = sqrt((npos.x-npos_0.x)*(npos.x-npos_0.x) + (npos.y-npos_0.y)*(npos.y-npos_0.y));

	if(rng_xy <= railLengthIn)
	{

		//Zero Moments
		moment( 0.0, 0.0, 0.0);

		//Apply Earth-Rate while missile in launcher
		omegaB = re_factored_geo->ECItoBODY * Vec(0.0, 0.0, omegae);

	}
	else
	{

		//Free Flight Mode
		if(rail_Flag == -1)
		{ 
			sys->mode = MODE_FREEFLIGHT;
			sys->t_free = sys->t_sys;
			rail_Flag =  1;
			sys->traceEvent("Off Rail (Freeflight)");
		}

		//Normal Force no longer valid since missile not on launcher
		Nx = 0.0;
		Ny = 0.0;
		Nz = 0.0;
	}

	//Missile Constrained by Normal Force and Detent Force
	if((force.x <= (Nx + Fdetent)) && (t_detent_release < 0.0) )
	{
		//Compute ECI Linear Acceleration (Velocity Derivatives)
		//On Launcher
		omegaI  = re_factored_geo->ECItoBODY.transpose() * omegaB;
		Vec wXr = omegaI.cross(ipos);
		ivel_d = omegaI.cross(wXr);

	}
	else
	{

		//Compute ECI Linear Acceleration (Velocity Derivatives)
		//During Free-Flight
		ivel_d = sumF_ECI/massIn + igrav;

		// If detent force exceeded, start breakwire timer
		if (State::sample() && t_detent_release < 0.0)
		{
				t_detent_release = State::t;
				sys->traceEvent("Detent Release");
		}

	}

	if (State::sample() && sys->brk_Flag != 1)
	{
		// If breakwire timer elapsed, then switch to breakwire mode
		if ((t_detent_release >= 0.0) && (State::t >= t_detent_release + breakwireDelay))
		{
			if(sys->brk_Flag == -1)
			{ 
				sys->mode = MODE_BREAKWIRE;
				sys->t_break = sys->t_sys;
				sys->brk_Flag =  1;
				sys->traceEvent("Breakwire (Flight)");
			}
		}
	}

	//Apply Tip-off at Tube Exit
	if( State::sample() && rng_xy > railLengthIn && tip_Flag == -1)
	{
		//Add Tipoff to body-rates
		tipoff.x = (wpo_mean +  wpo_err) * rtd;
		tipoff.y = (wqo_mean +  wqo_err*cos(re_factored_geo->phiLnch) + wro_err*sin(re_factored_geo->phiLnch)) * rtd;
		tipoff.z = (wro_mean + -wqo_err*sin(re_factored_geo->phiLnch) + wro_err*cos(re_factored_geo->phiLnch)) * rtd;
		omegaB.x = omegaB.x + tipoff.x*dtr;
		omegaB.y = omegaB.y + tipoff.y*dtr;
		omegaB.z = omegaB.z + tipoff.z*dtr;
		tip_Flag = 1;
	}

	//----------------------------------------------------------------------//
	//                                                                      //
	//     Body-Relative Rotational Accelerations (Non-Symmetric Missile)   // 
	//     ==================================    // 
	//                                                                      // 
	//        --- MOMENT VECTOR ---                                         // 
	//                        .                                             // 
	//        _               _       _              _                      // 
	//        M    =   iten * w   +   w  X  ( iten * w )                    // 
	//                         b       b              b                     // 
	//                                                                      // 
	//        --- SOLVE FOR WDOT ---                                        // 
	//        .                                                             // 
	//        _            -1     _       _              _                  // 
	//        w    =   iten   * ( M   -   w  X  ( iten * w )                // 
	//         b                           b              b                 // 
	//----------------------------------------------------------------------//

	//Calculate Angular Momentum (h = iten * w)
	angMom = inertiaTensorIn * omegaB;

	//Calculate Nutation (coning) Angle (angle btwn body-axis and angular momentum vector)
	Vec xbod = Vec(1.0, 0.0, 0.0);
	conAng   = vecAng(xbod, angMom);

	//Check for tumble conditions
	checkTumble();

	//w X h
	Vec wXh = omegaB.cross(angMom);

	//Calculate Body Rotational Accelerations (wdot)
	omegaB_d     = inertiaTensorIn.inv() * (moment - wXh);
	omegaB_d_Mag = omegaB_d.mag();

	//Quaternion Derivative
	quat_d[0] = -0.5 * ( omegaB.x * quat[1] + omegaB.y * quat[2] + omegaB.z * quat[3]);
	quat_d[1] =  0.5 * ( omegaB.x * quat[0] - omegaB.y * quat[3] + omegaB.z * quat[2]);
	quat_d[2] =  0.5 * ( omegaB.x * quat[3] + omegaB.y * quat[0] - omegaB.z * quat[1]);
	quat_d[3] = -0.5 * ( omegaB.x * quat[2] - omegaB.y * quat[1] - omegaB.z * quat[0]);

	//Normalize Quaternion
	if(State::ready)
	{
		quat = quat.normalize();
	}

}

//--------------------------------------------------------//

void Motion::UpdateTruth(Vec LTFWindVel)
{

	cout.precision(12);

	//Specific Force Acceleration (used by IMU)
	if(rail_Flag != 1)
	{
		//Subtract off gravity if still on launcher
		sf_b = re_factored_geo->ECItoBODY * (ivel_d - igrav);
	}
	else
	{
		sf_b = force/massIn;
	}

	//G-Load
	g_load = sf_b/G;

	g_load_yz_mag = pow((pow(g_load.z, 2.0) + pow(g_load.y, 2.0)), 0.5);

	if(g_load_yz_mag > g_load_yz_max)
	{
		g_load_yz_max = g_load_yz_mag;
	}

	//Update ECEF Position/Velocity
	update_ECEF(ipos, ivel, re_factored_geo->ECItoECEF, epos, evel);

	//Compute ECEF linear Accelerations
	evel_d = re_factored_geo->ECItoECEF * ivel_d;

	//Update Body Frame Velocities (no wind)
	uvw = re_factored_geo->ECEFtoBODY * evel;

	//Compute Relative Velocity in Body-Relative Coordinates
	//(used in Aero for calculating Alpha and Beta)
	//This is the velocity of the missile with respect to the wind velocity
	//(ie. vrel = missile velocity in body - wind velocity in body)
	Vec vwindb = re_factored_geo->LTFtoBODY * LTFWindVel;
	vb       = uvw - vwindb;
	vrel     = vb.mag();
	vb_yzmag = sqrt(vb.y*vb.y + vb.z*vb.z);
	vmag     = uvw.mag();

	//Angle of Attack
	angle_of_attack( vb, alphp, alphy, alph, aphi);
	alphp_r = alphp * dtr;
	alphy_r = alphy *dtr;

	//Update Euler Angles from NEDgtoBODY and NEDbtoBODY
	euler = re_factored_geo->NEDgtoBODY.getEuler();
	Vec euler_deg = euler * rtd;
	euler_deg.extract( roll, pitch, yaw );
	//
	eulerNbtoB = re_factored_geo->NEDbtoBODY.getEuler();
	Vec eulerNbtoB_deg = eulerNbtoB * rtd;
	eulerNbtoB_deg.extract( NEDbtoBODY_roll, NEDbtoBODY_pitch, NEDbtoBODY_yaw);
	eulerNbtoB.extract( NEDbtoBODY_roll_rad, NEDbtoBODY_pitch_rad, NEDbtoBODY_yaw_rad);

	//Non-rolling pitch rate (alpha-Total frame) (Needed by Aero)
	qnr = omegaB.y * cos(roll*dtr);

	//Determine Geodetic Latitude, Longitude and Altitude
	re_factored_geo->ecef2lla(epos, &re_factored_geo->latg, &re_factored_geo->lon, &re_factored_geo->altg);

	//Calculate NED Velocity
	nvel = re_factored_geo->ECEFtoNEDb * evel;

	//Calculate Flight Path Angles
	if(nvel.mag() <= tol)
	{
		gamH = 0.0;
		gamV = 0.0;
	}
	else
	{
		gamH = atan2(ltf_vel.y, ltf_vel.x)*rtd;
		gamV = asin(-ltf_vel.z/ltf_vel.mag())*rtd;
	}

	//Calculate NEDg Position Relative to LTF origin
	delPos = (epos - epos_i);

	npos   = (re_factored_geo->ECEFtoNEDg * delPos) + npos_i;

	//Calculate Launch-Tangent-Frame (LTF) Relative to LTF origin
	ltf_pos = re_factored_geo->NEDtoLTF * (npos - npos_i);
	XYZ_pos.x = ltf_pos.x;
	XYZ_pos.y = ltf_pos.y;
	XYZ_pos.z = -1 * ltf_pos.z;
	ltf_vel = re_factored_geo->NEDtoLTF * nvel;
	//
	//Update Euler Angles from LTFtoBODY
	eulerLTF = re_factored_geo->LTFtoBODY.getEuler();
	Vec eulerLTF_deg = eulerLTF * rtd;
	eulerLTF_deg.extract( LTFtoBODY_roll, LTFtoBODY_pitch, LTFtoBODY_yaw );
	eulerLTF.extract( LTFtoBODY_roll_r, LTFtoBODY_pitch_r, LTFtoBODY_yaw_r );

}

//--------------------------------------------------------//

void Motion::initECI(Vec epos, Vec evel, Mat i2e, Vec &ipos, Vec &ivel) {

	//---------------------------------------------------------//
	// Calcuates ECEF states from ECI states with
	// coordinate transformation and relative velocity vector
	//
	// Input:   ipos - position vector in ECI frame
	//          ivel - velocity vector in ECI frame
	//
	// Output:  epos - position vector in ECEF frame
	//          evel - velocity vector in ECEF frame
	//
	// Developer: Dennis Strickland
	//---------------------------------------------------------//

	//Transform ECEF position to ECI position
	ipos = i2e.transpose() * epos;

	//Transform ECEF velocity to ECI velocity
	Vec i_ivel = i2e.transpose() * evel;

	//Calculate local velocity (wxR)
	Vec omeg = Vec(0.0, 0.0, omegae);
	Vec wXr  = omeg.cross(ipos);

	//Add local velocity of ECEF frame to get true ECI velocities
	ivel = i_ivel + wXr;
}


//--------------------------------------------------------//
void Motion:: checkTumble() {

	//Checks for various tumble conditions
	
	
	///////////
	//Calculate tumble condition based on pitch/yaw angles
	//
	//
	//Check if pitch hits in the range of +/- 90 deg

	if (pitch >= (90.0 - deg_tol))
	{
		pos_pitch_Flag = 1;
	}


	if (pitch <= (-90.0 + deg_tol))
	{
		neg_pitch_Flag = 1;
	}

	if ((pos_pitch_Flag == 1) && (neg_pitch_Flag == 1))
	{
		pitch_tumble_Flag = 1;
	}

	//Check if yaw hits in the range of +/- 180 degrees
	
	if(  yaw >= (180.0 - deg_tol) )
	{
		pos_yaw_Flag = 1;
	}

	if(  yaw <= (-180.0 + deg_tol) )
	{
		neg_yaw_Flag = 1;
	}

	if ( (pos_yaw_Flag == 1) && (neg_yaw_Flag == 1))
	{
		yaw_tumble_Flag = 1;
	}

	// If pitch or yaw tumble flag activated
	if ( (pitch_tumble_Flag == 1) || (yaw_tumble_Flag == 1))
	{
		pitch_yaw_tumble_Flag = 1;
	}
	//////////////////////////////

	//Calculate Precession Rate (deg/s)
	if((sys->t_sys >= sys->t_bo) && ((abs(conAng) <= 90.0-cangTol) || (abs(conAng) >= 90.0+cangTol)) )
	{
		precRate = (inertiaTensorIn[0][0]*omegaB.x / ((inertiaTensorIn[1][1]-inertiaTensorIn[0][0])*cos(conAng*dtr)))*rtd;
	
		//Calculate Prec Rate 2 based on derivatives of rpy
		if ((pitch >= abs(89.0) && (pitch <= abs(91.0))))
		{
			yaw_d = 0.0;
		}
		else
		{
			yaw_d = ((omegaB.y * sin(roll*dtr)) + (omegaB.z * cos(yaw*dtr)))/cos(pitch*dtr);
		}

		roll_d = omegaB.x + yaw_d * sin(pitch*dtr);
		pitch_d = (omegaB.y * cos(roll*dtr)) - (omegaB.z * sin(roll*dtr));

		precRate2 = (pow( (pow(yaw_d, 2.0) + pow(pitch_d, 2.0)), 0.5)) * rtd;
	}
	else
	{
		precRate = 0.0;
		precRate2 = 0.0;
	}

}

//--------------------------------------------------------//

void Motion::update_ECEF(Vec ipos, Vec ivel, Mat i2e, Vec &epos, Vec &evel) {

	//---------------------------------------------------------//
	// Calcuates ECEF states from ECI states with
	// coordinate transformation and relative velocity vector
	//
	// Input:   ipos - position vector in ECI frame
	//          ivel - velocity vector in ECI frame
	//
	// Output:  epos - position vector in ECEF frame
	//          evel - velocity vector in ECEF frame
	//
	// Developer: Dennis Strickland
	//---------------------------------------------------------//

	//Transform ECI position to ECEF position
	epos = i2e * ipos;

	//Transform ECI velocity to ECEF velocity
	//(still inertial at this point)
	Vec i_evel = i2e * ivel;

	//Calculate local velocity (wxR)
	Vec omeg = Vec(0.0, 0.0, omegae);
	Vec wXr  = omeg.cross(epos);

	//Subtract local velocity of ECI frame to get true ECEF velocities
	evel = i_evel - wXr;
}

//-------------------------------------------------//

double Motion::vecAng( Vec x, Vec y) {

	double arg, ang;
	double tol = 1.0e-12;

	double dot  = x.dot(y);
	double xmag = x.mag();
	double ymag = y.mag();

	if((dot > -tol) && (dot < tol))
	{
		dot = 0.0;
	}

	if((xmag == 0.0) || (ymag == 0.0))
	{
		arg = 1.0;
	}
	else
	{
		arg = dot/(xmag*ymag);
	}

	//Angle between vectors
	ang = asin(arg)*rtd;

	return ang;
}

//-------------------------------------------------//

void Motion::angle_of_attack(Vec v_b, double &alphp, double &alphy, double &alpha, double &aphi)
{

	alphp = atan2_0( v_b.z, v_b.x) * rtd;
	if( v_b.mag() == 0.0)
	{
		alphy = 0.0;
	}
	else
	{
		alphy = asin( v_b.y / v_b.mag()) * rtd;
	}

	double yzVel = sqrt( v_b.y * v_b.y + v_b.z * v_b.z);
	if( v_b.x == 0.0)
	{
		alpha = 0.0;
	}
	else
	{
		alpha = atan2_0( yzVel, v_b.x) * rtd;
	}

	//Aero Roll Angle
	aphi = atan2_0(vb.y,vb.z) * rtd;
	if(aphi > 180.0)
	{
		aphi = aphi - 360.0;
	}
	else if(aphi < -180.0)
	{
		aphi = aphi + 360.0;
	}

}

double Motion::atan2_0( double y, double x)
{
	if( x == 0.0 && y == 0.0)
	{
		return 0.0;
	}
	else
	{
		return atan2( y, x);
	} 
}

//-------------------------------------------------//

double Motion::inducedMoment() {

	double tfree;

	static double prev_delMom;
	static double delMom = 0.0;
	double delT;
	double mom;

	//Curve-fit coefficients
	double A = -303232.0;
	double B =  1.97258e+06;
	double C = -5.3265e+06;
	double D =  7.72015e+06;
	double E = -6.47481e+06;
	double F =  3.14117e+06;
	double G = -821596.0;
	double H =  93981.2;

	//Delta time from free flight
	delT = sys->t_sys - sys->t_free - sys->sys_dt;

	//Previous value
	prev_delMom = delMom;

	//Curve-fit of spin-rate
	delMom = (A * (delT)*(delT)*(delT)*(delT)*(delT)*(delT)*(delT)*(delT) +
			B * (delT)*(delT)*(delT)*(delT)*(delT)*(delT)*(delT) +
			C * (delT)*(delT)*(delT)*(delT)*(delT)*(delT) +
			D * (delT)*(delT)*(delT)*(delT)*(delT) +
			E * (delT)*(delT)*(delT)*(delT) +
			F * (delT)*(delT)*(delT) +
			G * (delT)*(delT) +
			H * (delT))* dtr * pScaleIn;

	//Compute roll moment based on spin rate (M = I * pdot)
	if(prev_delMom != 0.0)
	{
		mom = (delMom - prev_delMom)/sys->sys_dt * inertiaTensorIn[0][0];
	}
	else
	{
		mom = 0.0;
	}

	return mom;

}

