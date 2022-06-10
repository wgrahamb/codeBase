//-----------------------------------------------------------//
// File: GuideLaw.cpp
// 
// This class determines pitch/yaw guidance commands.
//                                                           
// Developer: Dennis Strickland
//      Date: 30 September 2019
//-----------------------------------------------------------//

#include "GuideLaw.h"
#include "System.h"
#include "Output.h"

#ifdef SIXDOF
GuideLaw::GuideLaw( string infile, Output *outp, System *sysp) {

	cout << "GUIDANCE LAW CONSTRUCTED" << endl;

	//Local access
	out        = outp;
	sys        = sysp;

	string guideFile = "input/"+infile;

	//Input
	Filer *ff = new Filer( guideFile);
	ff->setLine0( "GuideLaw");
	sdtHz          = ff->getDouble( "sdtHz");
	tguide         = ff->getDouble( "tguide");
	t_fin_lock_off = ff->getDouble( "t_fin_lock_off");
	del_pm_Lim     = ff->getDouble( "del_pm_Lim");
	gamdot_Lim     = ff->getDouble( "gamdot_Lim");
	gamdot_Type    = ff->getInt( "gamdot_Type");

	//Waypoint Guidance
	gamY_T = ff->getDouble( "gamY_T");
	gamP_T = ff->getDouble( "gamP_T");
	//
	// wp1 is initialized from navproc->pt in init()

	delete ff;

	xks_vs_t = new Table1ff( guideFile.c_str());
	xks_vs_t->read( "xks_vs_t", true);

	xkn1_vs_t = new Table1ff( guideFile.c_str());
	xkn1_vs_t->read( "xkn1_vs_t", true);

	xkn2_vs_t = new Table1ff( guideFile.c_str());
	xkn2_vs_t->read( "xkn2_vs_t", true);
	//
	phi_c_vs_t = new Table1ff( guideFile.c_str());
	phi_c_vs_t->read( "phi_c_vs_t", true);
	//
	gamd_q_vs_t = new Table1ff( guideFile.c_str());
	gamd_q_vs_t->read( "gamd_q_vs_t", true);
	//
	gamd_r_vs_t = new Table1ff( guideFile.c_str());
	gamd_r_vs_t->read( "gamd_r_vs_t", true);

	//Output
	out->add("tgtX", &wp1.x);
	out->add("tgtY", &wp1.y);
	out->add("tgtZ", &alt);

}
#else
GuideLaw::GuideLaw() {
	// embedded SW loads parameters here
}
#endif

void GuideLaw::init()
{
	kt = 0;
	acomy   = acomz   = phi_c = 0.0;
	acomy1  = acomz1  = 0.0;
	acomy_t = acomz_t = 0.0;
	gamd_q  = gamd_r  = 0.0;

	t1    = 0.0;
	t_del = 0.0;
	xkni  = 0.0;

	mtpos   = Vecff(0.0, 0.0, 0.0);
	mtpos_b = Vecff(0.0, 0.0, 0.0);
	mtvel   = Vecff(0.0, 0.0, 0.0);
	mtvel_b = Vecff(0.0, 0.0, 0.0);

	//Waypoint Guidance
	gamY_d = gamY_T * dtr;
	gamP_d = gamP_T * dtr;
	//
	velTan_d.x =  cos(gamP_d)*cos(gamY_d);
	velTan_d.y =  cos(gamP_d)*sin(gamY_d);
	velTan_d.z = -sin(gamP_d);

	rngsq = 0.0;

	//Guide Flag
	guide_Flag = -1;

	//Guidance Gain variables
	prev_pm_n.x = 0.0;
	prev_pm_n.y = 0.0;
	prev_pm_n.z = 0.0;
	xkn     = 0.0;
	xkn_flg = -1;
	t_kf    = 0.0;

	sdt = 1.0/sdtHz;

	//Flight Path Angle Tolerance (deg)
	tol = 3.0;

	cnt = -1;
	t_eps = 1.0e-7;

}

void GuideLaw::update(
		bool guideLawInputProcessExecuting,
		Vecff guideLawInputLTFWaypoint,
		Vecff guideLawInputLTFPos,
		Vecff guideLawInputLTFVel,
		bool guideLawInputNav200Valid,
		double guideLawInputRollAngle,
		Matff guideLawInputDCMNav,
		Matff guideLawInputRolledToNonRolledBodyDCM,
		double guideLawInputAlpha,
		double guideLawInputBeta
	)
{
	#ifdef SIXDOF
	if (guideLawInputProcessExecuting) {
	#else
	{
	#endif

		//Autopilot does not run until launch
		if(sys->brk_Flag != 1) { return; }

		wp1.x = guideLawInputLTFWaypoint.x;
		wp1.y = guideLawInputLTFWaypoint.y;
		wp1.z = guideLawInputLTFWaypoint.z;
		alt = wp1.z * -1;

		//Roll Command
		phi_c   = phi_c_vs_t->interp( sys->t_flight );

		//User-Defined Command
		gamd_q_t = gamd_q_vs_t->interp( sys->t_flight );
		gamd_r_t = gamd_r_vs_t->interp( sys->t_flight );

		//Missile Position/Velocity from Navigation
		prev_pm_n = pm_n;
		//
		pm_n   = guideLawInputLTFPos;
		vm_n   = guideLawInputLTFVel;
		vm_mag = vm_n.mag();

		//Rolling to Non-rolling DCM
		BodyRollToNR_DCM = guideLawInputRolledToNonRolledBodyDCM;
		
		rngToGo     = wp1 - pm_n; // OG METHOD
		rngToGo_u   = rngToGo.unit();
		rngToGo_mag = rngToGo.mag();

		//Enter Guiance after Guide Time and Time of Fin Lock
		if((sys->t_flight >= tguide) && (sys->t_flight >= t_fin_lock_off) && (sys->mode < MODE_TERMGUIDE_ACTIVE))
		{
			//Guidance at 200Hz
			if(guideLawInputNav200Valid)
			{
				//Guidance Mode 1
				if(guide_Flag == -1)
				{
					sys->mode = MODE_MIDCOURSE_MODE1;
					t1 = sys->t_flight;
					guide_Flag = 1;
					sys->traceEvent("Midcourse Guidance Mode 1 Active");
				}

				//Note: phi=0 means Rolling Frame (phi != 0 means Non-Rolling Frame.
				// Terminal guidance is best with Non-Rolling Frame.
				phi = guideLawInputRollAngle; //0.0
				
				//--------------------------------------------//
				//Angle of Attack in Non-Rolling Frame
				alpha_NR = guideLawInputAlpha * dtr;
				beta_NR  = guideLawInputBeta  * dtr;

				//
				Vecff  aoa_NR    = Vecff(0.0, alpha_NR, beta_NR);
				Quatff aoa_NR_q  = aoa_NR.getQuatff();
				Matff  BodyToVel = aoa_NR_q.getDCM();

				//Velocity tangent unit vector in Inertial frame
				velTan_i_u = vm_n.unit();

				//Sine of Beta unit vector in Inertial frame (rolling)
				sbeta_i_u = velTan_i_u.cross(rngToGo_u); 

				//Sine of Beta unit vector in Body frame (rolling)
				sbeta_b_u = guideLawInputDCMNav * sbeta_i_u;

				//Sine of Beta unit vector in Body frame (non-rolling)
				sbeta_b_nr = BodyRollToNR_DCM * sbeta_b_u;

				//Sine of Beta unit vector in Velocity frame (non-rolling)
				sbeta_vel_nr = BodyToVel * sbeta_b_nr;

				//Missile relative velocity Cross desired velocity in Inertial frame (rolling)
				vXvd_i_u = velTan_i_u.cross(velTan_d);

				//Missile relative velocity Cross desired velocity in Body frame (rolling)
				vXvd_b_u = guideLawInputDCMNav * vXvd_i_u;

				//Missile relative velocity Cross desired velocity in Body frame (non-rolling)
				vXvd_b_nr = BodyRollToNR_DCM * vXvd_b_u;

				//Missile relative velocity Cross desired velocity in Velocity frame (non-rolling)
				vXvd_vel_nr = BodyToVel * vXvd_b_nr;

				//---------------------------------------------------//
				//Acceleration Command Logic
				//
				//Detect KF update (change in nav position)
				del_pm = (pm_n - prev_pm_n).mag();
				//
				if((del_pm >= del_pm_Lim) && (sys->t_flight >= tguide+1.0))
				{
					xkn_flg = 1;
					t_kf = sys->t_flight;
				}
				//Determine Guidance Gain
				if(xkn_flg == -1)
				{
					xkn = xkn1_vs_t->interp( sys->t_flight - t1 );
				}
				else
				{
					xkn = xkn2_vs_t->interp( sys->t_flight - t_kf );
				}
				//
				xks = xks_vs_t->interp( sys->t_flight - t1 );

				//Gamma Dot Command
				if(gamdot_Type == 1)
				{
					gamd_q1 = xkn*(xks*(sbeta_vel_nr.y) - (vXvd_vel_nr.y));
					gamd_r1 = xkn*(xks*(sbeta_vel_nr.z) - (vXvd_vel_nr.z));
				}
				else
				{
					gamd_q1 = xkn*(xks*asin(sbeta_vel_nr.y) - asin(vXvd_vel_nr.y));
					gamd_r1 = xkn*(xks*asin(sbeta_vel_nr.z) - asin(vXvd_vel_nr.z));
				}

				//Final Command
				gamd_q = gamd_q1 + gamd_q_t;
				gamd_r = gamd_r1 + gamd_r_t;

			} //200Hz
		} // sys->t_flight > t_guide
	} // executing 
}

#ifdef SIXDOF
void GuideLaw::rpt()
{
	if(State::sample(out->pt_console))
	{

		if(out->displayInput == 1)
		{
			printf( "GuideLaw  %10.4f\n", sys->t_flight);
			printf( "          %10.6f %10.6f\n", acomy, acomz);
		}

	}
}
#endif
