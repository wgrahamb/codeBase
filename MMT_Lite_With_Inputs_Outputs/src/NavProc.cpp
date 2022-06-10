//-----------------------------------------------------------//
// File: NavProc.cpp
// 
// This class processes the navigation input to provide
// navigated position/velocity/attitude output.
//                                                           
// Developer: Dennis Strickland
//      Date: 30 September 2019
//-----------------------------------------------------------//

#include "NavProc.h"
#include "System.h"
#include "Output.h"
#include "Util.h"
#include "SimConstants.h"

Integrator::Integrator() {/*{{{*/
	k   = 0;
	sdt = 0; 
	y0  = 0; 
	y   = 0; 
	u0  = 0; 
}

void Integrator::init( float y, float sdt) {
	y0 = y;
	this->sdt = sdt;
	k   = 0;
	u0  = 0.0; 
}

float Integrator::update( float u) {
	y  = ( u0 + u) / 2.0 * sdt + y0;
	y0 = y;
	u0 = u;
	return y;
}/*}}}*/


#ifdef SIXDOF 
NavProc::NavProc(string infile, Output *outp, System *sysp) {

	cout << "NAVIGATION PROCESS CONSTRUCTED" << endl;

	//Local access
	out        = outp;
	sys        = sysp;

	// imu = new IMU();
	// imu->readInputAndInit("highFidelityIMU/MMTHighFidelityIMUInput.txt");

	//Inputs
	Filer *ff = new Filer( "input/"+infile);
	ff->setLine0( "NavProc");
	sdtHz   = ff->getDouble( "sdtHz");
	burnout_time = ff->getDouble( "burnout_time");
	//
	orig_Lat = ff->getDouble( "orig_Lat");
	orig_Lon = ff->getDouble( "orig_Lon");
	orig_Altg = ff->getDouble( "orig_Altg");
	//
	tgt_Rng = ff->getDouble( "tgt_Rng");
	tgt_Az  = ff->getDouble( "tgt_Az");
	tgt_Hgt = ff->getDouble( "tgt_Hgt");

	phi0 = ff->getDouble( "phi0");
	//
	nav_sol_latency = ff->getDouble( "nav_sol_latency");
	//
	count_sf_v = ff->getDouble( "count_sf_v");
	count_sf_w = ff->getDouble( "count_sf_w");
	count_sf_w_x = ff->getDouble( "count_sf_w_x");
	//
	count_sf_sf      = ff->getDouble( "count_sf_sf");
	count_sf_omega_X = ff->getDouble( "count_sf_omega_X");
	count_sf_omega   = ff->getDouble( "count_sf_omega");
	//
	mDCM_IMU[0][0] = ff->getDouble( "mDCM_IMU[0][0]");
	mDCM_IMU[0][1] = ff->getDouble( "mDCM_IMU[0][1]");
	mDCM_IMU[0][2] = ff->getDouble( "mDCM_IMU[0][2]");
	mDCM_IMU[1][0] = ff->getDouble( "mDCM_IMU[1][0]");
	mDCM_IMU[1][1] = ff->getDouble( "mDCM_IMU[1][1]");
	mDCM_IMU[1][2] = ff->getDouble( "mDCM_IMU[1][2]");
	mDCM_IMU[2][0] = ff->getDouble( "mDCM_IMU[2][0]");
	mDCM_IMU[2][1] = ff->getDouble( "mDCM_IMU[2][1]");
	mDCM_IMU[2][2] = ff->getDouble( "mDCM_IMU[2][2]");
	delete ff;
	
#else
NavProc::NavProc() {
// embedded SW loads parameters here
#endif
}

void NavProc::init() {

	t1000 = 0.0;
	kt = 0;
	kp = 0;
	burnout_flg = -1;

	//Tolerance
	tol   = 1.0e-3;
	t_eps = 1.0e-5;

	sdt = 1.0/sdtHz;

	nav_solution_available = false;
	delt_error = 0.0;

	//Target Position (LTF)
	pt.x =  tgt_Rng;
	pt.y =  0.0;
	pt.z = -tgt_Hgt;

	//Target Velocity (LTF)
	vt = Vecff(0.0, 0.0, 0.0);
	
	wx_nav = wy_nav = wz_nav = 0.0; // pqr
	wx_int.init(0.0, sdt);

	acc_bf0( 0.0, 0.0, 0.0);
	acc_bf1( 0.0, 0.0, 0.0);
	omegaf0( 0.0, 0.0, 0.0);
	omegaf1( 0.0, 0.0, 0.0);
	vw0 = vwd = 0.0;
	accb_nav( 0.0, 0.0, 0.0);
	vb_nav( 0.0, 0.0, 0.0);
	gravb_nav(0.0, 0.0, 0.0);
	gravn_nav(0.0, 0.0, 0.0);
	omega_imu( 0.0, 0.0, 0.0);
	acc_grav_sum_p = Vecff(0.0, 0.0, 0.0);

	acc_b_NR(0.0, 0.0, 0.0);

	phi_hat  = 0.0;
	wx_nav_p = 0.0;

	qsd_p = 0.0;
	qxd_p = 0.0;
	qyd_p = 0.0;
	qzd_p = 0.0;

	cnt     = 0;
	cnt10Hz = 0;
	count_600Hz = 0;

	double phi, theta, psi;
	phi = sys->phi0 * dtr;
	theta = sys->tht0 * dtr;
	psi = sys->psi0 * dtr;

	phi_tmp   = phi;
	theta_tmp = theta;
	psi_tmp   = psi;

	phi_nav   = phi;
	theta_nav = theta;
	psi_nav   = psi;

	phi_nav_deg   = phi_nav*rtd;
	theta_nav_deg = theta_nav*rtd;
	psi_nav_deg   = psi_nav*rtd;

	quat_nav = Vecff(phi_nav, theta_nav, psi_nav).getQuatff();
	mDCM_nav = quat_nav.getDCM();

	pm_nav.x = 0.0;
	pm_nav.y = 0.0;
	pm_nav.z = 0.0;

	vm_nav_p.x = 0.0;
	vm_nav_p.y = 0.0;
	vm_nav_p.z = 0.0;

	vm_nav.x = 0.0;
	vm_nav.y = 0.0;
	vm_nav.z = 0.0;

	first10_valid = -1;
	nav10_valid   = 1;
	nav200_valid  = 1;

	#ifdef SIXDOF
	executing = false;
	#endif

}

//----------------------------------------------------//

void NavProc::update(
		Vec navprocInputLTFPosition,
		Vec navprocInputLTFVelocity,
		Vec navprocInputLTFEulerAngles,
		Vec navprocInputBodyAccel,
		Vec navprocInputBodyRate,
		double navprocInputTime,
		bool navprocInputSolutionAvailable
	)
{
	#ifdef SIXDOF
	if (navprocInputSolutionAvailable)
	{
		nav_solution_available = true;

		//10Hz Navigation Valid Flag
		nav10_valid   = 1;
		first10_valid = 1;

	}
	//executing = imu01->nav_msg_valid;
	executing = navprocInputSolutionAvailable;
	if (State::t == 0.0) executing = true;
	if (executing) {
	#else
	{
	#endif

		//Navigation does not run until PreLaunch mode
		if((!sys->prelaunch) || (first10_valid == -1)) { return; }

		sys->traceTiming("NavProc: Update");

		// nav100(); // debug
		// nav600();

		// TRUTH
		accb_nav.x = navprocInputBodyAccel.x;
		accb_nav.y = navprocInputBodyAccel.y;
		accb_nav.z = navprocInputBodyAccel.z;
		wx_nav = navprocInputBodyRate.x;
		wy_nav = navprocInputBodyRate.y;
		wz_nav = navprocInputBodyRate.z;

		if (nav_solution_available) {
			sys->traceTiming("NavProc: Received Nav Solution <<<<<<<<");
			nav_solution_available = false;
			count_600Hz = 0;
		} else {
			count_600Hz += 1;
		}
		delt = nav_sol_latency + count_600Hz*(1/600.0); // flight sw calculation
		float delt_actual = State::t - navprocInputTime; // true value not available in flight SW
		delt_error = delt - delt_actual;

		//Nav Valid Flag (600Hz flag)
		nav_valid = true;

		//Nav Valid Flag (200Hz flag)
		if(nav_valid == 1) { 
			cnt = cnt + 1;
			nav200_valid = 0;
		}
		if(fmod(cnt, 3) < t_eps ) {
			nav200_valid = 1;
			cnt = 0;
		}
		
		//Sum of Acceleration and Gravity in Local-Level Frame
		acc_grav_sum = accm_nav + gravn_nav;

		//Position Extrapolation to 600Hz (since Nav at 10Hz)
		pm_nav.x = pm_nav.x + 0.5*(vm_nav.x+vm_nav_p.x)*sdt + 0.5*(0.5*(acc_grav_sum.x+acc_grav_sum_p.x))*sdt*sdt;
		pm_nav.y = pm_nav.y + 0.5*(vm_nav.y+vm_nav_p.y)*sdt + 0.5*(0.5*(acc_grav_sum.y+acc_grav_sum_p.y))*sdt*sdt;
		pm_nav.z = pm_nav.z + 0.5*(vm_nav.z+vm_nav_p.z)*sdt + 0.5*(0.5*(acc_grav_sum.z+acc_grav_sum_p.z))*sdt*sdt;
		vm_nav_p.x = vm_nav.x;
		vm_nav_p.y = vm_nav.y;
		vm_nav_p.z = vm_nav.z;

		//Velocity Extrapolation to 600Hz (since Nav at 10Hz) (centripetal/centrifugal are neglected)
		vm_nav.x = vm_nav.x + 0.5*(acc_grav_sum.x+acc_grav_sum_p.x)*sdt;
		vm_nav.y = vm_nav.y + 0.5*(acc_grav_sum.y+acc_grav_sum_p.y)*sdt;
		vm_nav.z = vm_nav.z + 0.5*(acc_grav_sum.z+acc_grav_sum_p.z)*sdt;
		acc_grav_sum_p.x = acc_grav_sum.x;
		acc_grav_sum_p.y = acc_grav_sum.y;
		acc_grav_sum_p.z = acc_grav_sum.z;

		//Quaternion Derivative
		quat_d.s = -0.5 * ( wx_nav * quat_nav.x + wy_nav * quat_nav.y + wz_nav * quat_nav.z);
		quat_d.x =  0.5 * ( wx_nav * quat_nav.s - wy_nav * quat_nav.z + wz_nav * quat_nav.y);
		quat_d.y =  0.5 * ( wx_nav * quat_nav.z + wy_nav * quat_nav.s - wz_nav * quat_nav.x);
		quat_d.z = -0.5 * ( wx_nav * quat_nav.y - wy_nav * quat_nav.x - wz_nav * quat_nav.s);

		//Quaternion Extrapolation to 600Hz (since Nav at 10Hz) (earth-rate neglected)
		quat_nav.s = quat_nav.s + 0.5*(quat_d.s+qsd_p)*sdt;
		quat_nav.x = quat_nav.x + 0.5*(quat_d.x+qxd_p)*sdt;
		quat_nav.y = quat_nav.y + 0.5*(quat_d.y+qyd_p)*sdt;
		quat_nav.z = quat_nav.z + 0.5*(quat_d.z+qzd_p)*sdt;
		qsd_p = quat_d.s;
		qxd_p = quat_d.x;
		qyd_p = quat_d.y;
		qzd_p = quat_d.z;

		//Update Extrapolated 600Hz Solution with Navigation Solution at 10Hz
		if(nav10_valid) {

			//Update Position
			pm_nav.x = navprocInputLTFPosition.x;
			pm_nav.y = navprocInputLTFPosition.y;
			pm_nav.z = navprocInputLTFPosition.z;

			//Update Velocity
			vm_nav_p.x = navprocInputLTFVelocity.x;
			vm_nav_p.y = navprocInputLTFVelocity.y;
			vm_nav_p.z = navprocInputLTFVelocity.z;
			//
			vm_nav.x = navprocInputLTFVelocity.x;
			vm_nav.y = navprocInputLTFVelocity.y;
			vm_nav.z = navprocInputLTFVelocity.z;

			//Update Quaternion
			quat_nav = Vecff(
				navprocInputLTFEulerAngles.x,
				navprocInputLTFEulerAngles.y,
				navprocInputLTFEulerAngles.z
			).getQuatff();

			//Reset 10Hz Nav Valid Flag
			nav10_valid = 0;

		}

		//Normalize Quaternion
		quat_nav = quat_nav.normalize();

		//Inertial to Body DCM at 600Hz
		mDCM_nav = quat_nav.getDCM();

		//Euler Angles from DCM
		Vecff EulerAng = mDCM_nav.getEuler();
		//
		phi_nav   = EulerAng.x; 
		theta_nav = EulerAng.y; 
		psi_nav   = EulerAng.z; 
		//
		phi_nav_deg   = phi_nav*rtd;
		theta_nav_deg = theta_nav*rtd;
		psi_nav_deg   = psi_nav*rtd;

		//Body-Velocity at 600Hz
		vb_nav   = mDCM_nav * vm_nav;

		//Local-Level Acceleration at 600Hz
		accm_nav = mDCM_nav.transpose() * accb_nav;

		//Local-Level Gravity at 10Hz
		gravn_nav = Vecff(0.0, 0.0, 9.81);

		//Body-Gravity at 10Hz
		gravb_nav = mDCM_nav * gravn_nav;

		//Initialize phi_hat and wx integrator
		if(kp++ == 0)
		{
			phi_hat = phi_nav;  //0.0
		}

		//Integrate wx to get phi estimate
		phi_hat    = phi_hat + 0.5*(wx_nav + wx_nav_p)*sdt;
		wx_nav_p = wx_nav;

		//Flight Path Angle
		if(vm_nav.mag() <= tol)
		{
			gamH_nav = 0.0;
			gamV_nav = 0.0;
		}
		else
		{
			gamH_nav = atan2(vm_nav.y, vm_nav.x)*rtd;
			gamV_nav = asin(-vm_nav.z/vm_nav.mag())*rtd;
		}

		//Set Mode based on burnout time
		if ((sys->t_flight >= burnout_time) && (burnout_flg == -1))
		{
			sys->mode = MODE_BURNOUT_DETECT;
			burnout_flg = 1;
			sys->traceEvent("Burnout Detect");
		}
	}
}

#ifdef SIXDOF 
void NavProc::rpt()
{
	if(State::sample(out->pt_console))
	{
		if(out->displayInput == 1)
		{
			printf( "NavProc   %10.4f\n", sys->t_flight);
			printf( "          %10.3f %10.3f %10.3f  (pm)\n", pm.x, pm.y, pm.z);
		}
	}
}
#endif
