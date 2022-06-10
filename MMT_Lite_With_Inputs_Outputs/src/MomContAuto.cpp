//-----------------------------------------------------------//
// File: MomContAuto.cpp
// 
// This class determines the moment command for use in the 
// actuator.
//                                                           
// Developer: Dennis Strickland                            
//      Date: 30 September 2019
//-----------------------------------------------------------//

#include "MomContAuto.h"

#include "System.h"
#include "Output.h"
#include "SimConstants.h"


#ifdef SIXDOF 
MomContAuto::MomContAuto(string infile, Output *outp, System *sysp) {

	cout << "PITCH AND YAW AUTOPILOT CONSTRUCTED" << endl;

	//Local access
	out        = outp;
	sys        = sysp;

	string momFile = "input/"+infile;

	//Input
	Filer *ff = new Filer( momFile);
	ff->setLine0( "MomContAuto");
	t0        = ff->getDouble( "t0");
	sdtHz     = ff->getDouble( "sdtHz");
	alph_max  = ff->getDouble( "alph_max");
	tdcp_gain = ff->getDouble( "tdcp_gain");
	tdcy_gain = ff->getDouble( "tdcy_gain");
	delete ff;

	//AP Pole Placement Gain Tables
	xA_table = new Table1ff( momFile.c_str());
	xA_table->read( "xA_table", true);
	//
	xwn_table = new Table1ff( momFile.c_str());
	xwn_table->read( "xwn_table", true);
	//
	xzeta_table = new Table1ff( momFile.c_str());
	xzeta_table->read( "xzeta_table", true);

	//Aero Coefficient Curve-Fit Tables
	cna_a1_table = new Table1ff( momFile.c_str());
	cna_a1_table->read( "cna_a1_table", true);
	//
	cna_a2_table = new Table1ff( momFile.c_str());
	cna_a2_table->read( "cna_a2_table", true);
	//
	cma_b1_table = new Table1ff( momFile.c_str());
	cma_b1_table->read( "cma_b1_table", true);
	//
	cma_b2_table = new Table1ff( momFile.c_str());
	cma_b2_table->read( "cma_b2_table", true);

#else
MomContAuto::MomContAuto() {
// embedded SW loads parameters here
#endif
}

void MomContAuto::init()
{

	kt = -1;
	tick = 0.0;
	mcp = mcy = 0.0;
	mcp0 = mcy0 = 0.0;
	acomy = acomz = 0.0;
	delp = dely = 0.0;
	amax = 0.0;

	mcy_icar = 0.0;
	mcp_icar = 0.0;

	sdt = 1.0/sdtHz;
	
	aaLim = 1.0;

	wyold = 0.0;
	wzold = 0.0;
	wyold_nr = 0.0;
	wzold_nr = 0.0;

	y1y = 0.0;
	y1z = 0.0;

	gdy_err0 = 0.0;
	gdz_err0 = 0.0;

	theta_dot = 0.0;
	psi_dot   = 0.0;
	//
	theta_ddotp = 0.0;
	psi_ddotp   = 0.0;

	gamd_qc_nr = 0.0;
	gamd_rc_nr = 0.0;

	//Coefficients for third order equation for Pitch/Yaw Autopilot
	//Pole Placement
	q_est = 0.0;
	A_ap    = xA_table->interp( q_est );
	wn_ap   = xwn_table->interp( q_est );
	zeta_ap = xzeta_table->interp( q_est );
	//
	c0 = A_ap * wn_ap*wn_ap;
	c1 = wn_ap*wn_ap + 2.0*A_ap*zeta_ap*wn_ap;
	c2 = A_ap + 2.0*zeta_ap*wn_ap;
	tn = 0.0/A_ap;

	xmrp = 0.0;

	gamd_ramp = 0.0;

	//Max Fin Limit
	del_max = 7.0;

	//Autopilot Active Flag
	auto_Flag = -1;

	t1 = -1; //DEBUG***

	cnt = 0;

}

void MomContAuto::update(
	double normGuidanceCommand,
	double sideGuidanceCommand,
	double gamDotLimit,
	bool processExecuting,
	double rollAngle,
	double mach,
	double dynamicPressure,
	double referenceArea,
	double CNA_in,
	double CND_in,
	double CMD0_in,
	double AJY_in,
	double mass,
	double centerOfGrav,
	double XIMU_in,
	double referenceDiameter,
	double speed,
	Vecff nonRolledBodyRate,
	Vecff bodyGravEst,
	Vecff bodyAccelEst,
	double alpha,
	double beta,
	double aoa
) {
	#ifdef SIXDOF
	if (processExecuting)
	{
		tick = (double)kt++;
	#else
	{
	#endif

		if( sys->t_flight >= t0)
		{

			//Autopilot Active Flag
			if(auto_Flag == -1)
			{
				sys->mode = MODE_PY_AUTOP_ACTIVE;
				auto_Flag = 1;
				sys->traceEvent("Pitch/Yaw Autopilot Active");
			}

			//Inputs from other objects
			amach = mach;     // mach number
			q_est = dynamicPressure;     // dyn. press.
			sref  = referenceArea;  // aero ref. area
			cna   = CNA_in;   // lin. nor. force coef.
			cnd   = CND_in;   // lin. nor. force coef per fin delta.
			cmd0  = CMD0_in;  // lin. pitch moment coef per fin delta.
			ajy   = AJY_in;   // Iyy
			amass = mass; // mass
			xcg   = centerOfGrav;
			ximu  = XIMU_in;
			dia   = referenceDiameter;
			vm    = speed;
			qsd   = q_est * sref * dia;
			qs    = q_est * sref;

			xms_cg  =  xcg - ximu;

			//Non-roll frames
			w_nr   = nonRolledBodyRate;
			gb_nr  = bodyGravEst;
			abg_nr = bodyAccelEst;

			alpha_hat_nr    = alpha    * dtr;
			beta_hat_nr     = beta     * dtr;
			alphaTot_hat_nr = aoa * dtr;

			//Aero Curve-Fit
			//Get aero curve-fit coefficients
			c_a1 = cna_a1_table->interp( amach );  //CNa-1st
			c_a2 = cna_a2_table->interp( amach );  //CNa-2nd
			c_b1 = cma_b1_table->interp( amach );  //CMa-1st
			c_b2 = cma_b2_table->interp( amach );  //CMa-2nd

			//CNa and CMa estimate (curve-fit)
			cna_est = (c_a1 + c_a2*alphaTot_hat_nr); 
			dx      = (xcg-xmrp)/dia; 
			cma_est = (c_b1 + dx*c_a1) + (c_b2 + dx*c_a2)*alphaTot_hat_nr; 

			//CNd estimate (curve-fit)
			cnd_est = cnd/(del_max*dtr);   //divided by fin limit

			//CMd estimate (curve-fit)
			cmd_est = cmd0/(del_max*dtr) + cnd_est * dx;

			//Aero Coefficients
			malpha_est = qsd / ajy * (cma_est + 0.0*cma2_est*alphaTot_hat_nr);
			mdelta_est = (qsd*cmd_est + 0.0*thr_est*xmom_arm)/ajy;
			ndelta_est = qs*cnd_est / (amass*vm);

			//Note: If ax is noisey, may want to use estimate of (thrust- drag/(m*v))
			//nalpha_est = qs*cna_est/(amass*vm) + 1.0*(axb_nr/vm);
			nalpha_est = qs*cna_est/(amass*vm) + 1.0*((abg_nr.x-gb_nr.x)/vm);
			nn_est = (nalpha_est - malpha_est*(ndelta_est/mdelta_est));

			//Gains
			//Note: Coefficients (c0,c1,c2) are set from AFMomRes.cpp to avoid timestep lag
			kn = c0 / nn_est;
			kq = c2 - nalpha_est;
			ka = c1 + malpha_est - kq*nn_est - c0*tn;

			//Compensation for Fin dynamics (non-rolling frame)
			qdot_est_nr = (w_nr.y - wyold_nr) / sdt;
			rdot_est_nr = (w_nr.z - wzold_nr) / sdt;
			wyold_nr    =  w_nr.y;
			wzold_nr    =  w_nr.z;
			qdotx  = -qdot_est_nr;
			rdotx  =  rdot_est_nr;
			xndomd = 0.0;

			if(mdelta_est != 0.0)
			{
				xndomd = -(ndelta_est/mdelta_est)*vm;
			}

			xms_cgx = xms_cg - xndomd;

			//Body-To-Velocity DCM
			Matff BodyToVel = Vecff(0.0, -alpha_hat_nr, beta_hat_nr).getDCM();

			//Acceleration Correction Factor due to fin dynamics (non-rolling)
			Vecff arm            = Vecff(xms_cgx, 0.0, 0.0);
			Vecff angAccImu      = Vecff(0.0, qdotx, -rdotx);  //correction factor
			Vecff angAccImu_X_r  = angAccImu.cross(arm);
			//
			Vecff accB           = Vecff(abg_nr.x, abg_nr.y, abg_nr.z);
			Vecff accB_corrected = accB + angAccImu_X_r;
			//
			Vecff gd = BodyToVel * accB_corrected; //accB only????

			//GammmaDot of system in Velocity frame
			gdot_eq = -gd.z/vm;
			gdot_er =  gd.y/vm;

			bhe  = atan(tan(beta_hat_nr)*cos(alpha_hat_nr));
			sbhe = sin(bhe);
			cbhe = cos(bhe);
			salp = sin(alpha_hat_nr);
			calp = cos(alpha_hat_nr);
				 
			double phi_hat_nr = rollAngle;

			gamd_q = normGuidanceCommand;
			gamd_r = sideGuidanceCommand;

			//Limit GammaDot Command ------------------------//
			gamdot_Lim = gamDotLimit * G / vm;  //gammadot limit

			gamdot_Mag = sqrt(gamd_q*gamd_q + gamd_r*gamd_r);
			
			gamdot_Scale = 1.0;
			if(gamdot_Mag > gamdot_Lim)
			{
				gamdot_Scale = gamdot_Lim/gamdot_Mag; 
			}
			gamd_q = gamd_q * gamdot_Scale;
			gamd_r = gamd_r * gamdot_Scale;

			//Error Signal (Gamma-Dot)
			gdy_err = gamd_q - gdot_eq;
			gdz_err = gamd_r - gdot_er;

			//Intergrate Error Signal with no coupling terms since non-rolling
			y1y = y1y + 0.5*(gdy_err + gdy_err0)*sdt; // + wx*y1z*sdt;
			y1z = y1z + 0.5*(gdz_err + gdz_err0)*sdt; // - wx*y1y*sdt;
			gdy_err0 = gdy_err;
			gdz_err0 = gdz_err;

			x2y = y1y + tn*gdy_err;
			x2z = y1z + tn*gdz_err;

			//Determine fin command (non-rolling)
			delta_qc = x2y*kn + (-kq*w_nr.y - ka*alpha_hat_nr);
			delta_rc = x2z*kn + (-kq*w_nr.z + ka*beta_hat_nr);

			mcp0 = delta_qc;
			mcy0 = delta_rc;

			//In a rolling autopilot, qnr,rnr are the desired non-rolling body rates 
			//expressed in rolling body frame and are thus identical to the 
			//body q and r. Using these term gives better results than the output 
			//of the integrators c (thdot, psdot)since they represent what SHOULD be obtained.

			dqdes = delta_qc;
			drdes = delta_rc;

			//Determine angular acceleration (non-rolling)
			//Gyrodynamics assumed zero and IYY assumed equal to IZZ (Moment Equations)
			theta_ddot = dqdes + malpha_est*alpha_hat_nr; // + wx*wz;
			psi_ddot   = drdes - malpha_est * beta_hat_nr; //  - wx*wy;

			//Integrate to form angular rates (non-rolling)
			theta_dot   = theta_dot + 0.5*(theta_ddot + theta_ddotp)*sdt;
			theta_ddotp = theta_ddot;
			//
			psi_dot   = psi_dot + 0.5*(psi_ddot + psi_ddotp)*sdt;
			psi_ddotp = psi_ddot;

			dis_thtd = theta_dot - w_nr.y;
			dis_psid = psi_dot   - w_nr.z;

			//TDC moment command
			delp_dis = tdcp_gain * (dis_thtd) * aaLim;
			dely_dis = tdcy_gain * (dis_psid) * aaLim;
 
			mcp_icar = delp_dis;
			mcy_icar = dely_dis;

			//---------------------------------------------------//
			//Total Acceleration Command
			mcp = mcp0 + mcp_icar;
			mcy = mcy0 + mcy_icar;

			// Integrator Reset.
			//************ Pitch/Yaw Channel Gains and Reset(if needed) *******//
			double yawFinCommandRads, pitchFinCommandRads;
			if(sys->t_flight >= t0)
			{

				//Coefficients for third order equation for Pitch/Yaw Autopilot
				//Pole Placement
				A_ap    = xA_table->interp( q_est );
				wn_ap   = xwn_table->interp( q_est );
				zeta_ap = xzeta_table->interp( q_est );

				c0 = A_ap * wn_ap*wn_ap;
				c1 = wn_ap*wn_ap + 2.0*A_ap*zeta_ap*wn_ap;
				c2 = A_ap + 2.0*zeta_ap*wn_ap;
				tn = 0.0/A_ap;

				//Gains
				kn = c0 / nn_est;
				kq = c2 - nalpha_est;
				ka = c1 + malpha_est - kq*nn_est - c0*tn;
				//
				double xki_py = kn;
				double xka_py = ka;
				double xkg_py = kq;

				// Fin command estimates.
				double mcp_b = mcp*cos(n_phi_hat_nr) - mcy*sin(n_phi_hat_nr);
				double mcy_b = mcp*sin(n_phi_hat_nr) + mcy*cos(n_phi_hat_nr);
				yawFinCommandRads = mcy_b / mdelta_est;
				pitchFinCommandRads = mcp_b / mdelta_est;

				if ((yawFinCommandRads * rtd) > del_max)
				{
					if (yawFinCommandRads > 0)
					{
						yawFinCommandRads = del_max * dtr;
					}
					else if (yawFinCommandRads < 0)
					{
						yawFinCommandRads = -1 * del_max * dtr;
					}
				}

				if ((pitchFinCommandRads * rtd) > del_max)
				{
					if (pitchFinCommandRads > 0)
					{
						pitchFinCommandRads = del_max * dtr;
					}
					else if (pitchFinCommandRads < 0)
					{
						pitchFinCommandRads = -1 * del_max * dtr;
					}
				}

				//Limited moment
				double mcplmtot = pitchFinCommandRads * mdelta_est;
				double mcylmtot = yawFinCommandRads * mdelta_est;

				double mcplm =  mcplmtot*cos(n_phi_hat_nr) + mcylmtot*sin(n_phi_hat_nr);
				double mcylm = -mcplmtot*sin(n_phi_hat_nr) + mcylmtot*cos(n_phi_hat_nr);

				double mcp0lm   = mcplm - mcp_icar;
				double mcy0lm   = mcylm - mcy_icar;

				//pitch integrator reset
				double yyy = 1.0/xki_py * (mcp0lm + xkg_py*w_nr.y + xka_py*alpha_hat_nr);
				y1y = yyy;

				//yaw integrator reset
				double zzz = 1.0/xki_py * (mcy0lm + xkg_py*w_nr.z - xka_py*beta_hat_nr);
				y1z = zzz;

			} //momcontauto->t0

			yawFinCommandDegrees = yawFinCommandRads * rtd;
			pitchFinCommandDegrees = pitchFinCommandRads * rtd;

		} // State::t >> t0
	} // executing
}

#ifdef SIXDOF 
void MomContAuto::rpt()
{
	if( State::sample( out->pt_console))
	{
		if (out->displayInput == 1)
		{
			printf( "MomContA  %10.4f %10d\n", sys->t_flight, kt);
		}

	}
}

#endif

