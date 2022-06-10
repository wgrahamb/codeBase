//-----------------------------------------------------------//
// File: RollAuto.cpp
// 
// This class provides roll commands and moments to the actuator
// as part of the roll autopilot functionality.
//                                                           
// Developer: Dennis Strickland
//      Date: 30 September 2019
//-----------------------------------------------------------//

#include "RollAuto.h"
#include "System.h"
#include "Output.h"

#ifdef SIXDOF 
RollAuto::RollAuto(string infile, Output *outp, System *sysp) {

	cout << "ROLL AUTOPILOT CONSTRUCTED" << endl;

	//Local access
	out        = outp;
	sys        = sysp;

	string rollFile = "input/"+infile;

	//Input
	Filer *ff = new Filer( rollFile);
	ff->setLine0( "RollAuto");
	sdtHz         = ff->getDouble( "sdtHz");
	t0            = ff->getDouble( "t0");
	delr_cmd_lim  = ff->getDouble( "delr_cmd_lim");
	kpr_gain      = ff->getDouble( "kpr_gain");
	tdc_gain      = ff->getDouble( "tdc_gain");
	delete ff;

	//AP Pole Placement Gain Tables
	xA_table = new Table1ff( rollFile.c_str());
	xA_table->read( "xA_table", true);
	//
	xwn_table = new Table1ff( rollFile.c_str());
	xwn_table->read( "xwn_table", true);
	//
	xzeta_table = new Table1ff( rollFile.c_str());
	xzeta_table->read( "xzeta_table", true);

#else
RollAuto::RollAuto() {
// embedded SW loads parameters here
#endif

}

void RollAuto::init()
{

	kt = -1;
	tick = 0.0;
	kp = 0;
	phi_hat = 0.0;
	phi_hat_deg = mom_icar = mom_c = mom_c1 = 0.0;
	sdt = 1.0/sdtHz;
	wx_integ.init(0.0,sdt);
	phi1 = 0.0;
	rolldot = 0.0;
	phi0 = 0.0;
	wx_p = 0.0;
	phi1_in   = 0.0;
	phi1_in_p = 0.0;
	aaLim = 1.0;
	auto_Flag = -1;

	//Coefficients for third order equation for Roll Autopilot
	//Pole Placement
	q = 0.0;
	A_ap    = xA_table->interp( q );
	wn_ap   = xwn_table->interp( q );
	zeta_ap = xzeta_table->interp( q );

	c0 = A_ap * wn_ap*wn_ap;
	c1 = wn_ap*wn_ap + 2.0*A_ap*zeta_ap*wn_ap;
	c2 = A_ap + 2.0*zeta_ap*wn_ap;
	tn = 0.0/A_ap;

} 

void RollAuto::update(
	bool navprocExecuting,
	double axialMomentOfInertia_in,
	Vecff nonRolledBodyRate_in,
	double mach_in,
	double cld_in,
	double q_in,
	double sref_in,
	double dia_in,
	double phi_in
)
{
	#ifdef SIXDOF
	if (navprocExecuting)
	{
		tick = (double)kt++;
	#else
	{
	#endif
		//Inputs
		ajx   = axialMomentOfInertia_in;
		w_nr  = nonRolledBodyRate_in;
		amach = mach_in;
		cld   = cld_in * rtd;
		q     = q_in;
		sref  = sref_in;
		dia   = dia_in;
		qsd   = q * sref * dia;

		//Get integral p*dt
		phi_hat     = phi_in;
		phi_hat_deg = phi_hat * rtd;

		//Execute Roll Autopilot
		if (sys->t_flight >= t0)
		{
			//Autopilot Active Flag
			if (auto_Flag == -1)
			{
				 sys->mode = MODE_ROLL_AUTOP_ACTIVE;
				 auto_Flag = 1;
				 sys->traceEvent("Roll Autopilot Active");
			}

			//Ldelta Estimate
			ldelta_est = (qsd * cld) / ajx;

			//FB Gain (Coefficients in AFMomRes.cpp)
			knr = c0;            //K1
			kpr = c2 * kpr_gain; //K2
			kar = (c1 - tn*c0);  //B

			//Phi Command
			phi_c  = 0.0;

			//Integrator Input
			phi1_in = phi_c - phi_hat;

			//Integration ()
			phi1 = phi1 + 0.5*(phi1_in + phi1_in_p)*sdt;
			phi1_in_p = phi1_in;

			//Roll Moment Command
			mom_c1 = knr * phi1 - kar * phi_hat - kpr * w_nr.x;

			//TDC (Torque Disturbance Compensation)
			rolldd  = mom_c1;   
			//rolldd  = contactres->afmomres->mom_c_lm;   
			rolldot = rolldot + rolldd*sdt;

			//TDC moment command
			mom_icar = tdc_gain * (rolldot - w_nr.x) * aaLim;
 
			//Total Roll Moment Command
			mom_c = mom_c1 + mom_icar;

			double rollFinCommandRads;

			if(sys->t_flight >= t0)
			{

				//Coefficients for third order equation for Roll Autopilot
				//Pole Placement
				A_ap    = xA_table->interp( q );
				wn_ap   = xwn_table->interp( q );
				zeta_ap = xzeta_table->interp( q );

				c0 = A_ap * wn_ap*wn_ap;
				c1 = wn_ap*wn_ap + 2.0*A_ap*zeta_ap*wn_ap;
				c2 = A_ap + 2.0*zeta_ap*wn_ap;
				tn = 0.0/A_ap;

				double kn_r = c0;
				double kp_r = c2;
				double ka_r = c1 - tn * c0;

				double xki_r = kn_r;                    //K1
				double xka_r = ka_r;                    //B
				double xkg_r = kp_r * kpr_gain; //K2

				rollFinCommandRads = mom_c / ldelta_est;
				if (abs(rollFinCommandRads) > (delr_cmd_lim * dtr))
				{
					if (rollFinCommandRads > 0.0)
					{
						rollFinCommandRads = delr_cmd_lim * dtr;
					}
					else if (rollFinCommandRads < 0.0)
					{
						rollFinCommandRads = -1.0 * delr_cmd_lim * dtr;
					}
				}

				double mcxlmtot = rollFinCommandRads * ldelta_est;
				double mcxlm    = mcxlmtot;

				//Limited moment
				double mom_c_lm = mcxlm - mom_icar;

				//Roll (Reset Roll Autopilot Integrator)
				double xxx = 1.0/xki_r * (mom_c_lm + xka_r*phi_hat + xkg_r*w_nr.x);

				phi1 = xxx;
			
			} //rollauto->t0

			rollFinCommandDegrees = rollFinCommandRads * rtd;

		} // State::t > t0
	} // executing
}

#ifdef SIXDOF 
void RollAuto::rpt()
{
	if( State::sample(out->pt_console))
	{
		if(out->displayInput == 1)
		{
			printf( "RollAuto  %10.4f\n", sys->t_flight);
		}
	}
}
#endif

