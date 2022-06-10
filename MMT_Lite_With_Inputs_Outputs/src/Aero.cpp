//-----------------------------------------------------------//
// File: AeroC.cpp
// 
// This class computes the aerodynamic forces and moments
// exerted on the missile during flight using Complex aero.
//                                                           
// Developer: Dennis Strickland                              
//-----------------------------------------------------------//

#include "Aero.h"
#include "System.h"
#include "Output.h"

#include "stdio.h"
#include <stdlib.h>

Aero::Aero(string infile, Output *outp, System *sysp) {

	//Local access
	out   = outp;
	sys   = sysp;

	//Complex Aero
	string aeroFileC = "./input/"+infile;

	//Input
	Filer *ff = new Filer( "./input/"+infile);
	aero_flag = ff->getInt("aero_Flag");
	errAero_Flag = ff->getInt( "errAero_Flag");
	ca_sig       = ff->getDouble( "ca_sig");
	cn_sig       = ff->getDouble( "cn_sig");
	cy_sig       = ff->getDouble( "cy_sig");
	cm_sig       = ff->getDouble( "cm_sig");
	cln_sig      = ff->getDouble( "cln_sig");
	cll_sig      = ff->getDouble( "cll_sig");
	cmq_sig      = ff->getDouble( "cmq_sig");
	clp_sig      = ff->getDouble( "clp_sig");
	xcp_sig      = ff->getDouble( "xcp_sig");
	delete ff;

	if(aero_flag == 1)
	{

		cout << "COMPLEX AERODYNAMICS CONSTRUCTED" << endl;

		//Instantiate Aero Data
		caon_table  = new Table3( aeroFileC.c_str());
		caoff_table = new Table3( aeroFileC.c_str());
		cll_table   = new Table3( aeroFileC.c_str());
		cln_table   = new Table3( aeroFileC.c_str());
		cm_table    = new Table3( aeroFileC.c_str());
		cn_table    = new Table3( aeroFileC.c_str());
		cy_table    = new Table3( aeroFileC.c_str());
		//
		cllp_table  = new Table2( aeroFileC.c_str());
		cmqC_table  = new Table2( aeroFileC.c_str());

		dp_inc_ca_table  = new Table4( aeroFileC.c_str());
		dp_inc_cll_table = new Table4( aeroFileC.c_str());
		dp_inc_cln_table = new Table4( aeroFileC.c_str());
		dp_inc_cm_table  = new Table4( aeroFileC.c_str());
		dp_inc_cn_table  = new Table4( aeroFileC.c_str());
		dp_inc_cy_table  = new Table4( aeroFileC.c_str());

		dr_inc_ca_table  = new Table4( aeroFileC.c_str());
		dr_inc_cll_table = new Table4( aeroFileC.c_str());
		dr_inc_cln_table = new Table4( aeroFileC.c_str());
		dr_inc_cm_table  = new Table4( aeroFileC.c_str());
		dr_inc_cn_table  = new Table4( aeroFileC.c_str());
		dr_inc_cy_table  = new Table4( aeroFileC.c_str());

		dy_inc_ca_table  = new Table4( aeroFileC.c_str());
		dy_inc_cll_table = new Table4( aeroFileC.c_str());
		dy_inc_cln_table = new Table4( aeroFileC.c_str());
		dy_inc_cm_table  = new Table4( aeroFileC.c_str());
		dy_inc_cn_table  = new Table4( aeroFileC.c_str());
		dy_inc_cy_table  = new Table4( aeroFileC.c_str());

		dpdr_inc_ca_table  = new Table5( aeroFileC.c_str());
		dpdr_inc_cll_table = new Table5( aeroFileC.c_str());
		dpdr_inc_cln_table = new Table5( aeroFileC.c_str());
		dpdr_inc_cm_table  = new Table5( aeroFileC.c_str());
		dpdr_inc_cn_table  = new Table5( aeroFileC.c_str());
		dpdr_inc_cy_table  = new Table5( aeroFileC.c_str());

		dpdy_inc_ca_table  = new Table5( aeroFileC.c_str());
		dpdy_inc_cll_table = new Table5( aeroFileC.c_str());
		dpdy_inc_cln_table = new Table5( aeroFileC.c_str());
		dpdy_inc_cm_table  = new Table5( aeroFileC.c_str());
		dpdy_inc_cn_table  = new Table5( aeroFileC.c_str());
		dpdy_inc_cy_table  = new Table5( aeroFileC.c_str());

		dydr_inc_ca_table  = new Table5( aeroFileC.c_str());
		dydr_inc_cll_table = new Table5( aeroFileC.c_str());
		dydr_inc_cln_table = new Table5( aeroFileC.c_str());
		dydr_inc_cm_table  = new Table5( aeroFileC.c_str());
		dydr_inc_cn_table  = new Table5( aeroFileC.c_str());
		dydr_inc_cy_table  = new Table5( aeroFileC.c_str());

		//Read Aero Data
		caon_table->read( "CAon_mach_alpha_phi",true);
		caoff_table->read( "CAoff_mach_alpha_phi",true);
		cll_table->read( "CLL_mach_alpha_phi",true);
		cln_table->read( "CLN_mach_alpha_phi",true);
		cm_table->read( "CM_mach_alpha_phi",true);
		cn_table->read( "CN_mach_alpha_phi",true);
		cy_table->read( "CY_mach_alpha_phi",true);
		//
		cllp_table->read( "CLLP_mach_alpha",true);
		cmqC_table->read( "CMQ_mach_alpha",true);

		dp_inc_ca_table->read( "dP_INC_CA_delp_mach_alpha_phi",false);
		dp_inc_cll_table->read( "dP_INC_CLL_delp_mach_alpha_phi",false);
		dp_inc_cln_table->read( "dP_INC_CLN_delp_mach_alpha_phi",false);
		dp_inc_cm_table->read( "dP_INC_CM_delp_mach_alpha_phi",false);
		dp_inc_cn_table->read( "dP_INC_CN_delp_mach_alpha_phi",false);
		dp_inc_cy_table->read( "dP_INC_CY_delp_mach_alpha_phi",false);

		dr_inc_ca_table->read( "dR_INC_CA_delr_mach_alpha_phi",false);
		dr_inc_cll_table->read( "dR_INC_CLL_delr_mach_alpha_phi",false);
		dr_inc_cln_table->read( "dR_INC_CLN_delr_mach_alpha_phi",false);
		dr_inc_cm_table->read( "dR_INC_CM_delr_mach_alpha_phi",false);
		dr_inc_cn_table->read( "dR_INC_CN_delr_mach_alpha_phi",false);
		dr_inc_cy_table->read( "dR_INC_CY_delr_mach_alpha_phi",false);

		dy_inc_ca_table->read( "dY_INC_CA_dely_mach_alpha_phi",false);
		dy_inc_cll_table->read( "dY_INC_CLL_dely_mach_alpha_phi",false);
		dy_inc_cln_table->read( "dY_INC_CLN_dely_mach_alpha_phi",false);
		dy_inc_cm_table->read( "dY_INC_CM_dely_mach_alpha_phi",false);
		dy_inc_cn_table->read( "dY_INC_CN_dely_mach_alpha_phi",false);
		dy_inc_cy_table->read( "dY_INC_CY_dely_mach_alpha_phi",false);

		dpdr_inc_ca_table->read( "dPdR_INC_CA_delp_delr_mach_alpha_phi",false);
		dpdr_inc_cll_table->read( "dPdR_INC_CLL_delp_delr_mach_alpha_phi",false);
		dpdr_inc_cln_table->read( "dPdR_INC_CLN_delp_delr_mach_alpha_phi",false);
		dpdr_inc_cm_table->read( "dPdR_INC_CM_delp_delr_mach_alpha_phi",false);
		dpdr_inc_cn_table->read( "dPdR_INC_CN_delp_delr_mach_alpha_phi",false);
		dpdr_inc_cy_table->read( "dPdR_INC_CY_delp_delr_mach_alpha_phi",false);

		dpdy_inc_ca_table->read( "dPdY_INC_CA_delp_dely_mach_alpha_phi",false);
		dpdy_inc_cll_table->read( "dPdY_INC_CLL_delp_dely_mach_alpha_phi",false);
		dpdy_inc_cln_table->read( "dPdY_INC_CLN_delp_dely_mach_alpha_phi",false);
		dpdy_inc_cm_table->read( "dPdY_INC_CM_delp_dely_mach_alpha_phi",false);
		dpdy_inc_cn_table->read( "dPdY_INC_CN_delp_dely_mach_alpha_phi",false);
		dpdy_inc_cy_table->read( "dPdY_INC_CY_delp_dely_mach_alpha_phi",false);

		dydr_inc_ca_table->read( "dYdR_INC_CA_dely_delr_mach_alpha_phi",false);
		dydr_inc_cll_table->read( "dYdR_INC_CLL_dely_delr_mach_alpha_phi",false);
		dydr_inc_cln_table->read( "dYdR_INC_CLN_dely_delr_mach_alpha_phi",false);
		dydr_inc_cm_table->read( "dYdR_INC_CM_dely_delr_mach_alpha_phi",false);
		dydr_inc_cn_table->read( "dYdR_INC_CN_dely_delr_mach_alpha_phi",false);
		dydr_inc_cy_table->read( "dYdR_INC_CY_dely_delr_mach_alpha_phi",false);
	}
	else if (aero_flag == -1)
	{

		cout << "BASIC AERODYNAMICS CONSTRUCTED" << endl;

		// INIT AERO DATA
		caon_table  = new Table3( aeroFileC.c_str());
		caoff_table = new Table3( aeroFileC.c_str());
		cll_table   = new Table3( aeroFileC.c_str());
		cln_table   = new Table3( aeroFileC.c_str());
		cm_table    = new Table3( aeroFileC.c_str());
		cn_table    = new Table3( aeroFileC.c_str());
		cy_table    = new Table3( aeroFileC.c_str());
		cllp_table  = new Table2( aeroFileC.c_str());
		cmqC_table  = new Table2( aeroFileC.c_str());

		dp_inc_ca_table  = new Table4( aeroFileC.c_str());
		dp_inc_cll_table = new Table4( aeroFileC.c_str());
		dp_inc_cln_table = new Table4( aeroFileC.c_str());
		dp_inc_cm_table  = new Table4( aeroFileC.c_str());
		dp_inc_cn_table  = new Table4( aeroFileC.c_str());
		dp_inc_cy_table  = new Table4( aeroFileC.c_str());

		dr_inc_ca_table  = new Table4( aeroFileC.c_str());
		dr_inc_cll_table = new Table4( aeroFileC.c_str());
		dr_inc_cln_table = new Table4( aeroFileC.c_str());
		dr_inc_cm_table  = new Table4( aeroFileC.c_str());
		dr_inc_cn_table  = new Table4( aeroFileC.c_str());
		dr_inc_cy_table  = new Table4( aeroFileC.c_str());

		dy_inc_ca_table  = new Table4( aeroFileC.c_str());
		dy_inc_cll_table = new Table4( aeroFileC.c_str());
		dy_inc_cln_table = new Table4( aeroFileC.c_str());
		dy_inc_cm_table  = new Table4( aeroFileC.c_str());
		dy_inc_cn_table  = new Table4( aeroFileC.c_str());
		dy_inc_cy_table  = new Table4( aeroFileC.c_str());

		// READ AERO DATA
		caon_table->read( "CAon_mach_alpha_phi",true);
		caoff_table->read( "CAoff_mach_alpha_phi",true);
		cll_table->read( "CLL_mach_alpha_phi",true);
		cln_table->read( "CLN_mach_alpha_phi",true);
		cm_table->read( "CM_mach_alpha_phi",true);
		cn_table->read( "CN_mach_alpha_phi",true);
		cy_table->read( "CY_mach_alpha_phi",true);
		cllp_table->read( "CLLP_mach_alpha",true);
		cmqC_table->read( "CMQ_mach_alpha",true);

		dp_inc_ca_table->read( "dP_INC_CA_delp_mach_alpha_phi",false);
		dp_inc_cll_table->read( "dP_INC_CLL_delp_mach_alpha_phi",false);
		dp_inc_cln_table->read( "dP_INC_CLN_delp_mach_alpha_phi",false);
		dp_inc_cm_table->read( "dP_INC_CM_delp_mach_alpha_phi",false);
		dp_inc_cn_table->read( "dP_INC_CN_delp_mach_alpha_phi",false);
		dp_inc_cy_table->read( "dP_INC_CY_delp_mach_alpha_phi",false);

		dr_inc_ca_table->read( "dR_INC_CA_delr_mach_alpha_phi",false);
		dr_inc_cll_table->read( "dR_INC_CLL_delr_mach_alpha_phi",false);
		dr_inc_cln_table->read( "dR_INC_CLN_delr_mach_alpha_phi",false);
		dr_inc_cm_table->read( "dR_INC_CM_delr_mach_alpha_phi",false);
		dr_inc_cn_table->read( "dR_INC_CN_delr_mach_alpha_phi",false);
		dr_inc_cy_table->read( "dR_INC_CY_delr_mach_alpha_phi",false);

		dy_inc_ca_table->read( "dY_INC_CA_dely_mach_alpha_phi",false);
		dy_inc_cll_table->read( "dY_INC_CLL_dely_mach_alpha_phi",false);
		dy_inc_cln_table->read( "dY_INC_CLN_dely_mach_alpha_phi",false);
		dy_inc_cm_table->read( "dY_INC_CM_dely_mach_alpha_phi",false);
		dy_inc_cn_table->read( "dY_INC_CN_dely_mach_alpha_phi",false);
		dy_inc_cy_table->read( "dY_INC_CY_dely_mach_alpha_phi",false);

	}
}

void Aero::init() { 

	tick = 0.0;
	kt = 0;

	force    = Vec( 0.0, 0.0, 0.0);
	moment   = Vec( 0.0, 0.0, 0.0);

	defl1 = defl2 = defl3 = defl4 = 0.0;
	delr = delp = dely = 0.0;
	xmrp = 9.903;

	bool disable = true;
	if(errAero_Flag == 1)
	{
		disable = false;
	}

	ca_Err    = 0.0;
	cn_Err      = 0.0;
	cy_Err     = 0.0;
	cln_Err     = 0.0;
	cm_Err      = 0.0;
	cll_Err      = 0.0;
	cmq_Err     = 0.0;
	clp_Err       = 0.0;
	xcp_Err      = 0.0;

	firstFlag = -1;

}

void Aero::update(
	double finDeflOne,
	double finDeflTwo,
	double finDeflThree,
	double finDeflFour,
	double centerOfGravity,
	double thrust,
	double refDiam,
	double refArea,
	double dynamicPressure,
	double inputMach,
	double totalAngleOfAttack,
	double phiPrime,
	double speed,
	double nonRolledRollRate,
	double nonRolledPitchRate
)
{
	if(State::sample())
	{
		tick = ( double)kt++;
	}

	if ( State::sample(State::EVENT, 0.0))
	{

	}

	q = dynamicPressure;
	mach = inputMach;
	totalAngleOfAttackDegrees = totalAngleOfAttack;
	phiPrimeDegrees = phiPrime;

	// Capture deflections locally for output
	defl1 = finDeflOne;
	defl2 = finDeflTwo;
	defl3 = finDeflThree;
	defl4 = finDeflFour;

	//Fin Mixing
	delr = (  defl1 + defl2 + defl3 + defl4) / 4.0;
	delp = ( -defl1 - defl2 + defl3 + defl4) / 4.0;
	dely = (  defl1 - defl2 - defl3 + defl4) / 4.0;

	//Calculate Coefficients
	getAeroCoeffs();

	// AERO DYNAMIC COEFFICIENTS

	// COMPLEX AERO
	if (aero_flag == 1)
	{

	
		// CA ON
		caon_stb = STABILITY_CAF;
		caon_d   = dP_INC_CA + dY_INC_CA + dR_INC_CA + dPdY_INC_CA + dPdR_INC_CA + dYdR_INC_CA;
		CA_PON_C = (caon_stb + caon_d) * (1.0+ca_Err);

		// CA OFF
		caoff_stb = STABILITY_CAU;
		caoff_d   = dP_INC_CA + dY_INC_CA + dR_INC_CA + dPdY_INC_CA + dPdR_INC_CA + dYdR_INC_CA;
		CA_POFF_C = (caoff_stb + caoff_d) * (1.0+ca_Err);

		//CA
		if(thrust > 0.0)
		{
			CA_C = CA_PON_C;
		}
		else
		{
			CA_C = CA_POFF_C;
		}

		// RATE ESTIMATES
		double vtol = 0.1;
		if (speed > vtol)
		{
			phat = (nonRolledRollRate * refDiam)/(2.0 * speed) * rtd;
			qhat = (nonRolledPitchRate * refDiam)/(2.0 * speed) * rtd;
		}
		else
		{
			phat = 0.0;
			qhat = 0.0;
		}

		// CN
		cn_stb = STABILITY_CN;
		cn_d   = (dP_INC_CN + dY_INC_CN + dR_INC_CN + dPdY_INC_CN + dPdR_INC_CN + dYdR_INC_CN);
		CN_C   = (cn_stb + cn_d) * (1.0+cn_Err);

		//CM
		xcal = centerOfGravity / refDiam;
		cm_stb = STABILITY_CM + STABILITY_CN*xcal - STABILITY_CN*xcp_Err;
		cm_d   = (dP_INC_CM + dY_INC_CM + dR_INC_CM + dPdY_INC_CM + dPdR_INC_CM + dYdR_INC_CM) + ( cn_d )*xcal;
		CMcg_C  = ((cm_stb + cm_d) * (1.0+cm_Err)) + (DAMP_CMQnr*qhat*cos(phiPrimeDegrees*dtr) * (1.0+cmq_Err));

		//CY
		cy_stb = STABILITY_CY;
		cy_d   = (dP_INC_CY + dY_INC_CY + dR_INC_CY + dPdY_INC_CY + dPdR_INC_CY + dYdR_INC_CY);
		CY_C   = (cy_stb + cy_d) * (1.0+cy_Err);

		//CLN
		cln_stb = (STABILITY_CLN + STABILITY_CY*xcal) - STABILITY_CY*xcp_Err;
		cln_d   = (dP_INC_CLN + dY_INC_CLN + dR_INC_CLN + dPdY_INC_CLN + dPdR_INC_CLN + dYdR_INC_CLN) + ( cy_d )*xcal;
		CLNcg_C = ((cln_stb + cln_d) * (1.0+cln_Err)) + (DAMP_CMQnr*qhat*sin(phiPrimeDegrees*dtr) * (1.0+cmq_Err));

		//CLL
		cll_stb = STABILITY_CLL;
		cll_d   = (dP_INC_CLL + dY_INC_CLL + dR_INC_CLL + dPdY_INC_CLL + dPdR_INC_CLL + dYdR_INC_CLL);
		CLL_C = ((cll_stb + cll_d) * (1.0+cll_Err)) + (DAMP_CLLP*phat * (1.0+clp_Err)); 

		force.x  = -CA_C * q * refArea;
		force.y  =  CY_C * q * refArea;
		force.z  = -CN_C * q * refArea;

		moment.x =  CLL_C   * q * refArea * refDiam;
		moment.y =  CMcg_C  * q * refArea * refDiam;
		moment.z =  CLNcg_C * q * refArea * refDiam;

	}

	else if (aero_flag == -1)
	{
		// CA ON
		caon_stb = STABILITY_CAF;
		caon_d   = dP_INC_CA + dY_INC_CA + dR_INC_CA;
		CA_PON_C = caon_stb + caon_d;

		// CA OFF
		caoff_stb = STABILITY_CAU;
		caoff_d   = dP_INC_CA + dY_INC_CA + dR_INC_CA;
		CA_POFF_C = caoff_stb + caoff_d;

		//CA
		if(thrust > 0.0)
		{
			CA_C = CA_PON_C;
		}
		else
		{
			CA_C = CA_POFF_C;
		}

		// RATE ESTIMATES
		double vtol = 0.1;
		if (speed > vtol)
		{
			phat = (nonRolledRollRate * refDiam)/(2.0 * speed) * rtd;
			qhat = (nonRolledPitchRate * refDiam)/(2.0 * speed) * rtd;
		}
		else
		{
			phat = 0.0;
			qhat = 0.0;
		}

		// CN
		cn_stb = STABILITY_CN;
		cn_d   = dP_INC_CN + dY_INC_CN + dR_INC_CN;
		CN_C   = cn_stb + cn_d;

		//CM
		xcal = centerOfGravity / refDiam;
		cm_stb = STABILITY_CM + STABILITY_CN*xcal - STABILITY_CN*xcp_Err;
		cm_d   = dP_INC_CM + dY_INC_CM + dR_INC_CM ;
		CMcg_C = cm_stb + cm_d + cn_d * xcal + DAMP_CMQnr * qhat * cos(phiPrimeDegrees * dtr);

		//CY
		cy_stb = STABILITY_CY;
		cy_d   = dP_INC_CY + dY_INC_CY + dR_INC_CY;
		CY_C = cy_stb + cy_d;

		//CLN
		cln_stb = (STABILITY_CLN + STABILITY_CY*xcal) - STABILITY_CY*xcp_Err;
		cln_d   = dP_INC_CLN + dY_INC_CLN + dR_INC_CLN;
		CLNcg_C = cln_stb + cln_d + cy_d * xcal + DAMP_CMQnr * qhat * sin(phiPrimeDegrees * dtr);

		//CLL
		cll_stb = STABILITY_CLL;
		cll_d   = dP_INC_CLL + dY_INC_CLL + dR_INC_CLL;
		CLL_C = cll_stb + cll_d + DAMP_CLLP * phat;

		force.x  = -CA_C * q * refArea;
		force.y  =  CY_C * q * refArea;
		force.z  = -CN_C * q * refArea;

		moment.x =  CLL_C   * q * refArea * refDiam;
		moment.y =  CMcg_C  * q * refArea * refDiam;
		moment.z =  CLNcg_C * q * refArea * refDiam;

	}

}

//----------------------------------------------//

void Aero::rpt() {

	if (State::sample( out->pt_console))
	{

		if (out->displayInput == 1)
		{
			printf( "Aero  %10.4f\n", sys->t_sys);
		}

	}

}

//-------------------------------------------------//

void Aero::getAeroCoeffs()
{

	if (aero_flag == 1)
	{

		//Interpolate Aero Data
		STABILITY_CAF = caon_table->interp(  mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		STABILITY_CAU = caoff_table->interp( mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		STABILITY_CLL = cll_table->interp(   mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		STABILITY_CLN = cln_table->interp(   mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		STABILITY_CM  = cm_table->interp(    mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		STABILITY_CN  = cn_table->interp(    mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		STABILITY_CY  = cy_table->interp(    mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		//
		DAMP_CLLP     = cllp_table->interp(  totalAngleOfAttackDegrees, mach);
		DAMP_CMQnr    = cmqC_table->interp(  totalAngleOfAttackDegrees, mach);

		dP_INC_CA  = dp_inc_ca_table->interp(  delp, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dP_INC_CLL = dp_inc_cll_table->interp( delp, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dP_INC_CLN = dp_inc_cln_table->interp( delp, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dP_INC_CM  = dp_inc_cm_table->interp(  delp, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dP_INC_CN  = dp_inc_cn_table->interp(  delp, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dP_INC_CY  = dp_inc_cy_table->interp(  delp, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);

		dR_INC_CA  = dr_inc_ca_table->interp(  delr, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dR_INC_CLL = dr_inc_cll_table->interp( delr, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dR_INC_CLN = dr_inc_cln_table->interp( delr, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dR_INC_CM  = dr_inc_cm_table->interp(  delr, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dR_INC_CN  = dr_inc_cn_table->interp(  delr, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dR_INC_CY  = dr_inc_cy_table->interp(  delr, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);

		dY_INC_CA  = dy_inc_ca_table->interp(  dely, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dY_INC_CLL = dy_inc_cll_table->interp( dely, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dY_INC_CLN = dy_inc_cln_table->interp( dely, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dY_INC_CM  = dy_inc_cm_table->interp(  dely, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dY_INC_CN  = dy_inc_cn_table->interp(  dely, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dY_INC_CY  = dy_inc_cy_table->interp(  dely, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);

		dPdR_INC_CA  = dpdr_inc_ca_table->interp(  delp, delr, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dPdR_INC_CLL = dpdr_inc_cll_table->interp( delp, delr, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dPdR_INC_CLN = dpdr_inc_cln_table->interp( delp, delr, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dPdR_INC_CM  = dpdr_inc_cm_table->interp(  delp, delr, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dPdR_INC_CN  = dpdr_inc_cn_table->interp(  delp, delr, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dPdR_INC_CY  = dpdr_inc_cy_table->interp(  delp, delr, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);

		dPdY_INC_CA  = dpdy_inc_ca_table->interp(  delp, dely, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dPdY_INC_CLL = dpdy_inc_cll_table->interp( delp, dely, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dPdY_INC_CLN = dpdy_inc_cln_table->interp( delp, dely, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dPdY_INC_CM  = dpdy_inc_cm_table->interp(  delp, dely, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dPdY_INC_CN  = dpdy_inc_cn_table->interp(  delp, dely, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dPdY_INC_CY  = dpdy_inc_cy_table->interp(  delp, dely, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);

		dYdR_INC_CA  = dydr_inc_ca_table->interp(  dely, delr, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dYdR_INC_CLL = dydr_inc_cll_table->interp( dely, delr, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dYdR_INC_CLN = dydr_inc_cln_table->interp( dely, delr, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dYdR_INC_CM  = dydr_inc_cm_table->interp(  dely, delr, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dYdR_INC_CN  = dydr_inc_cn_table->interp(  dely, delr, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dYdR_INC_CY  = dydr_inc_cy_table->interp(  dely, delr, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);

	}

	else if (aero_flag == -1)
	{

		//Interpolate Aero Data
		STABILITY_CAF = caon_table->interp(  mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		STABILITY_CAU = caoff_table->interp( mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		STABILITY_CLL = cll_table->interp(   mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		STABILITY_CLN = cln_table->interp(   mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		STABILITY_CM  = cm_table->interp(    mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		STABILITY_CN  = cn_table->interp(    mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		STABILITY_CY  = cy_table->interp(    mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		DAMP_CLLP     = cllp_table->interp(  totalAngleOfAttackDegrees, mach);
		DAMP_CMQnr    = cmqC_table->interp(  totalAngleOfAttackDegrees, mach);

		dP_INC_CA  = dp_inc_ca_table->interp(  delp, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dP_INC_CLL = dp_inc_cll_table->interp( delp, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dP_INC_CLN = dp_inc_cln_table->interp( delp, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dP_INC_CM  = dp_inc_cm_table->interp(  delp, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dP_INC_CN  = dp_inc_cn_table->interp(  delp, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dP_INC_CY  = dp_inc_cy_table->interp(  delp, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);

		dR_INC_CA  = dr_inc_ca_table->interp(  delr, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dR_INC_CLL = dr_inc_cll_table->interp( delr, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dR_INC_CLN = dr_inc_cln_table->interp( delr, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dR_INC_CM  = dr_inc_cm_table->interp(  delr, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dR_INC_CN  = dr_inc_cn_table->interp(  delr, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dR_INC_CY  = dr_inc_cy_table->interp(  delr, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);

		dY_INC_CA  = dy_inc_ca_table->interp(  dely, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dY_INC_CLL = dy_inc_cll_table->interp( dely, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dY_INC_CLN = dy_inc_cln_table->interp( dely, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dY_INC_CM  = dy_inc_cm_table->interp(  dely, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dY_INC_CN  = dy_inc_cn_table->interp(  dely, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);
		dY_INC_CY  = dy_inc_cy_table->interp(  dely, mach, totalAngleOfAttackDegrees, phiPrimeDegrees);

	}

}

//-------------------------------------------------//

void Aero::writeAero()
{
	stringstream sstm;

	sstm << setprecision(6) << fixed;

	if(firstFlag == -1)
	{
		sstm << "time  amach  alph  aphi  delr  delp  dely  STABILITY_CAF  STABILITY_CAU  STABILITY_CLL  STABILITY_CLN  STABILITY_CM  STABILITY_CN  STABILITY_CY  DAMP_CLLP  DAMP_CMQnr  dP_INC_CA  dP_INC_CLL  dP_INC_CLN  dP_INC_CM  dP_INC_CN  dP_INC_CY  dR_INC_CA  dR_INC_CLL  dR_INC_CLN  dR_INC_CM  dR_INC_CN  dR_INC_CY  dY_INC_CA  dY_INC_CLL  dY_INC_CLN  dY_INC_CM  dY_INC_CN  dY_INC_CY  dPdR_INC_CA  dPdR_INC_CLL  dPdR_INC_CLN  dPdR_INC_CM  dPdR_INC_CN  dPdR_INC_CY  dPdY_INC_CA  dPdY_INC_CLL  dPdY_INC_CLN  dPdY_INC_CM  dPdY_INC_CN  dPdY_INC_CY  dYdR_INC_CA  dYdR_INC_CLL  dYdR_INC_CLN  dYdR_INC_CM  dYdR_INC_CN  dYdR_INC_CY  Force.x  Force.y  Force.z  Moment.x  Moment.y  Moment.z" << endl;
		outAero << sstm.str();
		firstFlag = 1;
	}

	sstm << sys->t_sys << "   "
	<< mach << "   "
	<< totalAngleOfAttackDegrees  << "   "
	<< phiPrimeDegrees  << "   "
	<< delr       << "   "
	<< delp       << "   "
	<< dely       << "   "
	<< STABILITY_CAF  << "   "
	<< STABILITY_CAU  << "   "
	<< STABILITY_CLL  << "   "
	<< STABILITY_CLN  << "   "
	<< STABILITY_CM   << "   "
	<< STABILITY_CN   << "   "
	<< STABILITY_CY   << "   "
	<< DAMP_CLLP      << "   "
	<< DAMP_CMQnr     << "   "
	<< dP_INC_CA      << "   "
	<< dP_INC_CLL     << "   "
	<< dP_INC_CLN     << "   "
	<< dP_INC_CM      << "   "
	<< dP_INC_CN      << "   "
	<< dP_INC_CY      << "   "
	<< dR_INC_CA      << "   "
	<< dR_INC_CLL     << "   "
	<< dR_INC_CLN     << "   "
	<< dR_INC_CM      << "   "
	<< dR_INC_CN      << "   "
	<< dR_INC_CY      << "   "
	<< dY_INC_CA      << "   "
	<< dY_INC_CLL     << "   "
	<< dY_INC_CLN     << "   "
	<< dY_INC_CM      << "   "
	<< dY_INC_CN      << "   "
	<< dY_INC_CY      << "   "
	<< dPdR_INC_CA    << "   "
	<< dPdR_INC_CLL   << "   "
	<< dPdR_INC_CLN   << "   "
	<< dPdR_INC_CM    << "   "
	<< dPdR_INC_CN    << "   "
	<< dPdR_INC_CY    << "   "
	<< dPdY_INC_CA    << "   "
	<< dPdY_INC_CLL   << "   "
	<< dPdY_INC_CLN   << "   "
	<< dPdY_INC_CM    << "   "
	<< dPdY_INC_CN    << "   "
	<< dPdY_INC_CY    << "   "
	<< dYdR_INC_CA    << "   "
	<< dYdR_INC_CLL   << "   "
	<< dYdR_INC_CLN   << "   "
	<< dYdR_INC_CM    << "   "
	<< dYdR_INC_CN    << "   "
	<< dYdR_INC_CY    << "   "
	<< force.x        << "   " 
	<< force.y        << "   " 
	<< force.z        << "   " 
	<< moment.x       << "   " 
	<< moment.y       << "   " 
	<< moment.z       << endl;

	if((sys->t_sys <= 20.93) && (sys->t_sys >= 0.0))
	{
		outAero << sstm.str();
	}
	if (sys->t_sys > 20.93)
	{
		outAero.close();
	}

}
