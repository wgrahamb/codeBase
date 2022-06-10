#include "reFactoredGeometry.h"


reFactoredGeometry::reFactoredGeometry()
{

}

void reFactoredGeometry::init( double phi0, double tht0, double psi0, double lat0, double lon0, double alt0, double targetAz)
{

	phiLnch_Err = 0.0;
	thtLnch_Err = 0.0;
	psiLnch_Err = 0.0;

	//Launch Angle with uncertainties
	phiLnch = (phi0 + phiLnch_Err) * dtr;
	thtLnch = (tht0 + thtLnch_Err) * dtr;
	psiLnch = (psi0 + psiLnch_Err) * dtr;

	latLoc_Err = 0.0;
	lonLoc_Err = 0.0;
	altLoc_Err = 0.0;
	
	//Launch Location with uncertainties
	latg  = (lat0 + latLoc_Err) * dtr;
	lon   = (lon0 + lonLoc_Err) * dtr;
	altg  = alt0 + altLoc_Err;

	//Initial ECEFtoNEDg DCM (Launch Site)
	ECEFtoNEDg = Vec(0.0, -1 * (PI / 2.0 + lat0 * dtr), lon0 * dtr).getDCM();
	ECEFtoNEDb = ECEFtoNEDg; 

	//Initial NED to Launch-Tangent-Frame DCM
	NEDtoLTF = Vec(0.0, 0.0, targetAz * dtr).getDCM();

	//Initial LTFtoBODY, NEDgtoBODY and NEDbtoBODY
	LTFtoBODY  = Vec(phiLnch, thtLnch, psiLnch).getDCM();
	//
	NEDgtoBODY = LTFtoBODY * NEDtoLTF;
	//
	NEDbtoBODY = NEDgtoBODY;

	//Compute ECEFtoBODY DCM
	ECEFtoBODY = NEDgtoBODY * ECEFtoNEDg;
	
	//Initial ECI-To-ECEF DCM
	update_ECI2ECEF( State::t, ECItoECEF);

	//Compute ECItoBODY DCM
	ECItoBODY = ECEFtoBODY * ECItoECEF;
}

void reFactoredGeometry::update(
	double totalSystemTime,
	Quat eulerQuaternion
)
{

	//Update ECI-To-ECEF DCM
	update_ECI2ECEF(totalSystemTime, ECItoECEF);

	//Update ECItoBody DCM
	ECItoBODY = eulerQuaternion.getDCM(); 

	//Update ECEFtoNEDg (body referenced NED) DCMs
	ECEFtoNEDb = Vec(0.0, -(PI/2.0+latg), lon).getDCM();

	//Compute ECEFtoBODY DCM
	ECEFtoBODY = ECItoBODY * ECItoECEF.transpose();

	//Compute NEDgtoBODY DCM
	NEDgtoBODY = ECEFtoBODY * ECEFtoNEDg.transpose();

	//Compute LTFtoBODY DCM
	LTFtoBODY = NEDgtoBODY * NEDtoLTF.transpose();

	//Compute NEDbtoBODY DCM
	NEDbtoBODY = ECEFtoBODY * ECEFtoNEDb.transpose();

}

void reFactoredGeometry::update_ECI2ECEF(double time, Mat &i2e)
{
		//Update the ECI2ECEF direction cosine matrix
		double phi = (time)*omegae;   //Rotation angle from ECI to ECEF
		i2e[0][0] = cos(phi);
		i2e[0][1] = sin(phi);
		i2e[0][2] = 0.0;
		i2e[1][0] = -sin(phi);
		i2e[1][1] = cos(phi);
		i2e[1][2] = 0.0;
		i2e[2][0] = 0.0;
		i2e[2][1] = 0.0;
		i2e[2][2] = 1.0;
}

Vec reFactoredGeometry::gravity(Vec epos)
{
	//---------------------------------------------------------//
	//This routine uses the WGS84 Gravity Model with J2,J3,J4
	//harmonics.  The output is a gravity vector in ECEF.
	//
	// Input: xecef - missile position in ECEF coord - x (m)
	//        yecef - missile position in ECEF coord - y (m)
	//        zecef - missile position in ECEF coord - z (m)
	//
	//Output: 
	//        gravx - acceleration of gravity in ECEF coord - x (m/s^2)
	//        gravy - acceleration of gravity in ECEF coord - y (m/s^2)
	//        gravz - acceleration of gravity in ECEF coord - z (m/s^2)
	//
	//  Note:  WGS84 Gravity Model with J2,J3,J4 harmonics
	//
	//  Developer:  Dennis Strickland
	//
	//---------------------------------------------------------//

	//Calculate Geocentric Latitude, Longitude and radius
	lon   = atan2(epos.y, epos.x);
	latc  = atan(epos.z/(sqrt(epos.x*epos.x + epos.y*epos.y)));
	Brm   = epos.mag();

	//Calculate Altitude estimate and Geodetic Latitude based on Berger Technique
	elipfc   = (1.0/b_over_a)*(1.0/b_over_a) - 1.0;
	eccen    = sqrt(1.0 - b_over_a*b_over_a);
	e2       = eccen*eccen;
	e4       = e2*e2;
	eps_num  = e2*sin(2.0*latc) + e4*sin(latc)*sin(latc)*sin(latc)*cos(latc);
	eps_fac1 = Brm / a_semi;
	eps_fac2 = e2*cos(2.0*latc);
	eps_fac3 = 0.25*e4*(1.0 + (1.0 - 2.0*cos(2.0*latc))*cos(2.0*latc));
	epsilon  = eps_num / (2.0*(eps_fac1 - eps_fac2 - eps_fac3));
	latg     = epsilon + latc;   //Geodetic Latitude
	phi1pm   = atan(b_over_a*b_over_a*tan(latg));
	rm       = a_semi / sqrt(1.0 + elipfc*sin(phi1pm)*sin(phi1pm));
	
	//Calculate Gravity at the missile (NEDc)  (Britting model)
	Gr1 = (3.0/2.0)*J2*((a_semi/Brm)*(a_semi/Brm))*(3.0*cos(PI/2.0 - latc)*cos(PI/2.0 - latc) - 1.0);
	Gr2 = 2.0*J3*((a_semi/Brm)*(a_semi/Brm)*(a_semi/Brm))*cos(PI/2.0 - latc)*(5.0*cos(PI/2.0 - latc)*cos(PI/2.0 - latc) - 3.0);
	Gr3 = (5.0/8.0)*J4*((a_semi/Brm)*(a_semi/Brm)*(a_semi/Brm)*(a_semi/Brm))*(35.0*(cos(PI/2.0 - latc))*(cos(PI/2.0 - latc))*(cos(PI/2.0 - latc))*(cos(PI/2.0 - latc)) - 30.0*cos(PI/2.0 - latc)*cos(PI/2.0 - latc) + 3.0);
	Grmag = -(mu/(Brm*Brm)) * (1.0 - Gr1 - Gr2 - Gr3);
	
	Gp1 = 3.0*(mu/(Brm*Brm))*((a_semi/Brm)*(a_semi/Brm))*(sin(PI/2.0 - latc))*cos(PI/2.0 - latc);
	Gp2 = (1.0/2.0)*J3*(a_semi/Brm)*(1.0/cos(PI/2.0 - latc))*(5.0*cos(PI/2.0 - latc)*cos(PI/2.0 - latc) - 1.0);
	Gp3 = (5.0/6.0)*J4*((a_semi/Brm)*(a_semi/Brm))*(7.0*cos(PI/2.0 - latc)*cos(PI/2.0 - latc) - 3.0);
	Gpmag = Gp1*(J2 + Gp2 + Gp3);
	
	//Gravity at missile (NEDc)
	Vec gravNEDc = Vec(-Gpmag, 0.0, -Grmag);

	//ECEFtoNEDc DCM
	Mat ECEFtoNEDc = Vec(0.0, -(PI/2.0+latc), lon).getDCM();

	//Gravity in ECEF
	Vec gravECF = ECEFtoNEDc.transpose() * gravNEDc;

	return gravECF;
}

void reFactoredGeometry::dircosmat(double phi, double tht, double psi, double dcm[][3])
{
	// DETERMINE THE DIRECTION COSINE MATRIX
	// FOR REFERENCE SEE ETKIN PAGE 117
	// THIS IS THE YAW-PITCH-ROLL SEQUENCE

	double cphi,sphi,ctht,stht,cpsi,spsi;

	cphi = cos(phi);
	sphi = sin(phi);
	ctht = cos(tht);
	stht = sin(tht);
	cpsi = cos(psi);
	spsi = sin(psi);

	dcm[0][0] = ctht*cpsi;
	dcm[0][1] = ctht*spsi;
	dcm[0][2] = -stht;
	dcm[1][0] = sphi*stht*cpsi - cphi*spsi;
	dcm[1][1] = sphi*stht*spsi + cphi*cpsi;
	dcm[1][2] = sphi*ctht;
	dcm[2][0] = cphi*stht*cpsi + sphi*spsi;
	dcm[2][1] = cphi*stht*spsi - sphi*cpsi;
	dcm[2][2] = cphi*ctht;

}

void reFactoredGeometry::ecef2lla(Vec epos, double *latg, double *lon, double *alt)
{

	//---------------------------------------------------------//
	// Input:  xecf - x position in ECEF coordinates (meters)
	//         yecf - y position in ECEF coordinates (meters)
	//         zecf - z position in ECEF coordinates (meters)
	//
	// Output: latg - latitude Geodetic (radians)
	//         lon  - longitude (radians)
	//         alt  - altitude Geodetic (meters)
	//
	// Developer: Dennis Strickland
	//---------------------------------------------------------//

	double PHI1M,E2,E4,FACT1,FACT2,FACT3,EP;
	double PHI1PM,RLM, latc, RP;

	//Constants
	double FLAT   = 1.0-b_over_a;
	double ELIPFC = FLAT*(2.0 - FLAT)/((1.0 - FLAT)*(1.0 - FLAT));
	double ROM    = a_semi;
	double ECCEN2 = eccen2;

	//CALCULATE THE LAT/LONG (RELATIVE TO GREENWICH MERIDIAN)
	//AND ALTITUDE OF THE PLATFORM. NOTE GEOCENTRIC LAT PHI1M,
	//GEODETIC LAT latg
	*lon = atan2(epos.y,epos.x);
	latc = atan(epos.z/sqrt(pow(epos.x,2) + pow(epos.y,2)));
	RP    = sqrt(pow(epos.x,2) + pow(epos.y,2) + pow(epos.z,2));

	//ALTITUDE IS CALCULATED USING THE BERGER TECHNIQUE
	E2     = ECCEN2;
	E4     = E2*E2;
	FACT1  = E2*sin(2.*latc) + E4*pow(sin(latc),3)*cos(latc);
	FACT2  = E2*cos(2.*latc);
	FACT3  = .25*E4*(1. + (1. - 2.*cos(2.*latc))*cos(2.*latc));
	EP     = FACT1/(2.*(RP/ROM - FACT2 - FACT3));
	*latg  = latc + EP;

	PHI1PM = atan(pow((1. - FLAT),2)*tan(*latg));
	RLM    = ROM/sqrt(1. + ELIPFC*pow(sin(latc),2));
	//ESTIMATED ALTITUDE ABOVE THE EARTH MODEL GEOID
	*alt     = RP*cos(EP) - sqrt(pow(RLM,2) - pow((RP*sin(EP)),2)); // + 29.0; //- nav.GEOIDSEP;

}

Vec reFactoredGeometry::lla2ecef(double latg, double lon, double altg)
{
	 //Computes Earth-Centered-Earth-Fixed Coordinates from 
	 //Geodetic Latitude and Longitude
	 //--------------------------------------------------------//
	 //Input:
	 //      latg - geodetic latitude   (rad)
	 //      lon  - geodetic longitude  (rad)
	 //      altg - geodetic altitude   (m)
	 //
	 //Output:
	 //      xecef - x position in ECEF (m)
	 //      yecef - y position in ECEF (m)
	 //      zecef - z position in ECEF (m)
	 //
	 //      Developer: Dennis Strickland
	 //--------------------------------------------------------//	
	 
	 Vec epos;

	 //POSITION IN ECEF AXES FROM GEODETIC COORDINATES
	 double FLAT   = 1.0-b_over_a;
	 double ELIPFC = FLAT*(2.0 - FLAT)/((1.0 - FLAT)*(1.0 - FLAT));
	 double PHI1P  = atan((1.0- FLAT)*(1.0- FLAT)*tan(latg));
	 double RSURF  = a_semi/sqrt(1.0 + ELIPFC*sin(PHI1P)*sin(PHI1P));
	 double XLOC   = RSURF*cos(PHI1P) + altg*cos(latg);

	 epos.x = XLOC*cos(lon);
	 epos.y = XLOC*sin(lon);
	 epos.z = RSURF*sin(PHI1P) + altg*sin(latg);

	 return epos;
	 
}
