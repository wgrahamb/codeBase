//---------------------------------------------------------------------//
// File: Geometry.h
//
// This class determines the geometry necessary for an oblate
// rotating earth missile flight.  The primary Direction-Cosine-Matrices
// are determined in this routine.
//
//  Developer:  Dennis Strickland
//---------------------------------------------------------------------//

#ifndef REFACTOREDGEOMETRY_H
#define REFACTOREDGEOMETRY_H
#include "main.h"

class reFactoredGeometry
{

public:
	reFactoredGeometry();
	
	int kt;
	double t, tick, pretime;

	Vec  lla2ecef(double latg, double lon, double altg);
	void ecef2lla(Vec epos, double *latg, double *lon, double *alt);
	void dircosmat(double phi, double tht, double psi, double dcm[][3]);
	void update_ECI2ECEF(double time, Mat &i2e);
	Vec gravity(Vec epos);

	Mat ECItoECEF;
	Mat ECItoBODY;
	Mat ECEFtoNEDg;
	Mat ECEFtoNEDb;
	Mat ECEFtoBODY;
	Mat NEDgtoBODY;
	Mat NEDbtoBODY;
	Mat LTFtoBODY;
	Mat NEDtoLTF;

	double dxt, dyt, dzt;
	double latg, lon, altg;
	double phiLnch, thtLnch, psiLnch;
	double phiLnch_Err, thtLnch_Err, psiLnch_Err;
	double latLoc_Err, lonLoc_Err, altLoc_Err;

	double xecf, yecf, zecf;
	double latc, Brm;
	double elipfc, eccen, e2, e4;
	double eps_num, eps_fac1, eps_fac2, eps_fac3;
	double epsilon, phi1pm, rm;
	double Gr1, Gr2, Gr3, Grmag;
	double Gp1, Gp2, Gp3, Gpmag;
	double grav_est;
	double gx_nedc, gy_nedc, gz_nedc;
	double gravx, gravy, gravz;

	void init(double phi0, double tht0, double psi0, double lat0, double lon0, double alt0, double targetAz);
	void update(double totalSystemTime, Quat eulerQuaternion);

};

#endif