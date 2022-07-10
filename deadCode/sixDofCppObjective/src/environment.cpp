#include "math.h"
#include "iostream"
#include "environment.h"
#include "util.h"

environment::environment(mslDataPacket *dataPacket)
{
	std::cout << "ENVIRONMENT INITIATED" << std::endl;
	dataPacket->mslDynamicPressure = 0.0;
	dataPacket->mslTotalPressure = 101325.0; // STANDARD CONDITION AT ZERO ALTITUDE
	dataPacket->mslSpeedOfSound = 343.0; // STANDARD CONDITION AT ZERO ALTITUDE
}

void environment::update(mslDataPacket *dataPacket) {
	/*  
	STANDARD 1962 ATMOSPHERE MODEL.
	IS IN PRACTICAL AGREEMENT WITH THE ICAO STANDARD MODEL OVER THEIR
	COMMON ALTITUDE RANGE.

	INPUT:
	HEIGHT  ALTITUDE H (M OR FT)

	OUTPUT:
	VSOUND  VELOCITY OF SOUND AT H. (M/SEC OR FT/SEC)
	RHO     DENSITY OF AIR AT H. (KG/M**3 OR SLUG/FT**3)
	PRESS   PRESSURE OF AIR AT H.  (KG/M**2 OR LBS/FT**2)
	TEMP    TEMPERATURE OF AIR AT H.  (DEG K OR DEG R)

	REMARKS:
	THIS ROUTINE PROVIDES EXTRAPOLATED VALUES FOR H ABOVE
	700000.0 METERS.

	REFERENCE:
	U.S. STANDARD ATMOSPHERE, 1962
	U.S. GOVERNMENT PRINTING OFFICE
	WASHINGTON 25, D.C.

	v1.00, converted from a Fortran version by Ray Sells
	v1.00 by Ray Sells
	C++ version by Ray Sells, 4-19-02
	*/
	
	double altitude = dataPacket->mslPos[2];
	double z, z1, ep, xm, h2, h3, h4, h5, h6, tm;
	int i;

	double hh[9] =  {0.0, 11.0, 20.0, 32.0, 47.0, 52.0, 61.0, 79.0, 88.744};
	double pbh[9] = {0.0, 10331.9076, 2307.7398, 558.2615, 88.50965, 11.30876, 6.016163, 1.85682, .105812};
	double rlh[9] = {0.0, -6.5, 0.0, 1.0, 2.8, 0.0, -2.0, -4.0, 0.0};
	double hbh[9] = {0.0, 0.0, 11.0, 20.0, 32.0, 47.0, 52.0, 61.0, 79.0};
	double tbh[9] = {0.0, 288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 252.65, 180.65};
	double zz[14] = {0.0, 100., 110., 120., 150., 160., 170., 190., 230., 300., 400., 500., 600, 700.};
	double pb[14] = {0.0, .016762, .0030667, .7499e-3,.2571e-3, .516e-4,.3767e-4, .28476e-4,.17184e-4,.709767e-5, .1921e-5,.41097e-6,.1117e-6, .3518e-7};
	double rl[14] = {0.0, 3.0, 5.0, 10.0, 20.0, 15.0, 10.0, 7.0, 5.0, 4.0, 3.3, 2.6, 1.7, 1.1};
	double zb[14] = {0.0, 90.0, 100.0, 110.0, 120.0, 150.0, 160.0, 170.0, 190.0, 230.0, 300.0, 400.0, 500.0, 600.0};
	double tb[14] = {0.0, 180.65, 210.65, 260.65, 360.65, 960.65, 1110.65, 1210.65, 1350.65, 1550.65, 1830.65, 2160.65, 2420.65, 2590.65};
	double xxx[14] = {0.0, 33.166119, 33.064396, 32.96512, 32.784318, 32.530427, 32.455229, 32.309443, 32.016142, 31.492584, 30.704189, 29.808321, 28.937691, 28.123252};

	double aa = 1.5731262e-7;
	double bb = 2.4656553e-14;
	double cc = 3.8667054e-21;
	double dd = 6.0621354e-28;
	double ee = 9.5013649e-35;

	if(altitude < 0.0) altitude = 0.0;

	z = altitude * 0.001;
	if( z <= 90.0) /* 0 to 90 km */
	{ 
		h2 = altitude * altitude;
		h3 = h2 * altitude;
		h4 = h3 * altitude;
		h5 = h4 * altitude;
		h6 = h5 * altitude;

		altitude = ( altitude - aa * h2 + bb * h3 - cc * h4 + dd * h5 - ee * h6) * 0.001;

		for( i = 1; i <= 8; i++) 
		{
			if( altitude <= hh[i]) break;
		}
		if( i == 2 || i == 5 || i== 8) 
		{
			temperature = tbh[i];
			pressure = pbh[i] * exp( -34.16479 * ( altitude - hbh[i]) / tbh[i]);
		}
		else 
		{
			temperature = tbh[i] + ( altitude - hbh[i]) * rlh[i];
			ep = 34.16479 / rlh[i];
			pressure = pbh[i] * pow( tbh[i] / temperature, ep);
		}
		rho = pressure / ( 29.26945 * temperature);
		speedOfSound = 20.046333 * sqrt(temperature);
		/* convert press from kg/m^2 to N/m^2 */
		pressure *= 9.80665;
	}
	else /* 90 - 700 km */
	{ 
		z1 = altitude;
		if( z1 <= 170e3) 
		{
			xm = ( ( ( ( ( 0.14186509e-27 * 
					z1 - 0.111458341e-21) *
					z1 + 0.359201416e-16) * 
					z1 - 0.60665213e-11) *
					z1 + 0.565215183e-6) * 
					z1 - 0.275300356e-1) *
					z1 + 5.76957191e2;
		}
		else {
			xm = ( ( ( ( ( 1.180554e-33 *
					z1 - 3.429162e-27) *
					z1 + 3.8159669e-21) * 
					z1 - 2.0402054e-15) * 
					z1 + 5.6422141e-10) * 
					z1 - 1.0700489e-4) * 
					z1 + 3.5629995e+1;
		}
		for (i = 1; i <= 13; i++) 
		{
			if( z <= zz[i]) break;
		}
		if (i >= 13) i = 13;
		tm = tb[i] + ( z - zb[i]) * rl[i];
		temperature = tm * xm / 28.694;
		ep = xxx[i] / rl[i];
		pressure = pb[i] * pow( tb[i] / tm, ep);
		rho = pressure / ( 29.26945 * tm);
		speedOfSound = sqrt( 401.90467 * tm);

		/* convert press from kg/m^2 to N/m^2 */
		pressure *= 9.80665;
	}

	dataPacket->mslTotalPressure = pressure;
	dataPacket->mslSpeedOfSound = speedOfSound;
	dataPacket->mslDynamicPressure = 0.5 * rho * dataPacket->mslSpeed * dataPacket->mslSpeed;
}