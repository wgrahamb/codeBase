//-----------------------------------------------------------//
// File: SimConstants.h
// 
// This header file provides various constant values used by
// the simulation.
//                                                           
// Developer: Dennis Strickland
//-----------------------------------------------------------//

#ifndef CONSTANTS_H
#define CONSTANTS_H

// General Constants

const double PI    = 3.14159265358979323846;  // (rad)
const double TWOPI = 2.0 * PI;                // (rad)
const double EPS   = 1.0e-6;

const double RE = 6371008.7714; // mean radius of the semiaxes
                                // in WGS84 ellipsoid (m)
const double WE = 7.292115e-5;  // earth rotation rate (rad/sec)
const double GM = 3986005.0e8;  // earth grav. constant (m3/sec)
const double G  = 9.80665;

// Unit Conversion Constants
const double c0 = 299792458.0;    // (m/s)
const double rtd = 180.0 / PI;  // (deg/rad)
const double dtr = PI / 180.0;  // (rad/deg)
const double LBF_2_N = 4.4482216;     // (N/lbf)
const double N_2_LBF = 0.2248;
const double mtf = 3.281;
const double ftm = 0.3048;
const double KG_2_LB = 2.205;
const double LB_2_KG = 0.4536;
const double KT0 = 4e-21;

// Missile Constants.
const double nozzleExitArea = 0.000426642;
const double referenceArea = 0.003916121;
const double referenceDiameter = 0.070612;
const double imuOffSet = 0.235585;
const double railLength = 1.42233;

//Earth Constants
const double b_over_a = 0.9966471893;     //Flatness Factor
const double a_semi   = 6378137.0;
const double J2       = 1.08263e-3;       //Spherical Harmonics
const double J3       = 0.0;
const double J4       = -2.37091e-6;
const double mu       = 3.986005e14;      //Gravitational Constant m^3/s^2
const double omegae   = 7.2921151467e-5;  //(rad/sec)
const double eccen2   = 6.69437999013e-3;

const int MODE_POWER_ON          = -15;
const int MODE_INITIALIZE        = -10;
const int MODE_PRELAUNCH         = -5;
const int MODE_READY             =  0;
const int MODE_LAUNCH            =  5;
const int MODE_BREAKWIRE         = 10;
const int MODE_FREEFLIGHT        = 15;
const int MODE_ROLL_AUTOP_ACTIVE = 20;
const int MODE_PY_AUTOP_ACTIVE   = 25;
const int MODE_BURNOUT           = 30;
const int MODE_BURNOUT_DETECT    = 30;
const int MODE_MIDCOURSE_MODE1   = 35;
const int MODE_MIDCOURSE_MODE2   = 40;
const int MODE_SEEKER_ACTIVE     = 45;
const int MODE_TERMGUIDE_ACTIVE  = 50;
const int MODE_STOP              = 99;

#endif

