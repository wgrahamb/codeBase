
// Standard.
#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <math.h>
#include <vector>
#include <map>
#include <algorithm>
#include <string>
#include <dirent.h>

// Namespace.
using namespace std;

#ifndef MISSILEDATA_H
#define MISSILEDATA_H

// 70 MM Rocket and MK151 High Explosive Warhead.
const double ROCKET_BURNOUT = 1.11; // sec.
const double REF_LENGTH = 55.13; // Inches.
const double REF_DIAM = 2.75; // Inches.
const double REF_AREA = 3.14159 * REF_DIAM * REF_DIAM / 4.0;
const vector<vector<double>> WEIGHT_V_TIME{ // Lbm.
	{0.0, 22.95},
	{1.11, 15.73}
};
const vector<vector<double>> XCG_V_TIME{ // Inches from base.
	{0.0, 29.96},
	{1.11, 33.55}
};
const vector<vector<double>> AMOI_V_TIME{ // Lbf-in^2.
	{0.0, 26.2},
	{1.11, 19.7}
};
const vector<vector<double>> TMOI_V_TIME{ // Lbf-in^2.
	{0.0, 6248.0},
	{1.11, 5008.0}
};
const vector<vector<double>> THRUST_V_TIME{ // Lbf.
	{0.0, 0.0},
	{0.01, 1304.3},
	{0.04, 1400},
	{0.06, 1439.10},
	{0.19, 1245.70},
	{0.41, 1109},
	{0.44, 1267.20},
	{0.46, 1276.90},
	{0.49, 1451.80},
	{0.51, 1457.70},
	{0.54, 1267.20},
	{0.56, 1234},
	{0.86, 1522.20},
	{0.89, 1485},
	{0.91, 1611.10},
	{0.94, 1654.10},
	{0.96, 1780.10},
	{0.99, 1792.80},
	{1.04, 1463.50},
	{1.06, 1070.80},
	{1.09, 491.40},
	{1.11, 146.60},
	{1.12, 0.0}
};
const vector<vector<double>> TORQUE_V_TIME{ // Lbf-In.
	{0.0, 0.0},
	{0.01, 39.1},
	{0.04, 42.0},
	{0.06, 43.2},
	{0.19, 37.4},
	{0.41, 35.7},
	{0.44, 38.0},
	{0.46, 38.3},
	{0.49, 43.6},
	{0.51, 43.7},
	{0.54, 38.0},
	{0.56, 37.0},
	{0.86, 45.7},
	{0.89, 44.6},
	{0.91, 48.3},
	{0.94, 49.6},
	{0.96, 53.4},
	{0.99, 53.8},
	{1.04, 43.9},
	{1.06, 32.1},
	{1.09, 14.3},
	{1.11, 4.40},
	{1.12, 0.0}
};
const vector<vector<double>> CNA_V_MACH{ // Per rad.
	{0.0, 8.19},
	{0.6, 8.19},
	{0.9, 8.94},
	{1.15, 9.34},
	{1.3, 8.88},
	{1.6, 8.14},
	{2.48, 7.51},
	{2.97, 7.22}
};
const vector<vector<double>> XCP_V_MACH{ // Inches from nose.
	{0.0, 13.39 * REF_DIAM}, // Measured in units of 2.75 inches = 1 Caliber.
	{0.6, 13.3 * REF_DIAM},
	{0.9, 13.86 * REF_DIAM},
	{1.15, 14.50 * REF_DIAM},
	{1.3, 14.52 * REF_DIAM},
	{1.6, 13.85 * REF_DIAM},
	{2.48, 13.41 * REF_DIAM},
	{2.97, 13.36 * REF_DIAM}
};
const vector<vector<double>> CMQ_V_MACH{ // Per rad.
	{0.0, 1060.0},
	{0.6, 1060.0},
	{0.9, 1460.0},
	{1.15, 1385.0},
	{1.3, 1193.0},
	{1.6, 1069.0},
	{2.48, 850},
	{2.97, 800}
};
const vector<vector<double>> CL_V_MACH{ // Non dimensional.
	{0.0, -0.12},
	{0.6, -0.12},
	{0.9, -0.12},
	{1.0, -0.1},
	{1.1, -0.08},
	{1.15, -0.07},
	{1.3, -0.02},
	{1.6, -0.04},
	{1.9, -0.05},
	{2.2, -0.06},
	{2.5, -0.06},
	{3.0, -0.07},
};
const vector<vector<double>> CLD_V_MACH{ // Per rad.
	{0.0, 2.92},
	{0.6, 2.98},
	{0.9, 3.09},
	{1.0, 3.21},
	{1.1, 3.49},
	{1.15, 3.67},
	{1.3, 4.01},
	{1.6, 3.9},
	{1.9, 3.49},
	{2.2, 3.09},
	{2.5, 2.81},
	{3.0, 2.29},
};
const vector<vector<double>> CLP_V_MACH{ // Per rad.
	{0.0, -5.6},
	{0.6, -6.1},
	{0.9, -6.4},
	{1.0, -6.9},
	{1.1, -7.8},
	{1.15, -8.05},
	{1.3, -8.15},
	{1.6, -8.0},
	{1.9, -7.6},
	{2.2, -7.1},
	{2.5, -6.7},
	{3.0, -6.0},
};
const vector<vector<double>> CD_ON_V_MACH{ // Per rad.
	{0.0, 0.55},
	{0.78, 0.55},
	{0.82, 0.58},
	{0.90, 0.63},
	{0.94, 0.65},
	{1.0, 0.69},
	{1.03, 0.70},
	{1.06, 0.71},
	{1.10, 0.73},
	{1.15, 0.74},
	{1.18, 0.75},
	{1.28, 0.76},
	{1.34, 0.76},
	{1.48, 0.75},
	{1.58, 0.74},
	{1.71, 0.72},
	{1.94, 0.68},
	{2.20, 0.65},
	{2.40, 0.63},
	{2.60, 0.61},
	{3.0, 0.60}
};
const vector<vector<double>> CD_OFF_V_MACH{ // Per rad.
	{0.0, 0.7},
	{0.78, 0.7},
	{0.82, 0.73},
	{0.90, 0.81},
	{0.94, 0.86},
	{1.0, 0.96},
	{1.03, 0.98},
	{1.06, 0.99},
	{1.10, 1.0},
	{1.15, 1.01},
	{1.18, 1.01},
	{1.28, 1.01},
	{1.34, 1.01},
	{1.48, 0.97},
	{1.58, 0.97},
	{1.71, 0.94},
	{1.94, 0.88},
	{2.20, 0.87},
	{2.40, 0.77},
	{2.60, 0.73},
	{3.0, 0.70}
};

#endif
