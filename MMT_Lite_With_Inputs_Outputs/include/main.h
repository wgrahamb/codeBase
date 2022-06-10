//-----------------------------------------------------------------//
#define SIXDOF
#ifndef MAIN_H
#define MAIN_H

#ifdef SIXDOF 
#define STAMP "5-11-17"
#include <cstdio>
#include <iostream>
#include <cstring>
#include <iomanip>

#include "block.h"
#include "Output.h"
#include "SimConstants.h"
#include "vec.h"
#include "mat.h"
#include "quat.h"
#include "vecff.h"
#include "matff.h"
#include "quatff.h"
#include "Util.h"
#include "Filer.h"
#include "table1.h"
#include "table2.h"
#include "table3.h"
#include "table4.h"
#include "table5.h"
#include "table1ff.h"
#include "table2ff.h"

using namespace std;
using namespace tframes;

struct NAV_STATE
{
	double missileTimeOfFlight; // Seconds.
	Vec LTFPosition; // Meters.
	Vec LTFVelocity; // Meters per second.
	Vec specificForce; // Meters per s^2.
	Vec LTFEulerAngles; // Radians.
	Vec bodyRate; // Radians per second.
	Vec bodyRateDot; // Radians per second.
	Vec LTFWayPoint; // Meters.
};

#endif

#endif