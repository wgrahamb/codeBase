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

#include "eigen-3.4.0/eigen-3.4.0/Eigen/Core"
#include "eigen-3.4.0/eigen-3.4.0/Eigen/Dense"

using namespace std;
using namespace tframes;

struct NavigationState
{

	public:
		const double missileTimeOfFlight_;
		const Eigen::Vector3d missileLTFPosition_;
		const Eigen::Vector3d missileLTFVelocity_;
		const Eigen::Vector3d missileBodyAcceleration_;
		const Eigen::Vector3d missileLTFEulerAngles_;
		const Eigen::Vector3d missileBodyRate_;
		const Eigen::Vector3d missileBodyRateDot_;

		NavigationState(
			double missileTimeOfFlight,
			Eigen::Vector3d missileLTFPosition,
			Eigen::Vector3d missileLTFVelocity,
			Eigen::Vector3d missileBodyAcceleration,
			Eigen::Vector3d missileLTFEulerAngles,
			Eigen::Vector3d missileBodyRate,
			Eigen::Vector3d missileBodyRateDot
		) : 
		missileTimeOfFlight_(missileTimeOfFlight),
		missileLTFPosition_(missileLTFPosition),
		missileLTFVelocity_(missileLTFVelocity),
		missileBodyAcceleration_(missileBodyAcceleration),
		missileLTFEulerAngles_(missileLTFEulerAngles),
		missileBodyRate_(missileBodyRate),
		missileBodyRateDot_(missileBodyRateDot) {}

};

#endif

#endif