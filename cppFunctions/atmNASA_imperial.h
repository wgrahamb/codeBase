#pragma once

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
#include <vector>

// Namespace.
using namespace std;

namespace
{

	struct atmNASA_output
	{
		float rho; // sl per ft^3
		float p; // psf
		float a; // ft/s
		float g; // ft/s^2
		float q; // psf
		float tF; // Farenheit
		float mach; // non dimensional
	};

}

namespace atmNASA_imperial
{

	const static inline atmNASA_output update(float altitude, float speed) // ft, ft/s
	{

		const double RAD = 2.0856e7;

		atmNASA_output ret;

		float     alt;
		if        (altitude < 0.0) alt = 0.0;
		else      alt = altitude;

		if (alt < 36152.0)
		{
			ret.tF = 59 - 0.00356 * alt;
			ret.p = 2116.0 * pow(((ret.tF + 459.7) / 518.6), 5.256);
		}
		else if (36152.0 <= alt < 82345.0)
		{
			ret.tF = (-1.0 * 70.0);
			ret.p = 473.1 * (exp(1.73 - 0.000048 * alt));
		}
		else
		{
			ret.tF = (-1.0 * 205.05) + 0.00164 * alt;
			ret.p = 51.97 * pow(((ret.tF + 459.7) / 389.98), (-1.0 * 11.388));
		}

		ret.rho = ret.p / (1718.0 * (ret.tF + 459.7));
		ret.q = 0.5 * ret.rho * speed * speed;
		ret.g = 32.2 * pow((RAD / (RAD + alt)), 2);
		float tK = (ret.tF + 459.7) * (5.0 / 9.0);
		ret.a = sqrt(tK * 1.4 * 286) * 3.28084;
		ret.mach = speed / ret.a;

		return ret;

	}

}