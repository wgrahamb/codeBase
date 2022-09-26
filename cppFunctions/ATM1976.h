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

struct ATM1976_OUT
{

	float rho; // kg per m^3
	float p; // pascals
	float a; // m/s
	float g; // m/s^2
	float q; // pascals
	float tk; // Kelvin
	float mach; // non dimensional

};

class ATM1976
{

	private:

	const float R = 287.053; // air constant
	const float G = 6.673e-11; // gravity constant
	const float earthMass = 5.973e24; // kg
	const float rEarth = 6369; // radius of earth, km
	const float gmr = 34.63195; // gas constant
	const float rhosl = 1.225; // rho sea level, kg/m^3
	const float pressl = 101325; // pressure sea level, pascals
	const float tempksl = 288.15; // rho sea level, kg/m^3
	const vector<float> htab =
	{0.0, 11.0, 20.0, 32.0, 47.0, 51.0, 71.0, 84.852};
	const vector<float> ttab =
	{288.15, 216.65, 216.65, 228.65, 270.65, 270.65, 214.65, 186.946};
	const vector<float> ptab =
	{1.0, 2.233611e-1, 5.403295e-2, 8.5666784e-3,
	1.0945601e-3, 6.6063531e-4, 3.9046834e-5, 3.68501e-6};
	const vector<float> gtab =
	{-6.5, 0.0, 1.0, 2.8, 0.0, -2.8, -2.0, 0.0};

	public:

	inline ATM1976_OUT update(float altitude, float speed) // m, m/s
	{

		ATM1976_OUT ret;
		
		float delta;
		float alt = altitude / 1000.0;
		float hgt = alt * rEarth / (alt + rEarth);

		int i = 0;
		int j = 7;
		while (true)
		{
			int k = i + j / 2;
			if (hgt < htab[k])
			{
				j = k;
			}
			else
			{
				i = k;
			}
			if (j <= (i + 1))
			{
				break;
			}
		}

		if (alt < 84.852)
		{

			float tgrad = gtab[i];
			float tbase = ttab[i];
			float deltah = hgt - htab[i];
			float tlocal = tbase + tgrad * deltah;
			float theta = tlocal / ttab[0];

			if (tgrad == 0)
			{
				delta = ptab[i] * exp(-1.0 * gmr * deltah / tbase);
			}
			else
			{
				delta = ptab[i] * pow((tbase / tlocal), (gmr / tgrad));
			}

			float sigma = delta / theta;

			ret.rho = rhosl * sigma;
			ret.p = pressl * delta;
			ret.tk = tempksl * theta;

		}
		else
		{

			ret.rho = 0.0;
			ret.p = 0.0;
			ret.tk = 186.946;

		}

		ret.a = sqrt(1.4 * R * ret.tk);
		ret.q = 0.5 * ret.rho * speed * speed;
		ret.mach = speed / ret.a;
		float rad = (rEarth + alt) * 1000;
		ret.g = G * earthMass / (rad * rad);
		return ret;

	};

};