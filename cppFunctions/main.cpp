
// STL.
#include <iostream>

// Utility.
#include "global_constants.hpp"
#include "global_header.hpp"
#include "utility_header.hpp"
#include "util.h"

// Namespace.
using namespace std;

int main()
{

	cout << "HOWDY\n";

	double LAT = 38.8719 * RAD;
	double LON = 77.0563 * RAD;
	double ALT = 0.0;
	double TOF = 10.0;

	Matrix ECI = cad_in_geo84(LON, LAT, ALT, TOF);
	ECI.print();

	Matrix ECIGRAV = cad_grav84(ECI, TOF);
	ECIGRAV.print();

	return 0;

}
