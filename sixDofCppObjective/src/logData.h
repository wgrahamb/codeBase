#include "util.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <math.h>
#include <vector>
#include <map>
#include <algorithm>

class logData
{

	public:

		// CONSTRUCTOR
		logData(mslDataPacket *dataPacket);
		std::ofstream logFile;

		// UPDATE FUNCTION
		void update(mslDataPacket *dataPacket);

};