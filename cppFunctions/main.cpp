
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
#include <memory>

// Utility.
#include "global_constants.hpp"
#include "global_header.hpp"
#include "utility_header.hpp"
#include "util.h"

// Namespace.
using namespace std;

// Include.
#include "secondOrderActuator.h"
#include "randomNumbers.h"
#include "atm1976_metric.h"
#include "atmNASA_imperial.h"

int main()
{

     auto data = atm1976_metric::update(1000.0, 100.0);
     auto x = atmNASA_imperial::update(1000.0, 100.0);
     cout << "HOWDY\n";

}
