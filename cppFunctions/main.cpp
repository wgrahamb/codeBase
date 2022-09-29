
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
#include "ATM1976.h"

int main()
{

     auto data = ATM1976::update(1000.0, 100.0);
     cout << "HOWDY\n";

}
