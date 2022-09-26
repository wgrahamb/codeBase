
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

     ATM1976 *x = new ATM1976();
     ATM1976_OUT data = x->update(1000.0, 100.0);

     cout << "HOWDY\n";

}
