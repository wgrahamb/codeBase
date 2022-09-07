
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
#include "randomNumbers.h"

int main()
{

     shared_ptr<randomNumbers> rn = make_shared<randomNumbers>();

     long seed1 = 42000;
     long seed2 = 100000;
     long seed3 = 7777777;

     rn->init(seed3);
     for (int i = 0; i < 10000; i++)
     {
          float x = rn->makeNormalDraw(to_string(i), 0.1, 0.5);
          // float x = rn->makeUniformDraw(to_string(i), -0.1, 0.1);
     }
     string relFilePath = "output/MCData.txt";
     rn->writeDraws(relFilePath);
     cout << "\n";

     cout << "HOWDY\n";

}
