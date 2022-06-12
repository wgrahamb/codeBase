// Standard library.
#include "iostream"
#include "iomanip"
#include "fstream"
#include "math.h"
#include "algorithm"
#include "map"
#include "random"
#include "chrono"
#include "vector"
#include "map"

// Local utility.
#include "../util.h"
#include "../constants.h"

// Third party.
#include "eigen-3.4.0/Eigen/Core"
#include "eigen-3.4.0/Eigen/Dense"

// Namespace.
using namespace std;
using namespace Eigen;

class MMTPitchAndYawAutoPilot
{

	public:
		MMTPitchAndYawAutoPilot();
		map<string, int> tableNameIndexPairs;
		vector<vector<vector<double>>> tables;
		void lookUpTablesFormat(string inPutFile);
		void init();
		void update();

};