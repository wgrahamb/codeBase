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

// Local utility.
#include "../util.h"

// Namespace.
using namespace std;

class MMTMassAndMotorProperties
{
	public:

		MMTMassAndMotorProperties();
		map<string, int> tableNameIndexPairs;
		vector<vector<vector<double>>> tables;
		void lookUpTablesFormat(string inPutFile);
		void init();
		void update();

		// Time management.
		double timer;
		const double timeStep = 0.001;

		// Output.
		double mass, thrust, centerOfGravity, inertia[3][3];

};