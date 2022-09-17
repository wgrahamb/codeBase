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

// Namespace.
using namespace std;

#ifndef RANDOMNUMBERS_H
#define RANDOMNUMBERS_H

class randomNumbers
{

	public:

	randomNumbers();
	void init(long seed);
	void writeDraws(string relFilePath);
	float makeUniformDraw(string ID, float min, float max);
	float makeNormalDraw(string ID, float mean, float stdev);
	float makeUniformNoise(float min, float max);
	float makeNormalNoise(float mean, float stdev);

	private:

	float randomNumberGenerator();
	float uniformDistribution(float min, float max);
	float normalDistribution(float mean, float stdev);
	struct DRAW
	{
		string ID;
		string DRAW_TYPE;
		string DRAW_VALUE;
	};
	vector<DRAW> DRAWS;
	long SEED;
	int RUN_NUM = 0;

};

#endif