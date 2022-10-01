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

#ifndef WIN32
#include <dirent.h>
#endif

// Namespace.
using namespace std;

// Utility.
#include "util.h"

#ifndef WIN32
// GB. Lists the files in the input directory.
void loopThroughDirectory(string dirpath)
{

	struct dirent *entry = nullptr;
	DIR *dp = nullptr;
	dp = opendir(dirpath.c_str());
	if (dp != nullptr)
	{
		while ((entry = readdir(dp)))
		{
			string fileName = entry->d_name;
			cout << fileName << endl;
		}
	}
	closedir(dp);

};
#endif

// Returns the sign of a number.
double signum(double x)
{
	double y;
	if( x < 0.0)
	{
		y = -1.0;
	}
	else if (x > 0.0)
	{
		y = 1.0;
	}
	else
	{
		y = 0.0;
	}
	return y;
}

// GB. Simple integration method.
double trapezoidIntegrate(double dy_new, double dy, double y, double intStep)
{
	return y + ((dy_new + dy) * intStep / 2);
}

// GB.
void flightPathAnglesToLocalOrientation (double azimuth, double elevation, double localFrame[3][3])
{
	localFrame[0][0] = cos(elevation) * cos(azimuth);
	localFrame[0][1] = cos(elevation) * sin(azimuth);
	localFrame[0][2] = -1 * sin(elevation);
	localFrame[1][0] = -1 * sin(azimuth);
	localFrame[1][1] = cos(azimuth);
	localFrame[1][2] = 0;
	localFrame[2][0] = sin(elevation) * cos(azimuth);
	localFrame[2][1] = sin(elevation) * sin(azimuth);
	localFrame[2][2] = cos(elevation);
}

// GB. Direction cosine matrix.
void eulerAnglesToLocalOrientation (double phi, double theta, double psi, double matrix[3][3])
{
	matrix[0][0] = cos(psi) * cos(theta);
	matrix[0][1] = sin(psi) * cos(theta);
	matrix[0][2] = -1 * sin(theta);
	matrix[1][0] = cos(psi) * sin(theta) * sin(phi) - sin(psi) * cos(phi);
	matrix[1][1] = sin(psi) * sin(theta) * sin(phi) + cos(psi) * cos(phi);
	matrix[1][2] = cos(theta) * sin(phi);
	matrix[2][0] = cos(psi) * sin(theta) * cos(phi) + sin(psi) * sin(phi);
	matrix[2][1] = sin(psi) * sin(theta) * cos(phi) - cos(psi) * sin(phi);
	matrix[2][2] = cos(theta) * cos(phi);
}

// GB. Unit vector.
void unitVec (double vector[3], double unitVector[3])
{
	double mag = sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
	unitVector[0] = vector[0] / mag;
	unitVector[1] = vector[1] / mag;
	unitVector[2] = vector[2] / mag;
}

// GB.
void azAndElFromVector (double &az, double &el, double vector[3])
{
	az = atan2(vector[1], vector[0]);
	double t1 = pow(vector[0], 2);
	double t2 = pow(vector[1], 2);
	double t3 = sqrt(t1 + t2);
	el = atan2(vector[2], t3);
}

// GB.
void oneByThreeTimesThreeByThree(double arr[3], double matrix[3][3], double out[3])
{
	out[0] = matrix[0][0] * arr[0] + matrix[1][0] * arr[1] + matrix[2][0] * arr[2];
	out[1] = matrix[0][1] * arr[0] + matrix[1][1] * arr[1] + matrix[2][1] * arr[2];
	out[2] = matrix[0][2] * arr[0] + matrix[1][2] * arr[1] + matrix[2][2] * arr[2];
}

// GB.
void threeByThreeTimesThreeByOne(double matrix[3][3], double arr[3], double out[3])
{
	out[0] = matrix[0][0] * arr[0] + matrix[0][1] * arr[1] + matrix[0][2] * arr[2];
	out[1] = matrix[1][0] * arr[0] + matrix[1][1] * arr[1] + matrix[1][2] * arr[2];
	out[2] = matrix[2][0] * arr[0] + matrix[2][1] * arr[1] + matrix[2][2] * arr[2];
}

// GB.
void threeByThreeTimesThreeByThree(double mat1[3][3], double mat2[3][3], double out[3][3])
{
	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			out[i][j] = mat1[i][0] * mat2[0][j] + mat1[i][1] * mat2[1][j] + mat1[i][2] * mat2[2][j];
		}
	}
}

// GB.
void magnitude(double arr[3], double &out)
{
	double t1 = pow(arr[0], 2);
	double t2 = pow(arr[1], 2);
	double t3 = pow(arr[2], 2);
	out = sqrt(t1 + t2 + t3);
}

// GB.
void subtractTwoVectors(double vec1[3], double vec2[3], double out[3])
{
	out[0] = vec2[0] - vec1[0];
	out[1] = vec2[1] - vec1[1];
	out[2] = vec2[2] - vec1[2];
}

// GB.
void addTwoVectors(double vec1[3], double vec2[3], double out[3])
{
	out[0] = vec2[0] + vec1[0];
	out[1] = vec2[1] + vec1[1];
	out[2] = vec2[2] + vec1[2];
}

// GB.
void multiplyTwoVectors(double vec1[3], double vec2[3], double out[3])
{
	out[0] = vec2[0] * vec1[0];
	out[1] = vec2[1] * vec1[1];
	out[2] = vec2[2] * vec1[2];
}

// GB.
void crossProductTwoVectors (double vec1[3], double vec2[3], double out[3])
{
	out[0] = vec1[1] * vec2[2] - vec1[2] * vec2[1];
	out[1] = vec1[2] * vec2[0] - vec1[0] * vec2[2];
	out[2] = vec1[0] * vec2[1] - vec1[1] * vec2[0];
}

// GB.
void dotProductTwoVectors (double vec1[3], double vec2[3], double &out)
{
	out = vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2];
}

// GB.
void vectorProjection (double uv[3], double vec[3], double out[3])
{
	double uvXvu[3][3];
	uvXvu[0][0] = uv[0] * uv[0];
	uvXvu[0][1] = uv[0] * uv[1];
	uvXvu[0][2] = uv[0] * uv[2];
	uvXvu[1][0] = uv[1] * uv[0];
	uvXvu[1][1] = uv[1] * uv[1];
	uvXvu[1][2] = uv[1] * uv[2];
	uvXvu[2][0] = uv[2] * uv[0];
	uvXvu[2][1] = uv[2] * uv[1];
	uvXvu[2][2] = uv[2] * uv[2];
	out[0] = uvXvu[0][0] * vec[0] + uvXvu[0][1] * vec[1] + uvXvu[0][2] * vec[2];
	out[1] = uvXvu[1][0] * vec[0] + uvXvu[1][1] * vec[1] + uvXvu[1][2] * vec[2];
	out[2] = uvXvu[2][0] * vec[0] + uvXvu[2][1] * vec[1] + uvXvu[2][2] * vec[2];
}

// GB.
void multiplyVectorTimesScalar(double scalar, double vec[3], double out[3])
{
	out[0] = scalar * vec[0];
	out[1] = scalar * vec[1];
	out[2] = scalar * vec[2];
}

// GB.
void divideVectorByScalar(double scalar, double vec[3], double out[3])
{
	out[0] = vec[0] / scalar;
	out[1] = vec[1] / scalar;
	out[2] = vec[2] / scalar;
}

// GB.
void setArrayEquivalentToReference(double arrayToBeChanged[3], double reference[3])
{
	arrayToBeChanged[0] = reference[0];
	arrayToBeChanged[1] = reference[1];
	arrayToBeChanged[2] = reference[2];
}

// GB.
void setArrayEquivalentToZero(double array[3])
{
	array[0] = 0.0;
	array[1] = 0.0;
	array[2] = 0.0;
}

// GB.
void consolePrintArray(string id, double array[3])
{
	cout << id << " " << array[0] << " " << array[1] << " " << array[2] << "\n";
}

// GB.
double linearInterpolationWithBoundedEnds(std::vector<std::vector<double>> table, double tableInput)
{
	int lowIndex = -100000;
	int highIndex = 100000;
	double interpolatedValue;
	for (int i = 0; i < table.size(); i++)
	{
		double refValue = table[i][0];
		if (refValue > tableInput)
		{
			if (highIndex == 100000)
			{
				highIndex = i;
			}
			else if (refValue < table[highIndex][0])
			{
				highIndex = i;
			}
		}
		else if (refValue < tableInput)
		{
			if (lowIndex == -100000)
			{
				lowIndex = i;
			}
			else if (refValue > table[lowIndex][0])
			{
				lowIndex = i;
			}
		}
	}
	if (lowIndex == -100000)
	{
		lowIndex = 0;
		interpolatedValue = table[lowIndex][1];
	}
	else if (highIndex == 100000)
	{
		highIndex = table.size() - 1;
		interpolatedValue = table[highIndex][1];
	}
	else
	{
		interpolatedValue = table[lowIndex][1] + ((tableInput - table[lowIndex][0]) * ((table[highIndex][1] - table[lowIndex][1]) / (table[highIndex][0] - table[lowIndex][0])));
	}
	return interpolatedValue;
}

// GB.
double biLinearInterpolationWithBoundedBorders(std::vector<std::vector<double>> table, double tableRowInput, double tableColumnInput)
{
	int lowRowIndex = -100000;
	int highRowIndex = 100000;
	int lowColIndex = -100000;
	int highColIndex = 100000;
	int rows = table.size() - 1;
	int cols = table[0].size() - 1;
	double interpolatedValue;
	for (int i = 1; i <= rows; i++)
	{
		double refValue = table[i][0];
		if (refValue >= tableRowInput)
		{
			if (highRowIndex == 100000)
			{
				highRowIndex = i;
			}
			else if (refValue < table[highRowIndex][0])
			{
				highRowIndex = i;
			}
		}
		else if (refValue <= tableRowInput)
		{
			if (lowRowIndex == -100000)
			{
				lowRowIndex = i;
			}
			else if (refValue > table[lowRowIndex][0])
			{
				lowRowIndex = i;
			}
		}
	}
	if (lowRowIndex == -100000)
	{
		lowRowIndex = 1;
	}
	if (highRowIndex == 100000)
	{
		highRowIndex = rows;
	}
	for (int i = 1; i <= cols; i++)
	{
		double refValue = table[0][i];
		if (refValue >= tableColumnInput)
		{
			if (highColIndex == 100000)
			{
				highColIndex = i;
			}
			else if (refValue < table[0][highColIndex])
			{
				highColIndex = i;
			}
		}
		else if (refValue <= tableColumnInput)
		{
			if (lowColIndex == -100000)
			{
				lowColIndex = i;
			}
			else if (refValue > table[0][lowColIndex])
			{
				lowColIndex = i;
			}
		}
	}
	if (lowColIndex == -100000)
	{
		lowColIndex = 1;
	}
	if (highColIndex == 100000)
	{
		highColIndex = cols;
	}

	if (lowRowIndex == highRowIndex and lowColIndex != highColIndex)
	{
		double x = tableColumnInput;
		double x1 = table[0][lowColIndex];
		double x2 = table[0][highColIndex];
		double y1 = table[highRowIndex][lowColIndex];
		double y2 = table[highRowIndex][highColIndex];
		interpolatedValue = y1 + (x - x1) * ((y2 - y1) / (x2 - x1));
	}
	else if (lowRowIndex != highRowIndex and lowColIndex == highColIndex)
	{
		double x = tableRowInput;
		double x1 = table[lowRowIndex][0];
		double x2 = table[highRowIndex][0];
		double y1 = table[lowRowIndex][highColIndex];
		double y2 = table[highRowIndex][highColIndex];
		interpolatedValue = y1 + (x - x1) * ((y2 - y1) / (x2 - x1));
	}
	else if (lowRowIndex == highRowIndex and lowColIndex == highColIndex)
	{
		interpolatedValue = table[highRowIndex][highColIndex];
	}
	else
	{
		double x1 = table[lowRowIndex][0];
		double x2 = table[highRowIndex][0];
		double y1 = table[0][lowColIndex];
		double y2 = table[0][highColIndex];
		double corner11 = table[lowRowIndex][lowColIndex];
		double corner12 = table[lowRowIndex][highColIndex];
		double corner21 = table[highRowIndex][lowColIndex];
		double corner22 = table[highRowIndex][highColIndex];
		double t1 = corner11 * (x2 - tableRowInput) * (y2 - tableColumnInput);
		double t2 = corner21 * (tableRowInput - x1) * (y2 - tableColumnInput);
		double t3 = corner12 * (x2 - tableRowInput) * (tableColumnInput - y1);
		double t4 = corner22 * (tableRowInput - x1) * (tableColumnInput - y1);
		interpolatedValue = (t1 + t2 + t3 + t4) / ((x2 - x1) * (y2 - y1));
	}
	return interpolatedValue;
}
