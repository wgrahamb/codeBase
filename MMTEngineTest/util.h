#include <vector>
#include <string>

#ifndef UTILITY_H
#define UTILITY_H

const double degToRad = 0.0174532925199432;
const double radToDeg = 57.2957795130823;
const double mToKm = 1 / 1000;
const double pi = 3.14159265358979323846;
const double grav = 9.81;

double signum(double x);
double atan2_0(double y, double x);
double exponentialDistribution(double density);
double gaussianDistribution(double mean, double sig);
double markovDistribution(double sigma,double bcor,double time,double intstep,double &value_saved);
double rayleighDistribution(double mode);
double uniformDistribution(double min,double max);
double unituni();
double integrate(double dy_new, double dy, double y, double intStep);
void flightPathAnglesToLocalOrientation (double azimuth, double elevation, double localFrame[3][3]);
void eulerAnglesToLocalOrientation (double phi, double theta, double psi, double matrix[3][3]);
void unitVec (double vector[3], double unitVector[3]);
void azAndElFromVector (double &az, double &el, double vector[3]);
void oneByThreeTimesThreeByThree(double arr[3], double matrix[3][3], double out[3]);
void threeByThreeTimesThreeByOne(double matrix[3][3], double arr[3], double out[3]);
void threeByThreeTimesThreeByThree(double mat1[3][3], double mat2[3][3], double out[3][3]);
void magnitude(double arr[3], double &out);
void subtractTwoVectors(double vec1[3], double vec2[3], double out[3]);
void addTwoVectors(double vec1[3], double vec2[3], double out[3]);
void multiplyTwoVectors(double vec1[3], double vec2[3], double out[3]);
void crossProductTwoVectors (double vec1[3], double vec2[3], double out[3]);
void dotProductTwoVectors (double vec1[3], double vec2[3], double &out);
void vectorProjection (double uv[3], double vec[3], double out[3]);
void multiplyVectorTimesScalar(double scalar, double vec[3], double out[3]);
double linearInterpolationWithBoundedEnds(std::vector<std::vector<double>> table, double tableInput);
double biLinearInterpolationWithBoundedBorders(std::vector<std::vector<double>> table, double tableRowInput, double tableColumnInput);

struct NAV_STATE
{
	public:
		double missileTimeOfFlight; // Seconds.
		double missileENUWayPoint[3]; //Meters.
		double missileENUPosition[3]; // Meters.
		double missileENUVelocity[3]; // Meters per second.
		double missileENUAcceleration[3]; // Meters per s^2.
		double missileENUEulerAngles[3]; // Radians.
		double missileBodyRate[3]; // Radians per second.
		double missileBodyRateDot[3]; // Radians per second^2.
};

#endif