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
void divideVectorTimesScalar(double scalar, double vec[3], double out[3]);
double linearInterpolationWithBoundedEnds(std::vector<std::vector<double>> table, double tableInput);
double biLinearInterpolationWithBoundedBorders(std::vector<std::vector<double>> table, double tableRowInput, double tableColumnInput);

struct mslDataPacket
{
	double mslTimeStep;
	double mslIntegrationStep;
	double mslMaxTime;
	double mslTof;

	double mslDynamicPressure;
	double mslSpeedOfSound;
	double mslTotalPressure;

	// ENGINE OUTPUT
	double mslLocalOrient[3][3];
	double mslPos[3];
	double mslRange;
	double mslVel[3];
	double mslBodyVel[3];
	double mslSpeed;
	double mslMach;
	double mslAcc[3];
	double mslBodyAcc[3];
	double mslEuler[3];
	double mslEulerDot[3];
	double mslRate[3];
	double mslRateDot[3];
	double mslAlpha;
	double mslBeta;

	// ENGINE INPUT
	double mslWayPoint[3];
	double forwardLeftUpMslToIntercept[3];

	double CX;
	double CY;
	double CZ;
	double CL;
	double CN;
	double CM;
	double mslMass;
	double mslTMoi;
	double mslAMoi;
	double mslThrust;
	double mslRefArea;
	double mslRefDiam;

	double missDistance;
	bool fly;
	std::string lethality;
};

#endif