#include "math.h"
#include "iostream"
#include "iomanip"

using namespace std;

// Input is the three euler angles.
// Phi = roll angle in radians.
// Theta = pitch angle in radians.
// Psi = yaw angle in radians.
// Returns a direciton cosine matrix.
void eulerAnglesToLocalOrientation (double phi, double theta, double psi, double matrix[3][3]) {
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

// Forward multiplication.
void threeByThreeTimesThreeByOne(double matrix[3][3], double arr[3], double out[3]) {
	out[0] = matrix[0][0] * arr[0] + matrix[0][1] * arr[1] + matrix[0][2] * arr[2];
	out[1] = matrix[1][0] * arr[0] + matrix[1][1] * arr[1] + matrix[1][2] * arr[2];
	out[2] = matrix[2][0] * arr[0] + matrix[2][1] * arr[1] + matrix[2][2] * arr[2];
}

// Backwards multiplication
void oneByThreeTimesThreeByThree(double arr[3], double matrix[3][3], double out[3])
{
	out[0] = matrix[0][0] * arr[0] + matrix[1][0] * arr[1] + matrix[2][0] * arr[2];
	out[1] = matrix[0][1] * arr[0] + matrix[1][1] * arr[1] + matrix[2][1] * arr[2];
	out[2] = matrix[0][2] * arr[0] + matrix[1][2] * arr[1] + matrix[2][2] * arr[2];
}

// USE CASES
int main()
{

	// Missile.
	double mslENUPosition[3], mslENUEulerAngles[3];
	mslENUPosition[0] = 0.0;
	mslENUPosition[1] = 0.0;
	mslENUPosition[2] = 0.0;
	mslENUEulerAngles[0] = 0.1;
	mslENUEulerAngles[1] = 0.45;
	mslENUEulerAngles[2] = 0.1;

	// Target
	double tgtENUPosition[3];
	tgtENUPosition[0] = 3000.0;
	tgtENUPosition[1] = 3000.0;
	tgtENUPosition[2] = 3000.0;

	// Relative position
	double ENUMissileToTargetRelativePosition[3];
	ENUMissileToTargetRelativePosition[0] = tgtENUPosition[0] - mslENUPosition[0];
	ENUMissileToTargetRelativePosition[1] = tgtENUPosition[1] - mslENUPosition[1];
	ENUMissileToTargetRelativePosition[2] = tgtENUPosition[2] - mslENUPosition[2];

	cout << "\n";
	cout << "RELATIVE POSITION IN ENU X: " << ENUMissileToTargetRelativePosition[0] << " Y: " << ENUMissileToTargetRelativePosition[1] << " Z: " << ENUMissileToTargetRelativePosition[2] << endl;

	// Create DCM.
	double ENUToFLUMatrix[3][3];
	eulerAnglesToLocalOrientation(mslENUEulerAngles[0], -1.0 * mslENUEulerAngles[1], mslENUEulerAngles[2], ENUToFLUMatrix);

	// Transform ENU to FLU.
	double FLUMissileToTargetRelativePosition[3];
	threeByThreeTimesThreeByOne(ENUToFLUMatrix, ENUMissileToTargetRelativePosition, FLUMissileToTargetRelativePosition);

	cout << "RELATIVE POSITION IN FLU X: " << FLUMissileToTargetRelativePosition[0] << " Y: " << FLUMissileToTargetRelativePosition[1] << " Z: " << FLUMissileToTargetRelativePosition[2] << endl;

	// Test, transform FLU to ENU.
	double testENUMissileToTargetRelativePosition[3];
	oneByThreeTimesThreeByThree(FLUMissileToTargetRelativePosition, ENUToFLUMatrix, testENUMissileToTargetRelativePosition);

	cout << "TEST, RELATIVE POSITION IN ENU X: " << testENUMissileToTargetRelativePosition[0] << " Y: " << testENUMissileToTargetRelativePosition[1] << " Z: " << testENUMissileToTargetRelativePosition[2] << endl;
	cout << "\n";

	return 0;
}