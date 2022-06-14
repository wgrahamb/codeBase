#include "iostream"
#include "iomanip"
#include "math.h"

#include "actuators.h"
#include "System.h"
#include "Output.h"
#include "Filer.h"

actuators::actuators(string inFile, Output *outp, System *sysp)
{

	string file = "input/" + inFile;
	Filer *ff = new Filer(file);
	fidelityFlag = ff->getInt("fidelityFlag");
	delete ff;

	cout << "SECOND ORDER ACTUATOR CONSTRUCTED" << endl;

	out = outp;
	sys = sysp;

}

void actuators::init()
{

	fn = 40.0; // HZ
	zeta = 0.7; // ND
	del_inc = 0.125; // DEGREES

	del1c = defl1 = x1_1 = x2_1 = x1d_1 = x2d_1 = 0.0;
	del2c = defl2 = x1_2 = x2_2 = x1d_2 = x2d_2 = 0.0;
	del3c = defl3 = x1_3 = x2_3 = x1d_3 = x2d_3 = 0.0;
	del4c = defl4 = x1_4 = x2_4 = x1d_4 = x2d_4 = 0.0;

	wn_1 =  fn * 2.0 * PI;
	wn_2 =  fn * 2.0 * PI;
	wn_3 =  fn * 2.0 * PI;
	wn_4 =  fn * 2.0 * PI;
	//
	zeta_1 = zeta;
	zeta_2 = zeta;
	zeta_3 = zeta;
	zeta_4 = zeta;

}

void actuators::handleInput(
	NavigationState const &navigationState,
	double rollFinCommandDegrees,
	double pitchFinCommandDegrees,
	double yawFinCommandDegrees,
	double mach,
	double q
)
{
	rollFinCommandInDegrees = rollFinCommandDegrees;
	pitchFinCommandInDegrees = pitchFinCommandDegrees;
	yawFinCommandInDegrees = yawFinCommandDegrees;
	machSpeed = mach;
	dynamicPressure = q;

	rollAngle = navigationState.missileLTFEulerAngles_[0];

	Vecff euler;
	euler.x = navigationState.missileLTFEulerAngles_[0];
	euler.y = navigationState.missileLTFEulerAngles_[1];
	euler.z = navigationState.missileLTFEulerAngles_[2];
	
	Matff dcm = euler.getDCM();

	Vecff missileLocalVelocity;
	missileLocalVelocity.x = navigationState.missileLTFVelocity_[0];
	missileLocalVelocity.y = navigationState.missileLTFVelocity_[1];
	missileLocalVelocity.z = navigationState.missileLTFVelocity_[2];

	Vecff missileBodyVelocity = dcm * missileLocalVelocity;

	double temp = sqrt(missileBodyVelocity.y * missileBodyVelocity.y + missileBodyVelocity.z * missileBodyVelocity.z);
	if (missileBodyVelocity.x == 0.0)
	{
		angleOfAttack = 0.0;
	}
	else
	{
		angleOfAttack = atan2(temp, missileBodyVelocity.x) * rtd;
	}

}

void actuators::update()
{

	double del1c, del2c, del3c, del4c;
	double del1b, del2b, del3b, del4b;
	del1b = -1 * pitchFinCommandInDegrees + yawFinCommandInDegrees + rollFinCommandInDegrees;
	del2b = -1 * pitchFinCommandInDegrees - yawFinCommandInDegrees + rollFinCommandInDegrees;
	del3b =  pitchFinCommandInDegrees - yawFinCommandInDegrees + rollFinCommandInDegrees;
	del4b =  pitchFinCommandInDegrees + yawFinCommandInDegrees + rollFinCommandInDegrees;

	// find max deflection and then normalize to del_max, if req'd
	double delb_max = 0.0;
	double del_max = 7.0;
	double aa;
	if( fabs( del1b) > delb_max) delb_max = fabs( del1b);
	if( fabs( del2b) > delb_max) delb_max = fabs( del2b);
	if( fabs( del3b) > delb_max) delb_max = fabs( del3b);
	if( fabs( del4b) > delb_max) delb_max = fabs( del4b);
	if( delb_max > del_max)
	{
		aa = del_max / delb_max;
	}
	else
	{
		aa = 1.0;
	}
	del1c =  del1b * aa;
	del2c =  del2b * aa;
	del3c =  del3b * aa;
	del4c =  del4b * aa;

	// ACTUATORS
	double wn_1, wn_2, wn_3, wn_4;
	double zeta_1, zeta_2, zeta_3, zeta_4;
	double del_inc = 0.125;
	double fn = 40;
	double zeta = 0.7;
	wn_1 = wn_2 = wn_3 = wn_4 = fn * 2.0 * PI * 1.0;
	zeta_1 = zeta_2 = zeta_3 = zeta_4 = zeta;

	// CANARD ONE
	u1_1  = int((fabs(del1c) + del_inc / 2.0) / del_inc) * del_inc * signum(del1c);
	double x1_1_new = integrate(x2_1, x1d_1, x1_1, 0.001);
	x1d_1 = x2_1;
	x1_1 = x1_1_new;
	double temp = (u1_1 - x1_1) * wn_1 * wn_1 - 2.0 * zeta_1 * wn_1 * x2_1;
	double x2_1_new = integrate(temp, x2d_1, x2_1, 0.001);
	x2d_1 = temp;
	x2_1 = x2_1_new;
	defl1  = x1_1;

	// CANARD TWO
	u1_2  = int( ( fabs( del2c) + del_inc / 2.0) / del_inc) * del_inc * signum(del2c);
	double x1_2_new = integrate(x2_2, x1d_2, x1_2, 0.001);
	x1d_2 = x2_2;
	x1_2 = x1_2_new;
	temp = ( u1_2 - x1_2) * wn_2 * wn_2 - 2.0 * zeta_2 * wn_2 * x2_2;
	double x2_2_new = integrate(temp, x2d_2, x2_2, 0.001);
	x2d_2 = temp;
	x2_2 = x2_2_new;
	defl2 = x1_2;

	// CANARD THREE
	u1_3  = int( ( fabs( del3c) + del_inc / 2.0) / del_inc) * del_inc * signum(del3c);
	double x1_3_new = integrate(x2_3, x1d_3, x1_3, 0.001);
	x1d_3 = x2_3;
	x1_3 = x1_3_new;
	temp = ( u1_3 - x1_3) * wn_3 * wn_3 - 2.0 * zeta_3 * wn_3 * x2_3;
	double x2_3_new = integrate(temp, x2d_3, x2_3, 0.001);
	x2d_3 = temp;
	x2_3 = x2_3_new;
	defl3  = x1_3;

	// CANARD FOUR
	u1_4  = int( ( fabs( del4c) + del_inc / 2.0) / del_inc) * del_inc * signum(del4c);
	double x1_4_new = integrate(x2_4, x1d_4, x1_4, 0.001);
	x1d_4 = x2_4;
	x1_4 = x1_4_new;
	temp = ( u1_4 - x1_4) * wn_4 * wn_4 - 2.0 * zeta_4 * wn_4 * x2_4;
	double x2_4_new = integrate(temp, x2d_4, x2_4, 0.001);
	x2d_4 = temp;
	x2_4 = x2_4_new;
	defl4  = x1_4;

}

double actuators::signum(double x)
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

double actuators::integrate(double dy_new, double dy, double y, double intStep)
{
	return y + ((dy_new + dy) * intStep / 2);
}

double actuators::uniform()
{
	double value;
	value=(double)rand()/RAND_MAX;
	return value;
}