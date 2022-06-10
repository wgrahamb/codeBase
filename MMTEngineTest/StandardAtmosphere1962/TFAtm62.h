// Version 6a000, by Ray Sells, DESE Research, Inc., 2007-03-06
#ifndef TFATM62_H
#define TFATM62_H

class TFAtm62
{

	public:
		TFAtm62();
		void test();
		void init();
		void update(double h, double speed);
		double rho, a, press, temp, q, mach;

};

#endif

