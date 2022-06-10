// Version 6a000, by Ray Sells, DESE Research, Inc., 2007-03-06
#ifndef TFATM62_H
#define TFATM62_H
#include "main.h"


class TFAtm62 : public Block {
  public:
    TFAtm62();
    void update( double h);
    void test();
    double rho_() { return rho;};
    double vs_() { return vs;};
    double p_() { return press;};
    double tm_() { return temp;};
  protected:
    double rho, vs, press, temp;
};

#endif

