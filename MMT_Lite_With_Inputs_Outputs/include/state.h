// v50207, by Ray Sells, DESE Research, Inc.
/*
----------------
This OPEN SOURCE CODE is part of the Object-Oriented Simulation Kernel (OSK)
created by

                Ray Sells, DESE Research, Inc., 2003.

                        All Rights Reserved.

The OSK consists of state.h, state.cpp, block.h, block.cpp, sim.h, and sim.cpp.

Permission to use, copy, modify, and distribute the OSK software and its  
documentation for any purpose and without fee is hereby granted,  
provided that this notice appears in all copies and supporting documentation.

DESE RESEARCH, INC. AND RAY SELLS DISCLAIM ALL WARRANTIES WITH REGARD
TO THIS SOFTWARE, INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
AND FITNESS, IN NO EVENT SHALL DESE RESEARCH, INC. OR RAY SELLS
BE LIABLE FOR ANY SPECIAL, INDIRECT OR CONSEQUENTIAL DAMAGES OR ANY
DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, 
WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
SOFTWARE, EVEN IF THE SOFTWARE IS USED FOR ITS INTENDED PURPOSE.
----------------
*/
#ifndef STATE_H
#define STATE_H
#include <iostream>
using namespace std;

class State {
  public:
    State() {};
    State( double &x, double &xd);
    static  double  dt, dtp;
    static  double  t, t1;
    static  long    inc;
    static  int     tickfirst, ticklast;
    static  bool    ready;
    static  const double EVENT;
    static  const double EPS, del;

    static void set();
    static void reset( double dtp);
    static int sample();
    static int sample( double dt, double t_event = 0);
    virtual void propagate();
    virtual void updateclock();
    virtual State *factory( double &x, double &xd) {
      return new State( x, xd);
    }

  protected:
    double  x0;
    double  *x, *xd;
    double  xd0, xd1, xd2, xd3;
    static  int     kpass;

};

#endif
