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
#ifndef BLOCK_H
#define BLOCK_H
#define ACCESS_FN(T,N) T N##_(){return N;} 

#include "state.h"
#include "state_euler.h"
#include "state_rk2.h"
#include "state_mm.h"
#include <vector>
using namespace std;

class Block {
  public:
    Block();
    virtual ~Block() {};
    virtual void init() {};
    virtual void update() {};
    virtual void rpt() {};
    vector<State*> vState;
    void propagateStates();
    friend class Sim;
    void addIntegrator( double &x, double &xd);
    static State *integrator;
  protected:
    int initCount;
};

#include "sim.h"

#endif
