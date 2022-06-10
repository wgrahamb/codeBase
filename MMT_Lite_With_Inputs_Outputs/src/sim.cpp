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
#include "sim.h"
#include <iostream>
int Sim::stop;
int Sim::stop0;

Sim::Sim( double *dts, double tmax, vector< vector<Block *> > &vStage) {
	this->dt = dt;
	this->tmax = tmax;
	this->vStage = vStage;
	this->dts = dts;
}

void Sim::run( void) {
	State::set();
	Sim::stop = Sim::stop0 = 0;

	// INITIATION
	vector< vector<Block*> >::iterator stage, stage0;
	vector<Block*>::iterator obj;
	for( stage = vStage.begin(); stage != vStage.end(); stage++)
	{
		for( obj = stage->begin(); obj != stage->end(); obj++)
		{
			(*obj)->initCount = 0;
		}
	}

	// UPDATE
	int ii = 0;
	for( stage = vStage.begin(); stage != vStage.end(); stage++)
	{
		State::dt = this->dts[ii];
		State::reset( this->dts[ii]);
		ii++;
		stage0 = stage;
		State::tickfirst = 1;

		// TURN THIS INTO AN INIT FUNCTION
		for( obj = stage->begin(); obj != stage->end(); obj++)
		{
			(*obj)->init();
			(*obj)->initCount++;
		}


		while( true)
		{
			// TURN THIS INTO AN UPDATE FUNCTION
			State::sample( State::EVENT, tmax);
			for( obj = stage->begin(); obj != stage->end(); obj++)
			{
				(*obj)->update();
			}
			if( State::ready)
			{
				for( obj = stage->begin(); obj != stage->end(); obj++)
				{
					(*obj)->rpt();
				}
				State::tickfirst = 0;
				if( Sim::stop != Sim::stop0)
				{
					Sim::stop0 = Sim::stop;
					break;
				}
				if( State::t + State::EPS >= tmax)
				{
					Sim::stop = -1;
					break;
				}
			}
			
			for( obj = stage->begin(); obj != stage->end(); obj++)
			{
				(*obj)->propagateStates();
			}
			( *(stage->begin()))->integrator->updateclock();
		}
		if( Sim::stop < 0)
		{
			break;
		}
	}

	// LAST REPORT
	State::ticklast = 1;
	for( obj = stage0->begin(); obj != stage0->end(); obj++)
	{
		(*obj)->rpt();
	}
	return;
}


