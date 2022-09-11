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
#include <dirent.h>

// Namespace.
using namespace std;

// Header.
#include "randomNumbers.h"

// Constants
#define IA 16807
#define IM 2147483647
#define AM (1.0/IM)
#define IQ 127773
#define IR 2836
#define MASK 123459876

randomNumbers::randomNumbers(){ cout << "RANDOM NUMBERS CONSTRUCTED\n"; }

// Set initial seed once per run.
void randomNumbers::init(long seed)
{
     SEED = seed;
     RUN_NUM += 1;
     cout << "RUN NUMBER " << RUN_NUM << ", SEED " << SEED << endl;
}

void randomNumbers::writeDraws(string relFilePath)
{

     ofstream logfile;
     logfile.open(relFilePath);
     logfile << "ID DRAW_TYPE DRAW_VALUE\n";
     int count = DRAWS.size();
     for (int i = 0; i < count; i++)
     {
          logfile << fixed <<
          DRAWS[i].ID << " " <<
          DRAWS[i].DRAW_TYPE << " " <<
          DRAWS[i].DRAW_VALUE << endl;
     }

}

float randomNumbers::makeUniformDraw(string ID, float min, float max)
{

     float value = uniformDistribution(min, max);

     DRAW DRAW_NEW;
     DRAW_NEW.ID = ID;
     DRAW_NEW.DRAW_TYPE = "UNIFORM";
     DRAW_NEW.DRAW_VALUE = to_string(value);
     DRAWS.push_back(DRAW_NEW);

     return value;

}

float randomNumbers::makeNormalDraw(string ID, float mean, float stdev)
{

     float value = normalDistribution(mean, stdev);

     DRAW DRAW_NEW;
     DRAW_NEW.ID = ID;
     DRAW_NEW.DRAW_TYPE = "NORMAL";
     DRAW_NEW.DRAW_VALUE = to_string(value);
     DRAWS.push_back(DRAW_NEW);

     return value;

}

float randomNumbers::makeUniformNoise(float min, float max)
{
     float ret = uniformDistribution(min, max);
     return ret;
}

float randomNumbers::makeNormalNoise(float mean, float stdev)
{
     float ret = normalDistribution(mean, stdev);
     return ret;
}

///////////////////////////////////////////////////////////////////////////////
// Ref Numerical Recipies, Second Edition. page 289
///////////////////////////////////////////////////////////////////////////////
float randomNumbers::randomNumberGenerator()
{

     long k;
     float ans;
     SEED ^= MASK;
     k=(SEED)/IQ;
     SEED=IA*(SEED-k*IQ)-IR*k;
     if (SEED < 0) SEED += IM;
     ans=AM*(SEED);
     SEED ^= MASK;
     return ans; // Returns a number between 0.0 and 1.0.

}

///////////////////////////////////////////////////////////////////////////////
// Generating uniform random distribution between 'min' and 'max' 
// 010913 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
float randomNumbers::uniformDistribution(float min, float max)
{

     double value;
     value=min+(max-min)*randomNumberGenerator();
     return value;

}

///////////////////////////////////////////////////////////////////////////////
// Generating a standard distribution with 'mean' and 'sig' std deviation
// Ref Numerical Recipies, p 289, 1992 Cambridge University Press
// parameter input:
// stdev = standard deviation of Gaussian distribution - unit of variable
// mean = mean value of Gaussian distribution - unit of variable
// 010913 Created by Peter H Zipfel
///////////////////////////////////////////////////////////////////////////////
float randomNumbers::normalDistribution(float mean, float stdev)
{
     static int iset=0;
     static double gset;
     double fac,rsq,v1,v2;
     float value;
     if(iset==0)
     {
          do
          {
               v1=2.*randomNumberGenerator()-1.;
               v2=2.*randomNumberGenerator()-1.;
               rsq=v1*v1+v2*v2;
          }while(rsq>=1.0||rsq==0);
          fac=sqrt(-2.*log(rsq)/rsq);
          gset=v1*fac;
          iset=1;
          value=v2*fac;
     }
     else
     {
          iset=0;
          value=gset;
     }
     return value*stdev+mean;
}