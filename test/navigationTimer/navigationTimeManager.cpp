#include "navigationTimeManager.h"
#include "math.h"

navigationTimeManager::navigationTimeManager()
{
	navigationTimer = 0.0;
	variableNavigationTimeStep = 1 / 2500.0;
	CONSTANT_NAVIGATION_TIME_STEP = variableNavigationTimeStep;
}

int navigationTimeManager::sample(double sampleTimeStep, double sampleEventTime)
{

	if (sampleTimeStep < 0.0)
	{

		if (sampleEventTime < (endSampleTime - EPS) && sampleEventTime >= (navigationTimer + EPS))
		{
			endSampleTime = sampleEventTime;
		}

		variableNavigationTimeStep = endSampleTime - navigationTimer;

		if (fabs(sampleEventTime - navigationTimer) < EPS)
		{
			return 1;
		}
		else
		{
			return 0;
		}

	}

	double tempEndSampleTime = (floor(((navigationTimer + EPS) / sampleTimeStep) + 1)) * sampleTimeStep;

	if (tempEndSampleTime < (endSampleTime - EPS))
	{
		endSampleTime = tempEndSampleTime;
	}

	variableNavigationTimeStep = endSampleTime - navigationTimer;

	if ((navigationTimer - tempEndSampleTime + sampleTimeStep) < EPS)
	{
		return 1;
	}
	else
	{
		return 0;
	}

}

void navigationTimeManager::updateClock()
{
	navigationTimer = endSampleTime;
	endSampleTime = (floor(((navigationTimer + EPS) / CONSTANT_NAVIGATION_TIME_STEP) + 1)) * CONSTANT_NAVIGATION_TIME_STEP;
}