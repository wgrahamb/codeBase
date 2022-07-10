

class navigationTimeManager
{

	public:

		// Methods.
		navigationTimeManager();
		int sample(double sampleTimeStep, double sampleEventTime);
		void updateClock();

		// Variables
		double navigationTimer;
		double variableNavigationTimeStep;
		double CONSTANT_NAVIGATION_TIME_STEP;
		double endSampleTime;

		// Constant.
		const double EPS = 1e-8;

};