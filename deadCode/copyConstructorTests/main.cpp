#include "iostream"
#include "iomanip"

// Only purpose is to hold a number.
struct value
{
	public:
		double value;
};

// Class to be copied.
struct simpleMath
{

	public:

		// Normal constructor that takes a pointer to a "value" as an input.
		simpleMath(value *inputValueStructure) : fxnValue(inputValueStructure) {}

		// Defined copy constructor.
		simpleMath(simpleMath& newObject)
		{
			timer = newObject.timer;
			timeStep = newObject.timeStep;
			*fxnValue = *(newObject.fxnValue);
		}

		// Object timer.
		double timer;

		// Object time step to iterate by.
		double timeStep;

		// Pointer to "value."
		value *fxnValue;

		// Simple mathematic function.
		void fxn()
		{

			// Iterate timer.
			timer += timeStep;

			// Update value.
			fxnValue->value += (timer * timer);

		}

};

int main()
{

	// Value one.
	value objOneValue;
	objOneValue.value = 0.0;

	// Simple math object one.
	simpleMath objOne(&objOneValue);
	objOne.timer = 0.0;
	objOne.timeStep = 0.1;

	// Loop through function.
	for (int i = 0; i < 100; i++)
	{
		objOne.fxn();
	}

	// Default copy constructor.
	// The pointer to "value" in object two is the exact same as the pointer in object one.
	// So, when the value is updated by the function in object one, it is reflected in both object one and object two.
	// However, the timer of each object is local to the object, so the timer of object one will update with the function but the timer of object two will not.

	// Defined copy constructor.
	// Because the copy constructor is defined, the memory allocated to the "value" pointer is also copied.
	// This means that the value of object one and object two can be updated without interfering with each other.
	simpleMath objTwo = objOne;

	// Loop through function.
	for (int i = 0; i < 100; i++)
	{
		objOne.fxn();
	}

	// Loop through function.
	for (int i = 0; i < 1000; i++)
	{
		objTwo.fxn();
	}

	return 0;
}