
Use:
- Each individual project is configured to run from this directory level.
- All PY scripts require a python interpreter and a few pip installs.
Start with numpy, pandas, and matplotlib.
- All CPP projects are configured by CMAKE and require a compiler.
They should only require a "cmake" from the command line, and then your preferred
method of building. This includes the CADAC_SIMULATIONS.

Folder Structure:

	3DOFS - Swap cpp base model for model from pipselection. Check rotating earth.

	Includes multiple three degree of freedom models. Some 3DOF translational and
	some 3DOF side scrollers. Has a base cpp model and a base python model intended
	for development and rapid prototype test beds.


	CADAC_SIMULATIONS - Convert to CMake projects.

	# REFERENCE - MODELING AND SIMULATION OF AEROSPACE
	# VEHICLE DYNAMICS SECOND EDITON - PETER H. ZIPFEL
	This is the source code provided with the textbook listed above. It is written by
	Peter H. Zipfel. It is a very good resource, written to help students, such
	as myself, learn the complexities of modeling.


	CPP_6DOF_70MM_ROCKET - In development.

	I found this paper, released by the Government for public use, early in my
	career. It gives enough data of a 70MM HYDRA rocket to model its dynamics.
	I have not started this project, but the paper and its transcribed data are included.


	CPP_6DOF_SAM - In development.

	This will be a partial port from CADAC_SIMULATIONS ADS6. This is a surface-
	to-air missile. It will start as a Dynamics engine, to determine performance
	of the missile. It may stay that way. I may also write a simple shell around it
	to use in modeling environments. I have not started this project.


	CPP_6DOF_SRAAM_V1

	This is the very first six degree of freedom missile model I wrote, in CPP. I am fond of it.
	So, it will live here forever. It is not very good code or modeling. It does however hit its
	target, and that doesn't happen on accident. It is also runs very fast and is good for batch runs.
	Included are scripts to run the model.


	CPP_6DOF_SRAAM_V2 - See main for to do list.

	This is a cleaned version of CPP_6DOF_SRAAM_V1, written to be easy to integrate
	into other environments. It has a different motion model than V1, one that allows
	it to fly ballistically. The motion model used in V1 tends to harsh alpha rates (it could be
	wrong), while this one stabilizes. It also has a redesigned, simpler, and more barbaric
	control theory. The utility in V1 has been moved to a seperate file. The actuators have been 
	re-written and componentized. There are three methods of integration. Euler, RK2, and
	RK4. The logging and visual contain higher fidelity. It runs pretty fast, but not as
	fast as V1. Also included is a way to fly the missile in a 3DOF motion model, very
	useful for predictor models. There is a use-case provided in the main function.


	cppFunctions

	Includes multiple cpp functions for modeling, some written by me, some written by Zipfel.
	It also includes any stand alone classes or functions that I find useful. It will build as its
	own project.asl;


	PY_5DOF_AIM
	PY_5DOF_MOCK_HELLFIRE_DYNAMICS - In development.
	PY_6DOF_SRAAM
	pythonFunctions






















