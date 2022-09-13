
Use:
- Each individual project is configured to run from this directory level.
- All PY scripts require a python interpreter and a few pip installs.
Start with numpy, pandas, and matplotlib.  
- All CPP projects are configured by CMAKE and require a compiler.
They should only require a "cmake" from the command line, and then your preferred
method of building. This includes the CADAC_SIMULATIONS.

Folder Structure: - Go through each project and make sure any executable can be made
                           from this directory. Write python scripts to automate the creation
                           of build folders.

	3DOFS - add an objective cpp 3dof base.

	Includes multiple three degree of freedom models. Some 3DOF translational and
	one 3DOF side scroller. Has a base CPP model and a base python model intended
	for development and rapid prototype test beds.


	CADAC_SIMULATIONS - Convert to CMake projects.

	REFERENCE: Modeling and Simulation of Aerospace Vehicle Dynamics,
	Second Edition, Peter H. Zipfel.

	This is the source code provided with the textbook listed above. It is written by
	Peter H. Zipfel. It is a very good resource, written to help students such
	as myself, learn the complexities of modeling.


	CPP_6DOF_70MM_ROCKET - In development.

	I found this paper early in my career. It was released by the Government for public use.
	It gives enough data of a 70MM HYDRA rocket to model its dynamics. I have not
	started this project, but the paper and its transcribed data are included.


	CPP_6DOF_SAM - In development.

	This will be a partial port from CADAC_SIMULATIONS ADS6. This is a surface-
	to-air missile. It will start as a Dynamics engine, to determine performance
	of the missile. It may stay that way. I may also write a simple shell around it
	to use in modeling environments. I have not started this project.


	CPP_6DOF_SRAAM_V1 - Convert to CMAKE and clean directory.

	This is a port from CADAC_SIMULATIONS SRAAM6, Seemingly based on a Sidewinder,
	this is the very first CPP six degree of freedom missile model I wrote. I am fond
	of it. So, it will live here forever. It is not very good code or modeling. It does however
	hit its target, and that doesn't happen on accident. It also runs very fast and is good
	for batch runs.  Included are scripts to run the model.


	CPP_6DOF_SRAAM_V2 - See main for to do list.

	This is a cleaned version of CPP_6DOF_SRAAM_V1, written to be easy to integrate
	into other environments. It has a different motion model than V1, one that allows
	it to fly ballistically. The motion model used in V1 tends to harsh alpha rates (it could be
	wrong), while this one stabilizes. It also has a simpler and more barbaric
	control theory. The utility in V1 has been moved to a seperate file. The actuators have been 
	re-written and componentized. There are three methods of integration. Euler, RK2, and
	RK4. The logging and visual contain higher fidelity. It runs pretty fast, but not as
	fast as V1. Also included is a way to fly the missile in a 3DOF motion model, very
	useful for predictor models. There is a use-case provided in the main function.


	cppFunctions

	Includes multiple CPP functions for modeling, some written by me, some written by Zipfel.
	It also includes any stand alone classes or functions that I find useful. It will build as its
	own project.


	PY_5DOF_AIM

	This is a paired down port from CADAC_SIMULATIONS. It is a bit of a kludgy model,
	but it does work and runs very fast. It would be very easy to integrate into other environments.
	I really don't know what real life missile this is supposed to represent, but based
	on the reference characteristics, it may be designed after the old British
	Taildog missile, also called S.R.A.A.M. or Short Range Air-to-Air Missile. 


	PY_5DOF_MOCK_HELLFIRE - In development.

	REFERENCE: Tactical And Strategic Missile Guidance, Third Edition, Paul Zarchan.

	Using Zarchan's method for linearizing an airframe, I made a sketch of a mock 
	Hellfire and used those dimensions for calculations. Using those estimations for 
	the aerodynamics of the airframe, I then wrote a simple script to represent the 
	mass and motor properties of tbe mock Hellfire. I then wrapped these things in a 
	Dynamics constructor and a Dynamics flight simulator. This yielded a
	fast and portable mock Hellfire Dynamics engine. It takes a flight duration and 
	fin deflections as an input.


	PY_6DOF_SRAAM

	This is the python port of the CADAC_SIMULATIONS SRAAM6. I wrote this
	sim before the CPP version. It has gone through a fair amount of revision since its
	beginning. It uses the same motion model as CPP_6DOF_SRAAM_V1. It runs 
	pretty fast considering it is a good bit of code. It would be very easy to integrate
	into other environments.


	pythonFunctions

	Includes multiple python functions for modeling. Most of them are written by me,
	but some of it comes from elsewhere.






















