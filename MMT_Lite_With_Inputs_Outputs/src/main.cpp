#include "main.h"
#include "vector"
#include "chrono"

#include "Output.h"
#include "System.h"
#include "Util.h"
#include "PreFlight.h"
#include "MassProp.h"
#include "Motor.h"
#include "actuators.h"
#include "Atmos.h"
#include "Airframe.h"
#include "Aero.h"
#include "Motion.h"
#include "navigatorPerfect.h"
#include "NavProc.h"
#include "FlightProc.h"
#include "GuideLaw.h"
#include "MomContAuto.h"
#include "RollAuto.h"
#include "endCheck.h"

Output *out;
System *sys;
MathUtil *mutil;
PreFlight *preflt;

MassProp *mprop;
Motor *motor;
actuators *act;
Atmos *atm;
Aero *aero;
Motion *mot;
NavProc *navproc;
GuideLaw *guidelaw;
RollAuto *rollauto;
MomContAuto *momcontauto;

endCheck *ec;

typedef vector<Block*> vBlock;
vBlock objContainer;

void init()
{

	// INTEGRATOR CLASS
	Block::integrator = new State_euler();

	// CONSTRUCT OBJECTS
	out = new Output("sim.dat", sys); // COULD BE REMOVED DEPENDING ON ARCHITECTURE CHOICES, OTHERWISE GOOD
	sys = new System("sim.dat", out, mutil, navproc); // COULD BE REMOVED DEPENDING ON ARCHITECTURE CHOICES, OTHERWISE GOOD
	mutil = new MathUtil(sys); // HOPEFULLY BE ABLE TO DUMP
	preflt = new PreFlight(out, sys); // ADD IN TO SYS

	// DYNAMICS
	mprop = new MassProp("massprop.dat", out, sys);
	atm = new Atmos("atmos.dat", out, sys);
	motor = new Motor("motor.dat", out, sys);
	act = new actuators("actuators.dat", out, sys);
	aero = new Aero("aero.dat", out, sys);
	mot = new Motion("motion.dat", out, sys);

	// GNC
	navproc = new NavProc("navproc.dat", out, sys);
	guidelaw = new GuideLaw("guidelaw.dat", out, sys);
	rollauto = new RollAuto("rollauto.dat", out, sys);
	momcontauto = new MomContAuto("momcontauto.dat", out, sys);

	ec = new endCheck(sys, out, navproc);

	// RE ASSIGN POINTERS
	sys->reAssignPtr(mutil, navproc);
	out->reAssignPtr(sys);

}

int main()
{

	// FORMAT CONSOLE OUTPUT
	std::cout << "\n";

	// START WALL CLOCK
	auto wallClockStart = chrono::high_resolution_clock::now();

	// INITIATE SIMULATION
	init();

	// INITIALIZE OBJECT
	sys->init(); // OVER HEAD
	preflt->init(); // POINTLESS BUT INTEGRAL

	navproc->init();

	guidelaw->init();
	rollauto->init();
	momcontauto->init();
	mprop->init();
	atm->init();
	motor->init();
	act->init();
	aero->init();

	mot->init(); // END ENGINE

	out->init();
	ec->init();

	// PARAMETERS
	double tmax = 400.0;
	State::dt = sys->sys_dt;
	State::reset(sys->sys_dt);
	State::tickfirst = 1;
	double lastTime = 0;

	std::cout << "\n";
	std::cout << "FLIGHT" << std::endl;

	// RUN SIMULATION WITH OBJECTS LISTED
	while (ec->lethality == "FLYING")
	{
		// UPDATE ALL OBJECTS
		State::sample(State::EVENT, tmax);
		{

			sys->update(); // OVER HEAD
			preflt->update(); // POINTLESS BUT INTEGRAL

			if (sys->t_sys > 0.0)
			{
				double breakPoint = 0.0;
			}

			// START ENGINE

			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			/*

			To do:
			1) Set a constant nav state, capable of being referenced but not changed.
			2) Add a method to each module called "handleInputs" which receives a reference
			to the nav state and any other outputs from other modules.
			3) After done, remove bloat (navigator, navproc, flightproc).
			4) Try and hack the integrator. Since euler integration works, should be easy.
			5) Once the integrator is hacked, I am the captain now.

			*/
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Navigation state.
			double missileTimeOfFlight = sys->t_flight;
			// Translational state
			Eigen::Vector3d missileLTFPosition = {mot->ltf_pos.x, mot->ltf_pos.y, mot->ltf_pos.z};
			Eigen::Vector3d missileLTFVelocity = {mot->ltf_vel.x, mot->ltf_vel.y, mot->ltf_vel.z};
			Eigen::Vector3d missileBodyAcceleration = {mot->sf_b.x, mot->sf_b.y, mot->sf_b.z};
			// Rotational state
			Eigen::Vector3d missileLTFEulerAngles = {mot->eulerLTF.x, mot->eulerLTF.y, mot->eulerLTF.z};
			Eigen::Vector3d missileBodyRate = {mot->omegaB.x, mot->omegaB.y, mot->omegaB.z};
			Eigen::Vector3d missileBodyRateDot = {mot->omegaB_d.x, mot->omegaB_d.y, mot->omegaB_d.z};
			// Target
			Eigen::Vector3d missileWayPoint = {5000.0, 0.0, 2000.0};

			NavigationState navigationState(
				missileTimeOfFlight,
				missileLTFPosition,
				missileLTFVelocity,
				missileBodyAcceleration,
				missileLTFEulerAngles,
				missileBodyRate,
				missileBodyRateDot,
				missileWayPoint
			);

			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Navproc handle inputs.
			navproc->handleInput(navigationState);

			// Navproc Update. (Does nothing. leaving for now)
			navproc->update();
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Atmosphere handle inputs.
			atm->handleInput(navigationState);

			// Atmosphere Update.
			atm->update();

			// Atmoshpere Outputs.
			double atmosOutputPressure = atm->p;
			double atmosOutputDynamicPressure = atm->q;
			double atmosOutputMach = atm->amach;
			double atmosOutputAirTemp = atm->airTemp;
			double atmosOutputAirTempNominal = atm->airTemp_nom;
			Vec atmosOutputLTFWindVelocity = atm->vwind;
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Mass properties handle inputs.
			mprop->handleInput(navigationState);

			// Mass properties update.
			mprop->update();

			// Mass properties output.
			double mpropOutputXcg = mprop->xcg;
			double mpropOutputYcg = mprop->ycg;
			double mpropOutputZcg = mprop->zcg;
			double mpropOutputMass = mprop->mass;
			Mat mpropOutputInertiaTensor = mprop->iten;
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Motor handle input.
			motor->handleInput(
				navigationState,
				mpropOutputXcg,
				mpropOutputYcg,
				mpropOutputZcg,
				atmosOutputAirTemp,
				atmosOutputAirTempNominal,
				atmosOutputPressure
			);

			// Motor update.
			motor->update();

			// Motor outputs.
			double motorOutputThrust = motor->thrust;
			Vec motorOutputForce = motor->force;
			Vec motorOutputMoment = motor->moment;
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Guide Law Inputs.
			guidelaw->handleInput(navigationState);

			// Guide Law Update.
			guidelaw->update();
			
			// Guide Law Outputs.
			double guidelawOutputNormalGuidanceCommand = guidelaw->gamd_q;
			double guidelawOutputSideGuidanceCommand = guidelaw->gamd_r;
			double guidelawOutputGamDotLimit = guidelaw->gamdot_Lim;
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Roll Auto handle input.
			rollauto->handleInput(
				navigationState,
				atmosOutputMach,
				atmosOutputDynamicPressure
			);

			// Roll Auto Update.
			rollauto->update();

			// Roll Auto Outputs.
			double rollautoOutputRollFinCommandDegrees = rollauto->rollFinCommandDegrees;
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Momcontauto handle input.
			momcontauto->handleInput(
				navigationState,
				guidelawOutputNormalGuidanceCommand,
				guidelawOutputSideGuidanceCommand,
				guidelawOutputGamDotLimit,
				atmosOutputMach,
				atmosOutputDynamicPressure,
				mpropOutputMass,
				mpropOutputXcg
			);

			// Momcontauto update.
			momcontauto->update();

			// Momcontauto outputs.
			double momcontautoOutputYawFinCommandDegrees = momcontauto->yawFinCommandDegrees;
			double momcontautoOutputPitchFinCommandDegrees = momcontauto->pitchFinCommandDegrees;
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Actuators handle input.
			act->handleInput(
				navigationState,
				rollautoOutputRollFinCommandDegrees,
				momcontautoOutputPitchFinCommandDegrees,
				momcontautoOutputYawFinCommandDegrees,
				atmosOutputMach,
				atmosOutputDynamicPressure
			);

			// Actuators Update.
			act->update();

			//Actuators Ouput.
			double actOutputFinOneDeflectionDegrees = act->defl1;
			double actOutputFinTwoDeflectionDegrees = act->defl2;
			double actOutputFinThreeDeflectionDegrees = act->defl3;
			double actOutputFinFourDeflectionDegrees = act->defl4;
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Aero handle input.
			aero->handleInput(
				navigationState,
				actOutputFinOneDeflectionDegrees,
				actOutputFinTwoDeflectionDegrees,
				actOutputFinThreeDeflectionDegrees,
				actOutputFinFourDeflectionDegrees,
				mpropOutputXcg,
				motorOutputThrust,
				atmosOutputDynamicPressure,
				atmosOutputMach
			);

			// Aero update.
			aero->update();
			// Aero outputs.
			Vec aeroOutputForce = aero->force;
			Vec aeroOutputMoment = aero->moment;
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Motion inputs.
			Vec motionInputLTFWindVelocity = atmosOutputLTFWindVelocity;
			Vec motionInputAeroForce = aeroOutputForce;
			Vec motionInputAeroMoment = aeroOutputMoment;
			Vec motionInputMotorForce = motorOutputForce;
			Vec motionInputMotorMoment = motorOutputMoment;
			double motionInputMass = mpropOutputMass;
			Mat motionInputInertiaTensor = mpropOutputInertiaTensor;

			// Motion update.
			mot->update(
				motionInputLTFWindVelocity,
				motionInputAeroForce,
				motionInputAeroMoment,
				motionInputMotorForce,
				motionInputMotorMoment,
				motionInputMass,
				motionInputInertiaTensor
			);
			// Motion outputs grabbed directly from pointer.
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			// END ENGINE
			out->update();
			ec->update();

		}

		if (sys->t_flight > 0.0)
		{
			auto print_it = static_cast<int>(round(sys->t_flight * 10000.0)) % 10000;
			if (print_it == 0)
			{
				std::cout << setprecision(6) << "STATUS AT TIME OF FLIGHT " << sys->t_flight << " X " << mot->XYZ_pos.x << " Y " << mot->XYZ_pos.y << " Z " << mot->XYZ_pos.z << " RANGE " << mot->rng << " MACH " << atm->amach << endl;
				lastTime = sys->t_flight;
			}
		}
		

		if(State::ready)
		{
			// HAVE EACH OBJECT REPORT
			{

				sys->rpt(); // OVER HEAD
				preflt->rpt(); // POINTLESS BUT INTEGRAL

				navproc->rpt();
				guidelaw->rpt();
				rollauto->rpt();
				momcontauto->rpt();
				mprop->rpt();
				atm->rpt();
				motor->rpt();
				act->rpt();
				aero->rpt();
				mot->rpt(); // END ENGINE

				out->rpt();
				ec->rpt();

			}
			State::tickfirst = 0;
		}

		// PROPOGATE STATES
		{
			sys->propagateStates(); // OVER HEAD
			preflt->propagateStates(); // POINTLESS BUT INTEGRAL

			navproc->propagateStates();
			guidelaw->propagateStates();
			rollauto->propagateStates();
			momcontauto->propagateStates();
			mprop->propagateStates();
			atm->propagateStates();
			motor->propagateStates();
			act->propagateStates();
			aero->propagateStates();
			mot->propagateStates(); // END ENGINE

			out->propagateStates();
			ec->propagateStates();
		}

		Block::integrator->updateclock();

	}

	// CONSOLE REPORT
	std::cout << "\n" << endl;
	std::cout << "MISSION REPORT" << endl;
	std::cout << setprecision(6) << "FINAL STATUS AT TIME OF FLIGHT " << sys->t_flight << ": X " << mot->XYZ_pos.x << " Y " << mot->XYZ_pos.y << " Z " << mot->XYZ_pos.z << " RANGE " << mot->rng << " MACH " << atm->amach << endl;
	std::cout << setprecision(6) << "MISS DISTANCE " << ec->missDistance << " >>> FORWARD, LEFT, UP MISS DISTANCE " << ec->forwardLeftUpRelativeMissileToInterceptPos.x << " " << ec->forwardLeftUpRelativeMissileToInterceptPos.y << " " << ec->forwardLeftUpRelativeMissileToInterceptPos.z << endl;
	std::cout << "SIMULATION RESULT: " << ec->lethality << endl;
	auto wallClockEnd = chrono::high_resolution_clock::now();
	auto simRealRunTime = chrono::duration_cast<chrono::milliseconds>(wallClockEnd - wallClockStart);
	std::cout << "SIMULATION RUN TIME: " << simRealRunTime.count() / 1000.0 << " SECONDS" << endl;
	std::cout << "\n" << endl;

	// TERMINATE PROGRAM
	return 0;

}