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
Airframe *air;
Aero *aero;
Motion *mot;
navigatorPerfect *navigator;
NavProc *navproc;
FlightProc *flightproc;
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
	air = new Airframe("airframe.dat", out, sys);
	aero = new Aero("aero.dat", out, sys);
	mot = new Motion("motion.dat", out, sys);

	// GNC
	navigator = new navigatorPerfect("navigatorPerfect.dat", out, sys);
	navproc = new NavProc("navproc.dat", out, sys);
	flightproc = new FlightProc("flightproc.dat", out, sys);
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

	navigator->init(); // START ENGINE
	navproc->init();
	flightproc->init();

	guidelaw->init();
	rollauto->init();
	momcontauto->init();
	mprop->init();
	atm->init();
	motor->init();
	act->init();
	air->init();
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
			3) After done, remove bloat (navproc, flightproc).
			4) Try and hack the integrator. Since euler integration works, should be easy.
			5) Once the integrator is hacked, I am the captain now.

			*/
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Navigation state.
			double missileTimeOfFlight = sys->t_flight;
			Eigen::Vector3d missileLTFPosition = {mot->ltf_pos.x, mot->ltf_pos.y, mot->ltf_pos.z};
			Eigen::Vector3d missileLTFVelocity = {mot->ltf_vel.x, mot->ltf_vel.y, mot->ltf_vel.z};
			Eigen::Vector3d missileBodyAcceleration = {mot->sf_b.x, mot->sf_b.y, mot->sf_b.z};
			Eigen::Vector3d missileLTFEulerAngles = {mot->eulerLTF.x, mot->eulerLTF.y, mot->eulerLTF.z};
			Eigen::Vector3d missileBodyRate = {mot->omegaB.x, mot->omegaB.y, mot->omegaB.z};
			Eigen::Vector3d missileBodyRateDot = {mot->omegaB_d.x, mot->omegaB_d.y, mot->omegaB_d.z};

			NavigationState navigationState(
				missileTimeOfFlight,
				missileLTFPosition,
				missileLTFVelocity,
				missileBodyAcceleration,
				missileLTFEulerAngles,
				missileBodyRate,
				missileBodyRateDot
			);

			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Navigator handle inputs.
			navigator->handleInput(navigationState);

			// Navigator Update. (Does not actually do anything but I am leaving it for now.)
			navigator->update();

			// Navigator Outputs.
			double navigatorOutputLTFAltitude = -1 * navigator->pm.z;
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Navproc handle inputs.
			navproc->handleInput(navigationState);

			// Navproc Update.
			navproc->update();

			// Navproc Outputs.
			bool navprocOutputProcessExecuting = navproc->executing;
			double navprocOutputAltitude = navproc->pm_nav.z;
			Vecff navprocOutputBodyVelocity = navproc->vb_nav;
			Vec navprocOutputBodyRate;
			navprocOutputBodyRate.x = navproc->wx_nav;
			navprocOutputBodyRate.y = navproc->wy_nav;
			navprocOutputBodyRate.z = navproc->wz_nav;
			Vecff navprocOutputGravityBodyEstimate = navproc->gravb_nav;
			Vecff navprocOutputBodyAcceleration = navproc->accb_nav;
			double navprocOutputRollAngle = navproc->phi_hat;
			Vecff navprocOutputWaypoint = navproc->pt;
			Vecff navprocOutputLTFPosNav = navproc->pm_nav;
			Vecff navprocOutputLTFVelNav = navproc->vm_nav;
			bool navprocOutputNav200Valid = navproc->nav200_valid;
			Matff navprocOutputDCMNav = navproc->mDCM_nav;
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Flightproc handle inputs.
			flightproc->handleInput(navigationState);

			// Flightproc Update.
			flightproc->update();

			// Flightproc Outputs.
			Matff flightprocOutputRolledToNonRolledDCM = flightproc->BodyRollToNR_DCM;
			double flightprocOutputAlpha = flightproc->alpha_est_nr;
			double flightprocOutputBeta = flightproc->beta_est_nr;
			double flightprocOutputAxialMomentOfInertia = flightproc->ajx_est;
			Vecff flightprocOutputNonRolledBodyRateEstimate = flightproc->w_est_nr;
			double flightprocOutputMach = flightproc->amach;
			double flightprocOutputCld = flightproc->cld_est;
			double flightprocOutputDynamicPressure = flightproc->q_est;
			double flightprocOutputReferenceArea = flightproc->sref_est;
			double flightprocOutputReferenceDiameter = flightproc->dia_est;
			double flightprocOutputCna = flightproc->cna_est;
			double flightprocOutputCnd = flightproc->cnd_est;
			double flightprocOutputCmd0 = flightproc->cmd0_est;
			double flightprocOutputTransverseMomentOfInertia = flightproc->ajy_est;
			double flightprocOutputMass = flightproc->amass_est;
			double flightprocOutputXcg = flightproc->xcg_est;
			double flightprocOutputXimu = flightproc->ximu_est;
			double flightprocOutputSpeed = flightproc->v_est;
			Vecff flightprocOutputBodyGravEstimate = flightproc->gb_est_nr;
			Vecff flightprocOutputBodyAccelEstimate = flightproc->abg_est_nr;
			double flightprocOutputAngleOfAttack = flightproc->alphaTot_est_nr;
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Guide Law Inputs.
			bool guideLawInputProcessExecuting = true;
			Vecff guideLawInputLTFWaypoint = navprocOutputWaypoint;
			Vecff guideLawInputLTFPos = navprocOutputLTFPosNav;
			Vecff guideLawInputLTFVel = navprocOutputLTFVelNav;
			bool guideLawInputNav200Valid = true;
			double guideLawInputRollAngle = navprocOutputRollAngle;
			Matff guideLawInputDCMNav = navprocOutputDCMNav;
			Matff guideLawInputRolledToNonRolledBodyDCM = flightprocOutputRolledToNonRolledDCM;
			double guideLawInputAlpha = flightprocOutputAlpha;
			double guideLawInputBeta = flightprocOutputBeta;

			// Guide Law Update.
			guidelaw->update(
				guideLawInputProcessExecuting,
				guideLawInputLTFWaypoint,
				guideLawInputLTFPos,
				guideLawInputLTFVel,
				guideLawInputNav200Valid,
				guideLawInputRollAngle,
				guideLawInputDCMNav,
				guideLawInputRolledToNonRolledBodyDCM,
				guideLawInputAlpha,
				guideLawInputBeta
			);
			
			// Guide Law Outputs.
			double guidelawOutputNormalGuidanceCommand = guidelaw->gamd_q;
			double guidelawOutputSideGuidanceCommand = guidelaw->gamd_r;
			double guidelawOutputGamDotLimit = guidelaw->gamdot_Lim;
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Roll Auto Inputs.
			bool rollautoInputProcessExecuting = true;
			double rollautoInputAxialMomentOfInertia = flightprocOutputAxialMomentOfInertia;
			Vecff rollautoInputNonRolledBodyRateEstimate = flightprocOutputNonRolledBodyRateEstimate;
			double rollautoInputMach = flightprocOutputMach;
			double rollautoInputCld = flightprocOutputCld;
			double rollautoInputDynamicPressure = flightprocOutputDynamicPressure;
			double rollautoInputReferenceArea = flightprocOutputReferenceArea;
			double rollautoInputReferenceDiameter = flightprocOutputReferenceDiameter;
			double rollautoInputRollAngle = navprocOutputRollAngle;

			// Roll Auto Update.
			rollauto->update(
				rollautoInputProcessExecuting,
				rollautoInputAxialMomentOfInertia,
				rollautoInputNonRolledBodyRateEstimate,
				rollautoInputMach,
				rollautoInputCld,
				rollautoInputDynamicPressure,
				rollautoInputReferenceArea,
				rollautoInputReferenceDiameter,
				rollautoInputRollAngle
			);

			// Roll Auto Outputs.
			double rollautoOutputRollFinCommandDegrees = rollauto->rollFinCommandDegrees;
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Momcontauto inputs.
			double momcontautoInputNormalGuidanceCommand = guidelawOutputNormalGuidanceCommand;
			double momcontautoInputSideGuidanceCommand = guidelawOutputSideGuidanceCommand;
			double momcontautoInputGamDotLimit = guidelawOutputGamDotLimit;
			bool momcontautoInputProcessExecuting = navprocOutputProcessExecuting;
			double momcontautoInputRollAngle = navprocOutputRollAngle;
			double momcontautoInputMach = flightprocOutputMach;
			double momcontautoInputDynamicPressure = flightprocOutputDynamicPressure;
			double momcontautoInputReferenceArea = flightprocOutputReferenceArea;
			double momcontautoInputCna = flightprocOutputCna;
			double momcontautoInputCnd = flightprocOutputCnd;
			double momcontautoInputCmd0 = flightprocOutputCmd0;
			double momcontautoInputTransverseMomentOfInertia = flightprocOutputTransverseMomentOfInertia;
			double momcontautoInputMass = flightprocOutputMass;
			double momcontautoInputXcg = flightprocOutputXcg;
			double momcontautoInputXimu = flightprocOutputXimu;
			double momcontautoInputReferenceDiameter = flightprocOutputReferenceDiameter;
			double momcontautoInputSpeed = flightprocOutputSpeed;
			Vecff momcontautoInputNonRolledBodyRate = flightprocOutputNonRolledBodyRateEstimate;
			Vecff momcontautoInputBodyGravEstimate = flightprocOutputBodyGravEstimate;
			Vecff momcontautoInputBodyAccelEstimate = flightprocOutputBodyAccelEstimate;
			double momcontautoInputAlpha = flightprocOutputAlpha;
			double momcontautoInputBeta = flightprocOutputBeta;
			double momcontautoInputAngleOfAttack = flightprocOutputAngleOfAttack;

			// Momcontauto update.
			momcontauto->update(
				momcontautoInputNormalGuidanceCommand,
				momcontautoInputSideGuidanceCommand,
				momcontautoInputGamDotLimit,
				momcontautoInputProcessExecuting,
				momcontautoInputRollAngle,
				momcontautoInputMach,
				momcontautoInputDynamicPressure,
				momcontautoInputReferenceArea,
				momcontautoInputCna,
				momcontautoInputCnd,
				momcontautoInputCmd0,
				momcontautoInputTransverseMomentOfInertia,
				momcontautoInputMass,
				momcontautoInputXcg,
				momcontautoInputXimu,
				momcontautoInputReferenceDiameter,
				momcontautoInputSpeed,
				momcontautoInputNonRolledBodyRate,
				momcontautoInputBodyGravEstimate,
				momcontautoInputBodyAccelEstimate,
				momcontautoInputAlpha,
				momcontautoInputBeta,
				momcontautoInputAngleOfAttack
			);

			// Momcontauto outputs.
			double momcontautoOutputYawFinCommandDegrees = momcontauto->yawFinCommandDegrees;
			double momcontautoOutputPitchFinCommandDegrees = momcontauto->pitchFinCommandDegrees;
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Mass properties inputs.
			double mpropInputMassEstimate = flightprocOutputMass;

			// Mass properties update.
			mprop->update(mpropInputMassEstimate);

			// Mass properties output.
			double mpropOutputXcg = mprop->xcg;
			double mpropOutputYcg = mprop->ycg;
			double mpropOutputZcg = mprop->zcg;
			double mpropOutputMass = mprop->mass;
			Mat mpropOutputInertiaTensor = mprop->iten;
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Atmosphere Inputs.
			double atmosInputLTFAltitude = navigatorOutputLTFAltitude;
			double atmosInputSpeed = flightprocOutputSpeed;

			// Atmosphere Update.
			atm->update(
				atmosInputLTFAltitude,
				atmosInputSpeed
			);

			// Atmoshpere Outputs.
			double atmosOutputPressure = atm->p;
			double atmosOutputDynamicPressure = atm->q;
			double atmosOutputMach = atm->amach;
			double atmosOutputAirTemp = atm->airTemp;
			double atmosOutputAirTempNominal = atm->airTemp_nom;
			Vec atmosOutputLTFWindVelocity = atm->vwind;
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Motor inputs.
			double motorInputXCenterOfGravity = mpropOutputXcg;
			double motorInputYCenterOfGravity = mpropOutputYcg;
			double motorInputZCenterOfGravity = mpropOutputZcg;
			double motorInputAirTemp = atmosOutputAirTemp;
			double motorInputAirTempNominal = atmosOutputAirTempNominal;
			double motorInputPressure = atmosOutputPressure;

			// Motor update.
			motor->update(
				motorInputXCenterOfGravity,
				motorInputYCenterOfGravity,
				motorInputZCenterOfGravity,
				motorInputAirTemp,
				motorInputAirTempNominal,
				motorInputPressure
			);

			// Motor outputs.
			double motorOutputThrust = motor->thrust;
			Vec motorOutputForce = motor->force;
			Vec motorOutputMoment = motor->moment;
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Actuators Input.
			double actInputRollFinCommandDegrees = rollautoOutputRollFinCommandDegrees;
			double actInputPitchFinCommandDegrees = momcontautoOutputPitchFinCommandDegrees;
			double actInputYawFinCommandDegrees = momcontautoOutputYawFinCommandDegrees;
			double actInputAngleOfAttack = flightprocOutputAngleOfAttack;
			double actInputMach = flightprocOutputMach;
			double actInputDynamicPressure = flightprocOutputDynamicPressure;
			double actInputRollAngle = navprocOutputRollAngle;

			// Actuators Update.
			act->update(
				actInputRollFinCommandDegrees,
				actInputPitchFinCommandDegrees,
				actInputYawFinCommandDegrees,
				actInputAngleOfAttack,
				actInputMach,
				actInputDynamicPressure,
				actInputRollAngle
			);

			//Actuators Ouput.
			double actOutputFinOneDeflectionDegrees = act->defl1;
			double actOutputFinTwoDeflectionDegrees = act->defl2;
			double actOutputFinThreeDeflectionDegrees = act->defl3;
			double actOutputFinFourDeflectionDegrees = act->defl4;
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Airframe inputs.
			// Airframe update.
			air->update();
			// Airframe outputs.
			double airframeOutputRefDiameter = air->dia;
			double airframeOutputRefArea = air->sref;
			int airframeOutputSpinFlag = air->spin_Flag;
			double airframeOutputRailLength = air->xrail;
			double airframeOutputPScale = air->pScale;
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
			// Aero inputs.
			double aeroInputFinOneDeflectionDegrees = actOutputFinOneDeflectionDegrees;
			double aeroInputFinTwoDeflectionDegrees = actOutputFinTwoDeflectionDegrees;
			double aeroInputFinThreeDeflectionDegrees = actOutputFinThreeDeflectionDegrees;
			double aeroInputFinFourDeflectionDegrees = actOutputFinFourDeflectionDegrees;
			double aeroInputCenterOfGravity = mpropOutputXcg;
			double aeroInputThrust = motorOutputThrust;
			double aeroInputRefDiam = airframeOutputRefDiameter;
			double aeroInputRefArea = airframeOutputRefArea;
			double aeroInputDynamicPressure = atmosOutputDynamicPressure;
			double aeroInputMach = atmosOutputMach;
			double aeroInputTotalAngleOfAttack = flightprocOutputAngleOfAttack;
			double aeroInputPhiPrime = mot->aphi;
			double aeroInputSpeed = flightprocOutputSpeed;
			double aeroInputRollRate = flightprocOutputNonRolledBodyRateEstimate.x;
			double aeroInputPitchRate = flightprocOutputNonRolledBodyRateEstimate.y;

			// Aero update.
			aero->update(
				aeroInputFinOneDeflectionDegrees,
				aeroInputFinTwoDeflectionDegrees,
				aeroInputFinThreeDeflectionDegrees,
				aeroInputFinFourDeflectionDegrees,
				aeroInputCenterOfGravity,
				aeroInputThrust,
				aeroInputRefDiam,
				aeroInputRefArea,
				aeroInputDynamicPressure,
				aeroInputMach,
				aeroInputTotalAngleOfAttack,
				aeroInputPhiPrime,
				aeroInputSpeed,
				aeroInputRollRate,
				aeroInputPitchRate
			);
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
			int motionInputSpinFlag = airframeOutputSpinFlag;
			double motionInputRailLength = airframeOutputRailLength;
			double motionInputPScale = airframeOutputPScale;
			double motionInputMass = mpropOutputMass;
			Mat motionInputInertiaTensor = mpropOutputInertiaTensor;

			// Motion update.
			mot->update(
				motionInputLTFWindVelocity,
				motionInputAeroForce,
				motionInputAeroMoment,
				motionInputMotorForce,
				motionInputMotorMoment,
				motionInputSpinFlag,
				motionInputRailLength,
				motionInputPScale,
				motionInputMass,
				motionInputInertiaTensor
			);
			// Motion outputs grabbed directly from pointer.
			////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////





			// END ENGINE

			out->update();
			ec->update();

		}

		// cout << sys->t_sys << " " << sys->mode << endl;

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

				navigator->rpt(); // START ENGINE
				navproc->rpt();
				flightproc->rpt();
				guidelaw->rpt();
				rollauto->rpt();
				momcontauto->rpt();
				mprop->rpt();
				atm->rpt();
				motor->rpt();
				act->rpt();
				aero->rpt();
				air->rpt();
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

			navigator->propagateStates(); // START ENGINE
			navproc->propagateStates();
			flightproc->propagateStates();
			guidelaw->propagateStates();
			rollauto->propagateStates();
			momcontauto->propagateStates();
			mprop->propagateStates();
			atm->propagateStates();
			motor->propagateStates();
			act->propagateStates();
			aero->propagateStates();
			air->propagateStates();
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