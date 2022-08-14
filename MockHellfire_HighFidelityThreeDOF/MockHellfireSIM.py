
"""


TO DO:

1) GO THROUGH THE ENTIRE CODE AND CHECK UNITS. ***VERY IMPORTANT***
	A) MAKE SURE ALL STATE PARAMETERS ARE BEING POPULATED EACH LOOP.
2) ADD A "READ IN STATE" FUNCTION.
2) CREATE AN ANIMATION TO WATCH THE ORIENTATION OF THE MISSILE AS IT FLYS.
3) ADD DRAG PROFILE TO AERODYNAMICS.
4) VOTING SYSTEM FOR DYNAMICS UPDATE.
5) BASE CLASS - EACH MODULE NEEDS A "NEXT UPDATE TIME."
6) TARGET AND SEEKER MODELS.
7) GUIDANCE AND CONTROL MODULES.
8) ADD AN I.N.S.
	A) JUST PUT A DISTRIBUTION ON THE DYNAMICS TRUTH STATE.



OVERALL STRUCTURE FOR FULL SIMULATION:

DYNAMICS - 
	VOTING SYSTEM FOR FLIGHT UPDATE.
	DYNAMICS RUNS TO THE NEXT SIMULATION TIME.

TARGET AND SEEKER - RATE 2400 HZ

ACTUATOR - RATE 1200 HZ

NAVIGATION, GUIDANCE, AND CONTROL. - RATE 600 HZ
NAVIGATION - TRUTH.
GUIDANCE -
	IF SEEKER ON: 
		PROPORTIONAL GUIDANCE
		DYNAMIC SEEKER WITH ERROR
		TARGET STATES
	IF SEEKER OFF:
		LINE OF ATTACK GUIDANCE
		KINEMATIC SEEKER WITH TRUTH
		WAYPOINT
CONTROL -
	I'M PROBABLY GOING TO TRY AND WRITE A PID RATE CONTROLLER. ELSE I WILL USE EXISTING CODE.



MOCK HELLFIRE DIMENSIONS:

REFERENCE_DIAMETER 0.18 M
REFERENCE_LENGTH 1.6 M
NOSE_LENGTH 0.249733 M
WING_SPAN 66.1175 MM
WING_TIP_CHORD 91.047 MM
WING_ROOT_CHORD 0.123564 M
TAIL_SPAN 71.3548 MM
TAIL_TIP_CHORD 0.387894 M
TAIL_ROOT_CHORD 0.48084 M
DISTANCE_FROM_BASE_OF_NOSE_TO_WING 0.323925 M
STARTING_CG_FROM_NOSE 0.644605 M
FINAL_CG_FROM_NOSE 0.249733 M
EXTENDED_CENTER_OF_DEFLECTION_FROM_NOSE 1.8059 M
EXTENDED_REFERENCE_LENGTH 1.85026 M


"""

# MATH CONSTANTS
MM_TO_M = 1.0 / 1000.0
RAD_TO_DEG = 57.2957795130823
DEG_TO_RAD = 1.0 / RAD_TO_DEG
EPSILON = 0.00000001

# Python libraries.
import numpy as np
from numpy import array as npa
from numpy import linalg as la
import pandas as pd
import matplotlib.pyplot as plt
from ambiance import Atmosphere as atm
np.set_printoptions(suppress=True, precision=2)

# Utility.
from utility.returnAzAndElevation import returnEl
from utility import coordinateTransformations as ct

# Source.
from MockHellfireDYNAMICS import AirframeSimulation
from classes.SecondOrderActuator import SecondOrderActuator
from classes.Target import Target
from classes.KinematicTruthSeeker import KinematicTruthSeeker
from classes.MockHellfireNavGuidanceCtrl import MockHellfireNavGuidanceCtrl

if __name__ == "__main__":

	# Dynamics.
	MockHellfireDynamics = AirframeSimulation(
		INITIAL_MISSILE_RANGE=0.0,
		INITIAL_MISSILE_ALTITUDE=0.0,
		INITIAL_HRZ_VEL=10.0,
		INITIAL_VRT_VEL=10.0
	)

	# Actuator.
	Actuator = SecondOrderActuator()

	# Target and seeker.
	HellfireTarget = Target(
		INITIAL_TARGET_POSITION=npa([3000.0, 3000.0]),
		INITIAL_TARGET_VELOCITY=np.zeros(2)
	)
	Seeker = KinematicTruthSeeker()

	# Navigation, guidance, and control.
	NGC = MockHellfireNavGuidanceCtrl()

	TIME_INCREMENT = None # SECONDS

	while MockHellfireDynamics.getDynamicsState()["LETHALITY"] == "FLYING" or MockHellfireDynamics.getDynamicsState()["LETHALITY"] == "MAX_TIME":

		# Dynamics time of flight is reference.
		TOF = MockHellfireDynamics.getDynamicsState()["TOF"]

		# Get next update time.
		N = Actuator.NEXT_UPDATE_TIME
		N_ID = "ACTUATOR"
		if HellfireTarget.NEXT_UPDATE_TIME < N:
			N = HellfireTarget.NEXT_UPDATE_TIME
			N_ID = "TARGET"
		if Seeker.NEXT_UPDATE_TIME < N:
			N = Seeker.NEXT_UPDATE_TIME
			N_ID = "SEEKER"
		if NGC.NEXT_UPDATE_TIME < N:
			N = NGC.NEXT_UPDATE_TIME
			N_ID = "NGC"

		# Update dynamics.
		TIME_INCREMENT = N - TOF
		if TIME_INCREMENT < EPSILON:
			pass
		else:
			MockHellfireDynamics.dynamics(
				FLY_FOR_THIS_LONG=TIME_INCREMENT,
				FIN_DEFL_DEG=Actuator.DEFLECTION
			)
		DYNAMICS_STATE = MockHellfireDynamics.getDynamicsState()

		# Update components.
		if N_ID == "ACTUATOR":
			Actuator.update(NGC.DEFLECTION_COMMAND)
		elif N_ID == "TARGET":
			HellfireTarget.update()
		elif N_ID == "SEEKER":
			Seeker.update(
				BODY_TO_R_A_MATRIX=ct.BODY_TO_RANGE_AND_ALTITUDE(-1.0 * DYNAMICS_STATE["THT"]),
				MSL_POS=npa([DYNAMICS_STATE["RNG"], DYNAMICS_STATE["ALT"]]),
				MSL_VEL=npa([DYNAMICS_STATE["RNG_VEL"], DYNAMICS_STATE["ALT_VEL"]]),
				TGT_POS=HellfireTarget.targetRightUpPosition,
				TGT_VEL=HellfireTarget.targetRightUpVelocity
			)
		elif N_ID == "NGC":
			NGC.update(
				DYNAMICS_TRUTH_STATE=DYNAMICS_STATE,
				LINE_OF_SIGHT_RATE=Seeker.LINE_OF_SIGHT_RATE,
				CLOSING_SPEED=Seeker.CLOSING_SPEED
			)

	print(MockHellfireDynamics.getDynamicsState()["LETHALITY"])