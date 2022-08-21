# Python libraries.
import time
import copy
import numpy as np
from numpy import array as npa
from numpy import linalg as la
import pymap3d
np.set_printoptions(suppress=True, precision=2)

# Utility.

# Classes.
from classes.SecondOrderActuator import SecondOrderActuator

# Dynamics.
import MockHellfireDYNAMICS5DOF as DYN

"""

TO DO:
REAL MOTOR MODEL.
ROTATING, ELLIPTICAL EARTH MODEL.
GEOPOTENTIAL GRAVITY MODEL.
GUIDANCE AND CONTROL.
INS. FROM ZIPFEL.

"""

# MATH CONSTANTS
MM_TO_M = 1.0 / 1000.0
RAD_TO_DEG = 57.2957795130823
DEG_TO_RAD = 1.0 / RAD_TO_DEG
EPSILON = 0.00000001

if __name__ == "__main__":
	
	# Dynamics.
	LLA0 = npa([38.8719, 77.0563, 0.0])
	POS0 = np.zeros(3)
	AZ0 = -25
	EL0 = 45
	SPD0 = 10
	ID = "MOCK_HELLFIRE5DOF"
	MSL = DYN.Construct5DOFMissile(POS0, AZ0, EL0, SPD0, ID)

	# Actuators.
	PITCH_ACT = SecondOrderActuator("PITCH_DEFL")
	YAW_ACT = SecondOrderActuator("YAW_DEFL")

	# Sim control.
	TIME_INCREMENT = None
	
	# Simple Guidance and Control.
	MANEUVER = 15 # Seconds.
	PITCH_FIN_COMMAND = None
	YAW_FIN_COMMAND = None
	PITCHCOMMAND1 = -2 # Degrees.
	YAWCOMMAND1 = -4 # Degrees.
	PITCHCOMMAND2 = -1 # Degrees.
	YAWCOMMAND2 = 4 # Degrees.

	LAST_TIME = 0
	while MSL["LETHALITY"] == "FLYING" or MSL["LETHALITY"] == "MAX_TIME":

		# Dynamics tof is driver.
		TOF = MSL["STATE"]["TOF"]

		# Basic guidance and control.
		if TOF < MANEUVER:
			PITCH_FIN_COMMAND = PITCHCOMMAND1
			YAW_FIN_COMMAND = YAWCOMMAND1
		else:
			PITCH_FIN_COMMAND = PITCHCOMMAND2
			YAW_FIN_COMMAND = YAWCOMMAND2

		# Get next update time.
		N = PITCH_ACT.NEXT_UPDATE_TIME
		N_ID = "PITCH_ACT"
		if YAW_ACT.NEXT_UPDATE_TIME < N:
			N = YAW_ACT.NEXT_UPDATE_TIME
			N_ID = "YAW_ACT"

		# Update dynamics.
		TIME_INCREMENT = N - TOF
		if TIME_INCREMENT > EPSILON:
			MSL = DYN.Fly5DOF(
				MSL_INPUT=MSL,
				FLY_FOR_THIS_LONG=TIME_INCREMENT,
				PITCH_FIN_DEFL_DEG_INPUT=PITCH_ACT.DEFLECTION,
				YAW_FIN_DEFL_DEG_INPUT=YAW_ACT.DEFLECTION
			)

		# Update components.
		if N_ID == "PITCH_ACT":
			PITCH_ACT.update(PITCH_FIN_COMMAND)
		elif N_ID == "YAW_ACT":
			YAW_ACT.update(YAW_FIN_COMMAND)

		# Console report.
		CHECK = round(TOF, 3).is_integer()
		if CHECK and TOF != LAST_TIME:
			X = MSL["STATE"]["POS_0X"]
			Y = MSL["STATE"]["POS_0Y"]
			Z = MSL["STATE"]["POS_0Z"]
			print(f"TOF {TOF:.3} ENU {X:.2f} {Y:.2f} {Z:.2f}")
			LAST_TIME = TOF

