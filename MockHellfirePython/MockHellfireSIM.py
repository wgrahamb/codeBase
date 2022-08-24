# Python libraries.
import numpy as np
from numpy import array as npa
np.set_printoptions(suppress=True, precision=2)

# Utility.

# Classes.
from classes.SecondOrderActuator import SecondOrderActuator

# Dynamics.
import MockHellfireDYNAMICS5DOF as DYN

"""

TO DO:
REAL MOTOR MODEL.
GUIDANCE AND CONTROL.
INS. FROM ZIPFEL.

"""

# MATH CONSTANTS
MM_TO_M = 1.0 / 1000.0
RAD_TO_DEG = 57.2957795130823
DEG_TO_RAD = 1.0 / RAD_TO_DEG
EPSILON = 0.00001

if __name__ == "__main__":

	# Dynamics.
	LLA0 = npa([38.8719, 77.0563, 0.0])
	POS0 = np.zeros(3)
	AZ0 = 0
	EL0 = 45
	SPD0 = 10
	ID = "MOCK_HELLFIRE5DOF"
	MSL = DYN.Construct5DOFMissile(POS0, AZ0, EL0, SPD0, ID)

	# Components.
	COMPONENTS = {
		"PITCH_ACT": SecondOrderActuator("PITCH_DEFL"),
		"YAW_ACT": SecondOrderActuator("YAW_DEFL"),
	}

	# Sim control.
	TIME_INCREMENT = None

	# Simple Guidance and Control.
	MANEUVER1 = 20 # Seconds.
	MANEUVER2 = 40
	PITCH_FIN_COMMAND = None
	YAW_FIN_COMMAND = None
	PITCHCOMMAND1 = -3 # Degrees.
	YAWCOMMAND1 = 0 # Degrees.
	PITCHCOMMAND2 = -2 # Degrees.
	YAWCOMMAND2 = 0 # Degrees.
	PITCHCOMMAND3 = 5 # Degrees.
	YAWCOMMAND3 = 0 # Degrees.

	LAST_TIME = 0
	while MSL["LETHALITY"] == "FLYING" or MSL["LETHALITY"] == "MAX_TIME":

		# Dynamics tof is driver.
		TOF = MSL["STATE"]["TOF"]

		# Basic guidance and control.
		if TOF < MANEUVER1:
			PITCH_FIN_COMMAND = PITCHCOMMAND1
			YAW_FIN_COMMAND = YAWCOMMAND1
		elif TOF < MANEUVER2:
			PITCH_FIN_COMMAND = PITCHCOMMAND2
			YAW_FIN_COMMAND = YAWCOMMAND2
		else:
			PITCH_FIN_COMMAND = PITCHCOMMAND3
			YAW_FIN_COMMAND = YAWCOMMAND3

		# Get next update time.
		N = 0.0
		N_ID = ""
		for index, key in enumerate(COMPONENTS.keys()):
			if index == 0:
				N = COMPONENTS[f"{key}"].NEXT_UPDATE_TIME
				N_ID = key
			elif COMPONENTS[f"{key}"].NEXT_UPDATE_TIME < N:
				N = COMPONENTS[f"{key}"].NEXT_UPDATE_TIME
				N_ID = key

		# Update dynamics.
		TIME_INCREMENT = N - TOF
		if TIME_INCREMENT > EPSILON:
			MSL = DYN.Fly5DOF(
				MISSILE_INPUT_DICT=MSL,
				FLY_FOR_THIS_LONG=TIME_INCREMENT,
				PITCH_FIN_DEFL_DEG_INPUT=COMPONENTS["PITCH_ACT"].DEFLECTION,
				YAW_FIN_DEFL_DEG_INPUT=COMPONENTS["YAW_ACT"].DEFLECTION
			)

		# Update components. Would like to make this bit more terse.
		if N_ID == "PITCH_ACT":
			COMPONENTS["PITCH_ACT"].update(PITCH_FIN_COMMAND)
		elif N_ID == "YAW_ACT":
			COMPONENTS["YAW_ACT"].update(YAW_FIN_COMMAND)

		# Console report.
		CHECK = round(TOF, 3).is_integer()
		if CHECK and TOF != LAST_TIME:
			X = MSL["STATE"]["POS_0X"]
			Y = MSL["STATE"]["POS_0Y"]
			Z = MSL["STATE"]["POS_0Z"]
			MACH = MSL["STATE"]["MACH"]
			print(f"TOF {TOF:.3} ENU {X:.2f} {Y:.2f} {Z:.2f} MACH {MACH:.2f}")
			LAST_TIME = TOF

