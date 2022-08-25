# Python libraries.
import logging
import json
import numpy as np
from numpy import array as npa
from numpy import linalg as la
np.set_printoptions(suppress=True, precision=2)

# Utility.
from utility.returnAzAndElevation import returnAngle
from utility import coordinateTransformations as ct
from utility.writePickle import writepickle

# Classes.
from classes.ATM1976 import ATM1976
from classes.MockHellfireMassAndMotor import MockHellfireMassAndMotor

"""

TO DO:
REAL MOTOR MODEL.
FIVE DOF WITH ROTATING, ELLIPTICAL EARTH.
GUIDANCE AND CONTROL.
INS. FROM ZIPFEL.

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
STARTING_CG_FROM_NOSE 0.644605 m
UNCORRECTED_CENTER_OF_DEFLECTION_FROM_NOSE 1.8059 M
UNCORRECTED_REFERENCE_LENGTH 1.85026 m

"""

# CONSTANTS. ###############################################################################
MM_TO_M = 1.0 / 1000.0
RAD_TO_DEG = 57.2957795130823

# INPUTS. ###############################################################################
TIME_STEP = 0.01 # Seconds.
MAX_TIME = 10 # Seconds.
INITIAL_POS = npa([0.0, 0.0]) # Meters.
INITIAL_VEL = npa([1.0, 1.0]) # Meters per second.
FIN_DEFLECTION_DEGREES = -3 # Degrees.
FIN_DEFLECTION_RADIANS = np.radians(FIN_DEFLECTION_DEGREES) # Radians.

# MOCK HELLFIRE PARAMETERS. ###############################################################################
REFERENCE_DIAMETER = 0.18 # Meters.
NOSE_LENGTH = 0.249733 # Meters.
REFERENCE_LENGTH = 1.6 # Meters.
# REFERENCE_LENGTH = 1.85026 # Meters.
WING_HALF_SPAN = 66.1175 * MM_TO_M / 2.0 # Meters.
WING_TIP_CHORD = 91.047 * MM_TO_M # Meters.
WING_ROOT_CHORD = 0.123564 # Meters.
TAIL_HALF_SPAN = 71.3548 * MM_TO_M / 2.0 # Meters.
TAIL_TIP_CHORD = 0.387894 # Meters.
TAIL_ROOT_CHORD = 0.48084 # Meters.
DISTANCE_FROM_BASE_OF_NOSE_TO_WING = 0.323925 # Meters.
CENTER_OF_DEFLECTION_FROM_NOSE = 1.8059 - NOSE_LENGTH # Meters (correction here due to oversight in drawing).
# CENTER_OF_DEFLECTION_FROM_NOSE = 1.8059 # Meters.
WING_AREA = 0.5 * WING_HALF_SPAN * (WING_TIP_CHORD + WING_ROOT_CHORD) # Meters squared.
TAIL_AREA = 0.5 * TAIL_HALF_SPAN * (TAIL_TIP_CHORD + TAIL_ROOT_CHORD) # Meters squared.
REFERENCE_AREA = np.pi * (REFERENCE_DIAMETER ** 2) / 4 # Meters squared.
NOSE_AREA = NOSE_LENGTH * REFERENCE_DIAMETER # Meters squared.
PLANFORM_AREA = (REFERENCE_LENGTH - NOSE_LENGTH) * REFERENCE_DIAMETER + 0.667 * NOSE_LENGTH * REFERENCE_DIAMETER # Meters squared.
NOSE_CENTER_OF_PRESSURE = 0.67 * NOSE_LENGTH # Meters.
WING_CENTER_OF_PRESSURE = NOSE_LENGTH + DISTANCE_FROM_BASE_OF_NOSE_TO_WING + 0.7 * WING_ROOT_CHORD - 0.2 * WING_TIP_CHORD # Meters.
AN = 0.67 * NOSE_LENGTH * REFERENCE_DIAMETER # Meters squared.
AB = (REFERENCE_LENGTH - NOSE_LENGTH) * REFERENCE_DIAMETER # Meters squared.
BODY_CENTER_OF_PRESSURE = (0.67 * AN * NOSE_LENGTH + AB * (NOSE_LENGTH + 0.5 * (REFERENCE_LENGTH - NOSE_LENGTH))) / (AN + AB) # Meters.

# ATMOSPHERE. ###############################################################################
ATMOS = ATM1976()
ATMOS.update(INITIAL_POS[1], la.norm(INITIAL_VEL))
RHO = ATMOS.rho # Kilograms per meter cubed.
Q = ATMOS.q # Pascals.
P = ATMOS.p # Pascals.
A = ATMOS.a # Meters per second.
G = ATMOS.g # Meters per second squared.
MACH = ATMOS.mach # Non dimensional.
BETA = None # "Normalized Speed" - Zarchan. I don't know what this is. Can't find it anywhere else.
if MACH > 1:
	BETA = np.sqrt(MACH ** 2 - 1) # Non dimensional.
else:
	BETA = MACH # Non dimensional.

# MASS AND MOTOR PROPERTIES. ###############################################################################
MASS_AND_MOTOR = MockHellfireMassAndMotor()
MASS_AND_MOTOR.update(0.0, P)
XCG = MASS_AND_MOTOR.CG_VALUES[0] # Meters from nose.
MASS = MASS_AND_MOTOR.MASS# Kilograms.
TMOI = MASS_AND_MOTOR.TRANSVERSE_MOI # Kilograms times meters squared.
THRUST = MASS_AND_MOTOR.THRUST # Newtons.

# BASIC DRAG MODEL. ###############################################################################
CD = 0.1 # Non dimensional.
DRAG_FORCE = CD * REFERENCE_AREA * Q # Newtons.
WIND_TO_BODY = ct.BODY_TO_RANGE_AND_ALTITUDE(0.0) # Non dimensional.
WIND_DRAG_FORCE = npa([-DRAG_FORCE, 0.0]) # Newtons.
BODY_DRAG = (WIND_TO_BODY @ WIND_DRAG_FORCE) / MASS # Meters per second squared.

# STATE. ###############################################################################
TOF = 0.0 # Seconds.
POS_0 = INITIAL_POS # Meters.
VEL_0 = INITIAL_VEL # Meters per second.
SPECIFIC_FORCE = np.zeros(2) # Meters per second squared..
T_0 = returnAngle(VEL_0[1], VEL_0[0]) # Radians.
TDOT_0 = 0.0 # Radians per second.
THETA_TO_LOCAL = ct.BODY_TO_RANGE_AND_ALTITUDE(-1.0 * T_0) # Non dimensional.
SPEED = la.norm(VEL_0) # Meters per second.
VEL_B = THETA_TO_LOCAL @ VEL_0 # Body velocity.
ALPHA = 0.0 # Radians.

 # DATA. ###############################################################################
STATE = None
LOGFILE = open("MockHellfirePython/data.txt", "w")
JSON_LIST = []
JSON_FILEPATH = "MockHellfirePython/JSONdata.pickle"

def writeHeader():

	DATAPOINT = ""
	COUNT = len(STATE.keys())
	for index, key in enumerate(STATE.keys()):
		if index == COUNT - 1:
			DATAPOINT += f"{key}\n"
		else:
			DATAPOINT += f"{key} "
	LOGFILE.write(DATAPOINT)

def writeData():

	DATAPOINT = ""
	COUNT = len(STATE.keys())
	for index, key in enumerate(STATE.keys()):
		VALUE = STATE[f"{key}"]
		if index == COUNT - 1:
			DATAPOINT += f"{VALUE}\n"
		else:
			DATAPOINT += f"{VALUE} "
	LOGFILE.write(DATAPOINT)

def storeJSON():
	JSONOBJ = json.dumps(STATE, indent=4)
	JSON_LIST.append(JSON_LIST)

def writeJSON():
	writepickle(JSON_LIST, JSON_FILEPATH)

def populateState():

	STATE = {

		"XCG": XCG,
		"MASS": MASS,
		"TMOI": TMOI,
		"THRUST": THRUST,

		"RHO": RHO,
		"Q": Q,
		"P": P,
		"A": A,
		"G": G,
		"MACH": MACH,
		"BETA": BETA,

		"TOF": TOF,
		"POS_0X": POS_0[0],
		"POS_0Y": POS_0[1],
		"VEL_0X": VEL_0[0],
		"VEL_0Y": VEL_0[1], 
		"SPEED": SPEED,
		"UDOT_0": SPECIFIC_FORCE[0],
		"WDOT_0": SPECIFIC_FORCE[1],
		"ALPHA": ALPHA,
		"T_0": T_0,
		"TDOT_0": TDOT_0,

	}

	return STATE

STATE = populateState()
writeHeader()
writeData()
storeJSON()

# INTEGRATION. ###############################################################################
INTEGRATION_PASS = 0
STATE_P0 = POS_0
STATE_V0 = VEL_0
STATE_T0 = T_0
STATE_TDOT0 = TDOT_0

V1 = np.zeros(2)
A1 = np.zeros(2)
TDOT1 = 0.0
TDOTDOT1 = 0.0

V2 = np.zeros(2)
A2 = np.zeros(2)
TDOT2 = 0.0
TDOTDOT2 = 0.0

V3 = np.zeros(2)
A3 = np.zeros(2)
TDOT3 = 0.0
TDOTDOT3 = 0.0

V4 = np.zeros(2)
A4 = np.zeros(2)
TDOT4 = 0.0
TDOTDOT4 = 0.0

 # LOOP. ###############################################################################
GO = True
while GO:

	# ATMOSPHERE.
	ATMOS.update(POS_0[1], SPEED)
	RHO = ATMOS.rho # Kilograms per meter cubed.
	Q = ATMOS.q # Pascals.
	P = ATMOS.p # Pascals.
	A = ATMOS.a # Meters per second.
	G = ATMOS.g # Meters per second squared.
	MACH = ATMOS.mach # Non dimensional.
	BETA = None # "Normalized Speed" - Zarchan. I don't know what this is. Can't find it anywhere else.
	if MACH > 1:
		BETA = np.sqrt(MACH ** 2 - 1) # Non dimensional.
	else:
		BETA = MACH # Non dimensional.

	# MASS AND MOTOR PROPERTIES.
	MASS_AND_MOTOR.update(TOF, P)
	XCG = MASS_AND_MOTOR.XCG
	MASS = MASS_AND_MOTOR.MASS
	TMOI = MASS_AND_MOTOR.TRANSVERSE_MOI
	THRUST = MASS_AND_MOTOR.THRUST

	# BASIC DRAG MODEL.
	DRAG_FORCE = CD * REFERENCE_AREA * Q
	WIND_TO_BODY = ct.BODY_TO_RANGE_AND_ALTITUDE(ALPHA)
	WIND_DRAG_FORCE = npa([-DRAG_FORCE, 0.0])
	BODY_DRAG = (WIND_TO_BODY @ WIND_DRAG_FORCE) / MASS

	# ATTITUDE.
	THETA_TO_LOCAL = ct.BODY_TO_RANGE_AND_ALTITUDE(-1.0 * T_0)
	SPEED = la.norm(VEL_0)
	VEL_B = THETA_TO_LOCAL @ VEL_0
	ALPHA = -1.0 * returnAngle(VEL_B[1], VEL_B[0])

	# NORMAL COEFFICIENT AND PITCHING MOMENT COEFFICIENT CALCULATION, NON DIMENSIONAL.
	CN = 2 * ALPHA + \
		(1.5 * PLANFORM_AREA * ALPHA * ALPHA) / REFERENCE_AREA + \
		(8 * WING_AREA * ALPHA) / (BETA * REFERENCE_AREA) + \
		(8 * TAIL_AREA * (ALPHA + FIN_DEFLECTION_RADIANS)) / (BETA * REFERENCE_AREA)
	CM = 2 * ALPHA * ((XCG - NOSE_CENTER_OF_PRESSURE) / REFERENCE_DIAMETER) + \
		((1.5 * PLANFORM_AREA * ALPHA * ALPHA) / REFERENCE_AREA) * ((XCG - BODY_CENTER_OF_PRESSURE) / REFERENCE_DIAMETER) + \
		((8 * WING_AREA * ALPHA) / (BETA * REFERENCE_AREA)) * ((XCG - WING_CENTER_OF_PRESSURE) / REFERENCE_DIAMETER) + \
		((8 * TAIL_AREA * (ALPHA + FIN_DEFLECTION_RADIANS)) / \
		(BETA * REFERENCE_AREA)) * ((XCG - CENTER_OF_DEFLECTION_FROM_NOSE) / REFERENCE_DIAMETER)

	# DERIVATIVES.
	TDOTDOT_0 = (Q * REFERENCE_AREA * REFERENCE_DIAMETER * CM) / TMOI
	WDOT_0 = (Q * REFERENCE_AREA * CN) / MASS
	UDOT_0 = THRUST / MASS
	LOCAL_G = npa([0.0, -1.0 * G])
	BODY_G = THETA_TO_LOCAL @ LOCAL_G
	SPECIFIC_FORCE = npa([UDOT_0, WDOT_0]) + BODY_G + BODY_DRAG
	ACC_0 = (SPECIFIC_FORCE @ THETA_TO_LOCAL)

	# STATE.
	if INTEGRATION_PASS == 0:

		# LOG DATA.
		STATE = populateState()
		writeData()
		storeJSON()

		# CONSOLE REPORT.
		if round(TOF, 3).is_integer():
			print(f"TOF : {TOF:.2f}, RNG, ALT : {POS_0}, MACH : {MACH:.2f}")

		# END CHECK.
		if TOF > MAX_TIME:
			print(f"MAX TIME - TOF : {TOF:.2f}, RNG, ALT : {POS_0}, MACH : {MACH:.2f}")
			GO = False
			writeJSON()
		if POS_0[1] < 0.0:
			print(f"GROUND - TOF : {TOF:.2f}, RNG, ALT : {POS_0}, MACH : {MACH:.2f}")
			GO = False
			writeJSON()
		if np.isnan(np.sum(POS_0)):
			print(f"NAN - TOF : {TOF:.2f}, RNG, ALT : {POS_0}, MACH : {MACH:.2f}")
			GO = False
			writeJSON()

		# BEGIN INTEGRATION PASS.
		STATE_P0 = POS_0
		STATE_V0 = VEL_0
		STATE_T0 = T_0
		STATE_TDOT0 = TDOT_0

		V1 = VEL_0
		A1 = ACC_0
		TDOT1 = TDOT_0
		TDOTDOT1 = TDOTDOT_0

		POS_0 = STATE_P0 + V1 * (TIME_STEP / 2.0)
		VEL_0 = STATE_V0 + A1 * (TIME_STEP / 2.0)
		T_0 = STATE_T0 + TDOT1 * (TIME_STEP / 2.0)
		TDOT_0 = STATE_TDOT0 + TDOTDOT1 * (TIME_STEP / 2.0)

		TOF += (TIME_STEP / 2.0)

		INTEGRATION_PASS += 1

	elif INTEGRATION_PASS == 1:

		V2 = VEL_0
		A2 = ACC_0
		TDOT2 = TDOT_0
		TDOTDOT2 = TDOTDOT_0

		POS_0 = STATE_P0 + V2 * (TIME_STEP / 2.0)
		VEL_0 = STATE_V0 + A2 * (TIME_STEP / 2.0)
		T_0 = STATE_T0 + TDOT2 * (TIME_STEP / 2.0)
		TDOT_0 = STATE_TDOT0 + TDOTDOT2 * (TIME_STEP / 2.0)

		INTEGRATION_PASS += 1

	elif INTEGRATION_PASS == 2:

		V3 = VEL_0
		A3 = ACC_0
		TDOT3 = TDOT_0
		TDOTDOT3 = TDOTDOT_0

		POS_0 = STATE_P0 + V3 * (TIME_STEP)
		VEL_0 = STATE_V0 + A3 * (TIME_STEP)
		T_0 = STATE_T0 + TDOT3 * (TIME_STEP)
		TDOT_0 = STATE_TDOT0 + TDOTDOT3 * (TIME_STEP)

		TOF += (TIME_STEP / 2.0)

		INTEGRATION_PASS += 1

	elif INTEGRATION_PASS == 3:

		V4 = VEL_0
		A4 = ACC_0
		TDOT4 = TDOT_0
		TDOTDOT4 = TDOTDOT_0

		POS_0 = STATE_P0 + (TIME_STEP / 6.0) * (V1 + 2 * V2 + 2 * V3 + V4)
		VEL_0 = STATE_V0 + (TIME_STEP / 6.0) * (A1 + 2 * A2 + 2 * A3 + A4)
		T_0 = STATE_T0 + (TIME_STEP / 6.0) * (TDOT1 + 2 * TDOT2 + 2 * TDOT3 + TDOT4)
		TDOT_0 = STATE_TDOT0 + (TIME_STEP / 6.0) * (TDOTDOT1 + 2 * TDOTDOT2 + 2 * TDOTDOT3 + TDOTDOT4)

		INTEGRATION_PASS = 0