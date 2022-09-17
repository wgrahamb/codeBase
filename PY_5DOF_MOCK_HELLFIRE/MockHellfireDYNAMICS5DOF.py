
# Python libraries.
import time
import copy
import numpy as np
from numpy import array as npa
from numpy import linalg as la
np.set_printoptions(suppress=True, precision=2)

# Utility.
from utility.returnAzAndElevation import returnAlphaAndBeta
from utility.interpolationGivenTwoVectors import linearInterpolation
import utility.coordinateTransformations as ct
import utility.loggingFxns as lf
import utility.earthTransforms as et

# Classes.
from classes.ATM1976 import ATM1976
from classes.MockHellfireMassAndMotor import MockHellfireMassAndMotor
from classes.SphericalNoseDrag import SphericalNoseDrag

"""

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
CENTER_OF_DEFLECTION_FROM_NOSE (1.8059 M - 0.249733 M)

UNCORRECTED_CENTER_OF_DEFLECTION_FROM_NOSE 1.8059 M
UNCORRECTED_REFERENCE_LENGTH 1.85026 m

"""

def Construct5DOFMissile(
	INITIAL_POSITION,
	INITIAL_AZIMUTH,
	INITIAL_ELEVATION,
	INITIAL_AIRSPEED,
	ID
):

	# ATMOSPHERE. ###############################################################################
	ATMOS = ATM1976()
	ATMOS.update(INITIAL_POSITION[1], la.norm(INITIAL_AIRSPEED))
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

	# STATE. ###############################################################################
	INITIAL_AZ = np.radians(INITIAL_AZIMUTH) # RADIANS.
	INITIAL_EL = np.radians(INITIAL_ELEVATION) # RADIANS.
	ENU_TO_FLU = ct.ORIENTATION_TO_LOCAL_TM(0.0, -INITIAL_EL, INITIAL_AZ)

	TOF = 0.0 # Seconds.
	SPEED = la.norm(INITIAL_AIRSPEED * (ENU_TO_FLU[0])) # Meters per second.
	VEL_B = ENU_TO_FLU @ (INITIAL_AIRSPEED * (ENU_TO_FLU[0])) # Body velocity.
	ALPHA, SIDESLIP = returnAlphaAndBeta(VEL_B)
	SPECIFIC_FORCE = np.zeros(3) # Meters per second squared.
	BODYRATE = np.zeros(3) # Radians per second.
	
	ENUPOS = INITIAL_POSITION # Meters.
	ENUVEL = INITIAL_AIRSPEED * (ENU_TO_FLU[0])
	ENUEULER = npa([0.0, INITIAL_EL, INITIAL_AZ]) # Radians.

	# DATA. ###############################################################################
	MISSILE = {

		"IDENTITY": ID,
		"LOGFILE": open(f"PY_5DOF_MOCK_HELLFIRE/output/{ID}.txt", "w"),
		"LETHALITY": "FLYING",
		"ATMOS": ATMOS,
		"MASS_AND_MOTOR": MASS_AND_MOTOR,
		"STATE": {

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
			"SPEED": SPEED,
			"ALPHA": ALPHA,
			"SIDESLIP": SIDESLIP,
			"PRATE": BODYRATE[0],
			"QRATE": BODYRATE[1],
			"RRATE": BODYRATE[2],
			"UDOT_0": SPECIFIC_FORCE[0],
			"VDOT_0": SPECIFIC_FORCE[1],
			"WDOT_0": SPECIFIC_FORCE[2],

			"ENUPOSX": ENUPOS[0],
			"ENUPOSY": ENUPOS[1],
			"ENUPOSZ": ENUPOS[2],
			"ENUVELX": ENUVEL[0],
			"ENUVELY": ENUVEL[1], 
			"ENUVELZ": ENUVEL[2], 
			"ENUPHI": ENUEULER[0],
			"ENUTHT": ENUEULER[1],
			"ENUPSI": ENUEULER[2],

		}

	}

	lf.writeHeader(MISSILE["STATE"], MISSILE["LOGFILE"])
	lf.writeData(MISSILE["STATE"], MISSILE["LOGFILE"])

	return MISSILE

def Fly5DOF(
	MISSILE_INPUT_DICT,
	FLY_FOR_THIS_LONG,
	PITCH_FIN_DEFL_DEG_INPUT,
	YAW_FIN_DEFL_DEG_INPUT
):

	# HANDLE INPUT. ###############################################################################
	MSL = copy.copy(MISSILE_INPUT_DICT)

	# PROCESS INPUT. ###############################################################################
	PITCH_FIN_DEFL_DEG = PITCH_FIN_DEFL_DEG_INPUT # Degrees.
	PITCH_FIN_DEFL_RAD = np.radians(PITCH_FIN_DEFL_DEG) # Radians.
	YAW_FIN_DEFL_DEG = YAW_FIN_DEFL_DEG_INPUT # Degrees.
	YAW_FIN_DEFL_RAD = np.radians(YAW_FIN_DEFL_DEG)
	MAX_TIME = MSL["STATE"]["TOF"] + FLY_FOR_THIS_LONG
	TIME_STEP = FLY_FOR_THIS_LONG
	DT_LIM = (1.0 / 100.0)
	if TIME_STEP > DT_LIM:
		TIME_STEP = DT_LIM

	# ALLOCATE STATE. ###############################################################################
	XCG = MSL["STATE"]["XCG"]
	MASS = MSL["STATE"]["MASS"]
	TMOI = MSL["STATE"]["TMOI"]
	THRUST = MSL["STATE"]["THRUST"]

	RHO = MSL["STATE"]["RHO"]
	Q = MSL["STATE"]["Q"]
	P = MSL["STATE"]["P"]
	A = MSL["STATE"]["A"]
	G = MSL["STATE"]["G"]
	MACH = MSL["STATE"]["MACH"]
	BETA = MSL["STATE"]["BETA"]

	TOF = MSL["STATE"]["TOF"]
	SPEED = MSL["STATE"]["SPEED"]
	ALPHA = MSL["STATE"]["ALPHA"]
	SIDESLIP = MSL["STATE"]["SIDESLIP"]
	BODYRATE = np.zeros(3)
	BODYRATE[0] = MSL["STATE"]["PRATE"]
	BODYRATE[1] = MSL["STATE"]["QRATE"]
	BODYRATE[2] = MSL["STATE"]["RRATE"]
	SPECIFIC_FORCE = np.zeros(3)
	SPECIFIC_FORCE[0] = MSL["STATE"]["UDOT_0"]
	SPECIFIC_FORCE[1] = MSL["STATE"]["VDOT_0"]
	SPECIFIC_FORCE[2] = MSL["STATE"]["WDOT_0"]

	ENUPOS = np.zeros(3)
	ENUPOS[0] = MSL["STATE"]["ENUPOSX"]
	ENUPOS[1] = MSL["STATE"]["ENUPOSY"]
	ENUPOS[2] = MSL["STATE"]["ENUPOSZ"]
	ENUVEL = np.zeros(3)
	ENUVEL[0] = MSL["STATE"]["ENUVELX"]
	ENUVEL[1] = MSL["STATE"]["ENUVELY"]
	ENUVEL[2] = MSL["STATE"]["ENUVELZ"]
	ENUEULER = np.zeros(3)
	ENUEULER[0] = MSL["STATE"]["ENUPHI"]
	ENUEULER[1] = MSL["STATE"]["ENUTHT"]
	ENUEULER[2] = MSL["STATE"]["ENUPSI"]

	# INTEGRATION STATE. ###############################################################################
	INTEGRATION_PASS = 0
	STATE_P0 = ENUPOS
	STATE_V0 = ENUVEL
	STATE_E0 = ENUEULER
	STATE_EDOT0 = BODYRATE
	V1 = np.zeros(3)
	A1 = np.zeros(3)
	EDOT1 = np.zeros(3)
	EDOTDOT1 = np.zeros(3)
	V2 = np.zeros(3)
	A2 = np.zeros(3)
	EDOT2 = np.zeros(3)
	EDOTDOT2 = np.zeros(3)
	V3 = np.zeros(3)
	A3 = np.zeros(3)
	EDOT3 = np.zeros(3)
	EDOTDOT3 = np.zeros(3)
	V4 = np.zeros(3)
	A4 = np.zeros(3)
	EDOT4 = np.zeros(3)
	EDOTDOT4 = np.zeros(3)

	# AIRFRAME. ###############################################################################
	# x = SphericalNoseDrag()
	# CD_LOOKUP = x.getCDValues()
	# MACH_LOOKUP = x.getMachValues()
	CD_LOOKUP = [0.1, 0.6]
	MACH_LOOKUP = [0.6, 2.5]
	MM_TO_M = 1.0 / 1000.0
	REFERENCE_DIAMETER = 0.18 # Meters.
	REFERENCE_LENGTH = 1.6 # Meters. # REFERENCE_LENGTH = 1.85026 # Meters.
	REFERENCE_AREA = np.pi * (REFERENCE_DIAMETER ** 2) / 4 # Meters squared.
	WING_HALF_SPAN = 66.1175 * MM_TO_M / 2.0 # Meters.
	WING_TIP_CHORD = 91.047 * MM_TO_M # Meters.
	WING_ROOT_CHORD = 0.123564 # Meters.
	WING_AREA = 0.5 * WING_HALF_SPAN * (WING_TIP_CHORD + WING_ROOT_CHORD) # Meters squared.
	TAIL_HALF_SPAN = 71.3548 * MM_TO_M / 2.0 # Meters.
	TAIL_TIP_CHORD = 0.387894 # Meters.
	TAIL_ROOT_CHORD = 0.48084 # Meters.
	TAIL_AREA = 0.5 * TAIL_HALF_SPAN * (TAIL_TIP_CHORD + TAIL_ROOT_CHORD) # Meters squared.
	NOSE_LENGTH = 0.249733 # Meters.
	# NOSE_AREA = NOSE_LENGTH * REFERENCE_DIAMETER # Meters squared.
	DISTANCE_FROM_BASE_OF_NOSE_TO_WING = 0.323925 # Meters.
	CENTER_OF_DEFLECTION_FROM_NOSE = 1.8059 - NOSE_LENGTH # Meters # CENTER_OF_DEFLECTION_FROM_NOSE = 1.8059 # Meters.
	PLANFORM_AREA = (REFERENCE_LENGTH - NOSE_LENGTH) * REFERENCE_DIAMETER + 0.667 * NOSE_LENGTH * REFERENCE_DIAMETER # Meters squared.
	NOSE_CENTER_OF_PRESSURE = 0.67 * NOSE_LENGTH # Meters.
	WING_CENTER_OF_PRESSURE = NOSE_LENGTH + DISTANCE_FROM_BASE_OF_NOSE_TO_WING + 0.7 * WING_ROOT_CHORD - 0.2 * WING_TIP_CHORD # Meters.
	AN = 0.67 * NOSE_LENGTH * REFERENCE_DIAMETER # Meters squared.
	AB = (REFERENCE_LENGTH - NOSE_LENGTH) * REFERENCE_DIAMETER # Meters squared.
	BODY_CENTER_OF_PRESSURE = (0.67 * AN * NOSE_LENGTH + AB * (NOSE_LENGTH + 0.5 * (REFERENCE_LENGTH - NOSE_LENGTH))) / (AN + AB) # Meters.

	# FUNCTION FOR RETURNING DATA DICTIONARY. ###############################################################################
	def populateState():

		STATE = {

			# MASS AND MOTOR PROPERTIES.
			"XCG": XCG,
			"MASS": MASS,
			"TMOI": TMOI,
			"THRUST": THRUST,

			# ATMOSPHERE.
			"RHO": RHO,
			"Q": Q,
			"P": P,
			"A": A,
			"G": G,
			"MACH": MACH,
			"BETA": BETA,

			# MISSILE BODY.
			"TOF": TOF,
			"SPEED": SPEED,
			"ALPHA": ALPHA,
			"SIDESLIP": SIDESLIP,
			"PRATE": BODYRATE[0],
			"QRATE": BODYRATE[1],
			"RRATE": BODYRATE[2],
			"UDOT_0": SPECIFIC_FORCE[0],
			"VDOT_0": SPECIFIC_FORCE[1],
			"WDOT_0": SPECIFIC_FORCE[2],

			# COORDINATE FRAME.
			"ENUPOSX": ENUPOS[0],
			"ENUPOSY": ENUPOS[1],
			"ENUPOSZ": ENUPOS[2],
			"ENUVELX": ENUVEL[0],
			"ENUVELY": ENUVEL[1], 
			"ENUVELZ": ENUVEL[2], 
			"ENUPHI": ENUEULER[0],
			"ENUTHT": ENUEULER[1],
			"ENUPSI": ENUEULER[2],

		}

		return STATE

	# LOOP. ###############################################################################
	GO = True
	while GO:

		# ATMOSPHERE.
		MSL["ATMOS"].update(ENUPOS[2], SPEED)
		RHO = MSL["ATMOS"].rho # Kilograms per meter cubed.
		Q = MSL["ATMOS"].q # Pascals.
		P = MSL["ATMOS"].p # Pascals.
		A = MSL["ATMOS"].a # Meters per second.
		G = MSL["ATMOS"].g # Meters per second squared.
		MACH = MSL["ATMOS"].mach # Non dimensional.
		BETA = None # "Normalized Speed" - Zarchan. I don't know what this is. Can't find it anywhere else.
		if MACH > 1:
			BETA = np.sqrt(MACH ** 2 - 1) # Non dimensional.
		else:
			BETA = MACH # Non dimensional.

		# MASS AND MOTOR PROPERTIES.
		MSL["MASS_AND_MOTOR"].update(TOF, P)
		XCG = MSL["MASS_AND_MOTOR"].XCG
		MASS = MSL["MASS_AND_MOTOR"].MASS
		TMOI = MSL["MASS_AND_MOTOR"].TRANSVERSE_MOI
		THRUST = MSL["MASS_AND_MOTOR"].THRUST

		# ATTITUDE.
		ENU_TO_FLU = ct.ORIENTATION_TO_LOCAL_TM(ENUEULER[0], -1.0 * ENUEULER[1], ENUEULER[2])
		SPEED = la.norm(ENUVEL)
		VEL_B = ENU_TO_FLU @ ENUVEL
		TEMP1, TEMP2 = returnAlphaAndBeta(VEL_B)

		# ALPHA AND BETA IN FORWARD, RIGHT, DOWN.
		ALPHA = -1.0 * TEMP1
		SIDESLIP = -1.0 * TEMP2

		# BASIC DRAG MODEL.
		CD = None # Non dimensional.
		if MACH >= MACH_LOOKUP[0]:
			CD = linearInterpolation(MACH, MACH_LOOKUP, CD_LOOKUP)
		else:
			CD = CD_LOOKUP[0]
		DRAG_FORCE = CD * REFERENCE_AREA * Q # Newtons.
		WIND_TO_BODY = ct.FLIGHTPATH_TO_LOCAL_TM(SIDESLIP, ALPHA) # Non dimensional.
		WIND_DRAG_FORCE = npa([-DRAG_FORCE, 0.0, 0.0]) # Newtons.
		BODY_DRAG = (WIND_TO_BODY @ WIND_DRAG_FORCE) / MASS # Meters per second squared.

		# AERODYNAMICS.
		CY = 2 * SIDESLIP + \
			(1.5 * PLANFORM_AREA * SIDESLIP * SIDESLIP) / REFERENCE_AREA + \
			(8 * WING_AREA * SIDESLIP) / (BETA * REFERENCE_AREA) + \
			(8 * TAIL_AREA * (SIDESLIP + YAW_FIN_DEFL_RAD)) / (BETA * REFERENCE_AREA)
		CN = 2 * SIDESLIP * ((XCG - NOSE_CENTER_OF_PRESSURE) / REFERENCE_DIAMETER) + \
			((1.5 * PLANFORM_AREA * SIDESLIP * SIDESLIP) / REFERENCE_AREA) * \
			((XCG - BODY_CENTER_OF_PRESSURE) / REFERENCE_DIAMETER) + \
			((8 * WING_AREA * SIDESLIP) / (BETA * REFERENCE_AREA)) * \
			((XCG - WING_CENTER_OF_PRESSURE) / REFERENCE_DIAMETER) + \
			((8 * TAIL_AREA * (SIDESLIP + YAW_FIN_DEFL_RAD)) / \
			(BETA * REFERENCE_AREA)) * ((XCG - CENTER_OF_DEFLECTION_FROM_NOSE) / REFERENCE_DIAMETER)

		CZ = 2 * ALPHA + \
			(1.5 * PLANFORM_AREA * ALPHA * ALPHA) / REFERENCE_AREA + \
			(8 * WING_AREA * ALPHA) / (BETA * REFERENCE_AREA) + \
			(8 * TAIL_AREA * (ALPHA + PITCH_FIN_DEFL_RAD)) / (BETA * REFERENCE_AREA)
		CM = 2 * ALPHA * ((XCG - NOSE_CENTER_OF_PRESSURE) / REFERENCE_DIAMETER) + \
			((1.5 * PLANFORM_AREA * ALPHA * ALPHA) / REFERENCE_AREA) * \
			((XCG - BODY_CENTER_OF_PRESSURE) / REFERENCE_DIAMETER) + \
			((8 * WING_AREA * ALPHA) / (BETA * REFERENCE_AREA)) * \
			((XCG - WING_CENTER_OF_PRESSURE) / REFERENCE_DIAMETER) + \
			((8 * TAIL_AREA * (ALPHA + PITCH_FIN_DEFL_RAD)) / \
			(BETA * REFERENCE_AREA)) * ((XCG - CENTER_OF_DEFLECTION_FROM_NOSE) / REFERENCE_DIAMETER)

		# DERIVATIVES.
		EDOTDOT_0 = np.zeros(3)
		EDOTDOT_0[0] = 0.0
		EDOTDOT_0[1] = (Q * REFERENCE_AREA * REFERENCE_DIAMETER * CM) / TMOI
		EDOTDOT_0[2] = (Q * REFERENCE_AREA * REFERENCE_DIAMETER * CN) / TMOI
		SPECIFIC_FORCE[0] = THRUST / MASS
		SPECIFIC_FORCE[1] = (Q * REFERENCE_AREA * CY) / MASS
		SPECIFIC_FORCE[2] = (Q * REFERENCE_AREA * CZ) / MASS
		LOCAL_G = npa([0.0, 0.0, -1.0 * G])
		BODY_G = ENU_TO_FLU @ LOCAL_G
		SPECIFIC_FORCE += (BODY_G + BODY_DRAG)
		ACC_0 = SPECIFIC_FORCE @ ENU_TO_FLU

		# STATE.
		if INTEGRATION_PASS == 0:

			# LOG DATA.
			MSL["STATE"] = populateState()
			lf.writeData(MSL["STATE"], MSL["LOGFILE"] )

			# END CHECK.
			if TOF > MAX_TIME:
				MSL["LETHALITY"] = "MAX_TIME"
				return MSL
			if ENUPOS[2] < 0.0:
				print(f"GROUND - TOF : {TOF:.2f}, ENU : {ENUPOS}, MACH : {MACH:.2f}")
				MSL["LETHALITY"] = "GROUND"
				return MSL
			if np.isnan(np.sum(ENUPOS)):
				print(f"NAN - TOF : {TOF:.2f}, ENU : {ENUPOS}, MACH : {MACH:.2f}")
				MSL["LETHALITY"] = "NAN"
				return MSL

			# BEGIN INTEGRATION PASS.
			STATE_P0 = ENUPOS
			STATE_V0 = ENUVEL
			STATE_E0 = ENUEULER
			STATE_EDOT0 = BODYRATE

			V1 = ENUVEL
			A1 = ACC_0
			EDOT1 = BODYRATE
			EDOTDOT1 = EDOTDOT_0

			ENUPOS = STATE_P0 + V1 * (TIME_STEP / 2.0)
			ENUVEL = STATE_V0 + A1 * (TIME_STEP / 2.0)
			ENUEULER = STATE_E0 + EDOT1 * (TIME_STEP / 2.0)
			BODYRATE = STATE_EDOT0 + EDOTDOT1 * (TIME_STEP / 2.0)

			TOF += (TIME_STEP / 2.0)

			INTEGRATION_PASS += 1

		elif INTEGRATION_PASS == 1:

			V2 = ENUVEL
			A2 = ACC_0
			EDOT2 = BODYRATE
			EDOTDOT2 = EDOTDOT_0

			ENUPOS = STATE_P0 + V2 * (TIME_STEP / 2.0)
			ENUVEL = STATE_V0 + A2 * (TIME_STEP / 2.0)
			ENUEULER = STATE_E0 + EDOT2 * (TIME_STEP / 2.0)
			BODYRATE = STATE_EDOT0 + EDOTDOT2 * (TIME_STEP / 2.0)

			INTEGRATION_PASS += 1

		elif INTEGRATION_PASS == 2:

			V3 = ENUVEL
			A3 = ACC_0
			EDOT3 = BODYRATE
			EDOTDOT3 = EDOTDOT_0

			ENUPOS = STATE_P0 + V3 * (TIME_STEP)
			ENUVEL = STATE_V0 + A3 * (TIME_STEP)
			ENUEULER = STATE_E0 + EDOT3 * (TIME_STEP)
			BODYRATE = STATE_EDOT0 + EDOTDOT3 * (TIME_STEP)

			TOF += (TIME_STEP / 2.0)

			INTEGRATION_PASS += 1

		elif INTEGRATION_PASS == 3:

			V4 = ENUVEL
			A4 = ACC_0
			EDOT4 = BODYRATE
			EDOTDOT4 = EDOTDOT_0

			ENUPOS = STATE_P0 + (TIME_STEP / 6.0) * (V1 + 2 * V2 + 2 * V3 + V4)
			ENUVEL = STATE_V0 + (TIME_STEP / 6.0) * (A1 + 2 * A2 + 2 * A3 + A4)
			ENUEULER = STATE_E0 + (TIME_STEP / 6.0) * (EDOT1 + 2 * EDOT2 + 2 * EDOT3 + EDOT4)
			BODYRATE = STATE_EDOT0 + (TIME_STEP / 6.0) * (EDOTDOT1 + 2 * EDOTDOT2 + 2 * EDOTDOT3 + EDOTDOT4)

			INTEGRATION_PASS = 0



if __name__ == "__main__":

	wallClockStart = time.time()

	# FIVEDOF
	POS0 = np.zeros(3)
	AZ0 = 0
	EL0 = 45
	SPD0 = 10
	MSL = Construct5DOFMissile(POS0, AZ0, EL0, SPD0, "MOCK_HELLFIRE5DOF")
	MSL = Fly5DOF(MSL, 100, -4.0, 0.0)

	wallClockEnd = time.time()
	print(f"RUN TIME : {wallClockEnd - wallClockStart} SECONDS")
