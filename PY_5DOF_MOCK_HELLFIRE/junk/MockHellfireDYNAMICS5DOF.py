
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
	LLA_0,
	ENU_POS_0,
	ENU_AZ_0,
	ENU_EL_0,
	SPD_0,
	ID
):

	# ATMOSPHERE. ###############################################################################
	ATMOS = ATM1976()
	ATMOS.update(ENU_POS_0[1], la.norm(SPD_0))
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
	
	# GEODETIC.
	LLA = np.zeros(3)
	LLA = npa([np.radians(LLA_0[0]), np.radians(LLA_0[1]), LLA_0[2]])

	# ENU.
	ENUPOS = np.zeros(3)
	ENUVEL = np.zeros(3)
	ENUEULER = np.zeros(3)
	ENU_TO_FLU = np.zeros((3, 3))

	INITIAL_AZ = np.radians(ENU_AZ_0) # RADIANS.
	INITIAL_EL = np.radians(ENU_EL_0) # RADIANS.
	ENU_TO_FLU = ct.ORIENTATION_TO_LOCAL_TM(0.0, -INITIAL_EL, INITIAL_AZ)
	ENUPOS = ENU_POS_0 # Meters.
	ENUVEL = SPD_0 * (ENU_TO_FLU[0])
	ENUEULER = npa([0.0, INITIAL_EL, INITIAL_AZ]) # Radians.

	# BODY.
	TOF = 0.0 # Seconds.
	SPEED = la.norm(SPD_0 * (ENU_TO_FLU[0])) # Meters per second.
	VEL_B = ENU_TO_FLU @ (SPD_0 * (ENU_TO_FLU[0])) # Body velocity.
	ALPHA, SIDESLIP = returnAlphaAndBeta(VEL_B)
	SPECIFIC_FORCE = np.zeros(3) # Meters per second squared.
	BODYRATE = np.zeros(3) # Radians per second.

	# ECEF.
	ECEFPOS0 = np.zeros(3)
	ECEFPOS = np.zeros(3)
	ECEFVEL = np.zeros(3)
	ECEFEULER = np.zeros(3)
	ECEF_TO_ENU = np.zeros((3, 3))
	ECEF_TO_FLU = np.zeros((3, 3))

	ECEFPOS = et.LLA_TO_ECI(LLA, 0.0) # Can use this function for initial earth position.
	ECEFPOS0 = ECEFPOS
	ECEF_TO_ENU = ct.ORIENTATION_TO_LOCAL_TM(
		0.0,
		(np.pi / 2.0) + LLA[0],
		LLA[1]
	)
	ECEF_TO_FLU = ENU_TO_FLU @ ECEF_TO_ENU
	ECEFEULER = ct.EULER_FROM_DCM(ECEF_TO_FLU)
	ECEFVEL = VEL_B @ ECEF_TO_FLU

	# ECI.
	ECIPOS = np.zeros(3)
	ECIVEL = np.zeros(3)
	ECIEULER = np.zeros(3)
	ECI_TO_ECEF = np.zeros((3, 3))
	ECI_TO_FLU = np.zeros((3, 3))

	ECIPOS = et.LLA_TO_ECI(LLA, 0.0)
	ECI_TO_ECEF = et.ECI_TO_ECEF_TM(0.0)
	TEMP = ECI_TO_ECEF.transpose() @ ECEFVEL
	WEII3 = 7.292115e-5 # Rotation speed of earth. Radians per second.
	OMEGA = npa([0.0, 0.0, WEII3])
	ECIVEL_DUE_TO_ROTATION = np.cross(OMEGA, ECIPOS)
	ECIVEL = TEMP + ECIVEL_DUE_TO_ROTATION
	ECI_TO_FLU = ECEF_TO_FLU @ ECI_TO_ECEF
	ECIEULER = ct.EULER_FROM_DCM(ECI_TO_FLU)

	# DATA. ###############################################################################
	MISSILE = {

		"IDENTITY": ID,
		"LOGFILE": open(f"PY_5DOF_MOCK_HELLFIRE/output/{ID}.txt", "w"),
		"LETHALITY": "FLYING",
		"ATMOS": ATMOS,
		"MASS_AND_MOTOR": MASS_AND_MOTOR,
		"FRAMES": {

			# LLA.
			"LLA": LLA,

			# ECEF.
			"ECEFPOS0": ECEFPOS0,
			"ECEFPOS": ECEFPOS,
			"ECEFVEL": ECEFVEL,
			"ECEFEULER": ECEFEULER,
			"ECEF_TO_ENU": ECEF_TO_ENU,
			"ECEF_TO_FLU": ECEF_TO_FLU,

			# ECI.
			"ECIPOS": ECIPOS,
			"ECIVEL": ECIVEL,
			"ECIEULER": ECIEULER,
			"ECI_TO_ECEF": ECI_TO_ECEF,
			"ECI_TO_FLU": ECI_TO_FLU

		},
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

			# FRAMES. #
			# ENU.
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
	YAW_FIN_DEFL_DEG_INPUT,
	ROTATING_EARTH_FLAG
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

	LLA = MSL["FRAMES"]["LLA"]

	ECEFPOS0 = MSL["FRAMES"]["ECEFPOS0"]
	ECEFPOS = MSL["FRAMES"]["ECEFPOS"]
	ECEFVEL = MSL["FRAMES"]["ECEFVEL"]
	ECEFEULER = MSL["FRAMES"]["ECEFEULER"]
	ECEF_TO_ENU = MSL["FRAMES"]["ECEF_TO_ENU"]
	ECEF_TO_FLU = MSL["FRAMES"]["ECEF_TO_FLU"]

	ECIPOS = MSL["FRAMES"]["ECIPOS"]
	ECIVEL = MSL["FRAMES"]["ECIVEL"]
	ECIEULER = MSL["FRAMES"]["ECIEULER"]
	ECI_TO_ECEF = MSL["FRAMES"]["ECI_TO_ECEF"]
	ECI_TO_FLU = MSL["FRAMES"]["ECI_TO_FLU"]

	# INTEGRATION STATE. ###############################################################################
	INTEGRATION_PASS = 0
	if not ROTATING_EARTH_FLAG:
		STATE_P0 = ENUPOS
		STATE_V0 = ENUVEL
		STATE_E0 = ENUEULER
		STATE_WDOT0 = BODYRATE
	else:
		STATE_P0 = ECIPOS
		STATE_V0 = ECIVEL
		STATE_E0 = ECIEULER
		STATE_WDOT0 = BODYRATE

	V1 = np.zeros(3)
	A1 = np.zeros(3)
	BODYRATE1 = np.zeros(3)
	BODYRATEDOT1 = np.zeros(3)

	V2 = np.zeros(3)
	A2 = np.zeros(3)
	BODYRATE2 = np.zeros(3)
	BODYRATEDOT2 = np.zeros(3)

	V3 = np.zeros(3)
	A3 = np.zeros(3)
	BODYRATE3 = np.zeros(3)
	BODYRATEDOT3 = np.zeros(3)

	V4 = np.zeros(3)
	A4 = np.zeros(3)
	BODYRATE4 = np.zeros(3)
	BODYRATEDOT4 = np.zeros(3)

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

	def populateFrames():

		FRAMES = {

			# LLA.
			"LLA": LLA,

			# ECEF.
			"ECEFPOS0": ECEFPOS0,
			"ECEFPOS": ECEFPOS,
			"ECEFVEL": ECEFVEL,
			"ECEFEULER": ECEFEULER,
			"ECEF_TO_ENU": ECEF_TO_ENU,
			"ECEF_TO_FLU": ECEF_TO_FLU,

			# ECI.
			"ECIPOS": ECIPOS,
			"ECIVEL": ECIVEL,
			"ECIEULER": ECIEULER,
			"ECI_TO_ECEF": ECI_TO_ECEF,
			"ECI_TO_FLU": ECI_TO_FLU

		}

		return FRAMES

	# LOOP. ###############################################################################
	GO = True
	LAST_TIME = TOF
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

		if ROTATING_EARTH_FLAG:

			# Rotation rate of earth.
			WEII3 = 7.292115e-5 # Rotation speed of earth. Radians per second.
			OMEGA = npa([0.0, 0.0, WEII3])

			# Eci gravity.
			GEODETICGRAV = et.GEODETIC_GRAV(ECIPOS, TOF)
			LLA_TO_ECI_TM = et.LLA_TO_ECI_TM(LLA, TOF)
			ECIGRAV = LLA_TO_ECI_TM.transpose() @ GEODETICGRAV

			# Pseudo forces. No Euler force because the rotation rate of earth is assumed to be constant.
			# CORIOLIS_FORCE = -2.0 * MASS * (np.cross(OMEGA, ECIVEL))
			# TEMP1 = -1.0 * MASS * OMEGA
			# TEMP2 = np.cross(OMEGA, ECIPOS)
			# CENTRIFUGAL_FORCE = np.cross(TEMP1, TEMP2)

			# Body rate dot and specific force.
			BODYRATEDOT = np.zeros(3)
			BODYRATEDOT[0] = 0.0
			BODYRATEDOT[1] = (Q * REFERENCE_AREA * REFERENCE_DIAMETER * CM) / TMOI
			BODYRATEDOT[2] = (Q * REFERENCE_AREA * REFERENCE_DIAMETER * CN) / TMOI
			SPECIFIC_FORCE[0] = THRUST / MASS
			SPECIFIC_FORCE[1] = (Q * REFERENCE_AREA * CY) / MASS
			SPECIFIC_FORCE[2] = (Q * REFERENCE_AREA * CZ) / MASS

			# Derivative calculated in ECI.
			ACC_0 = \
			(SPECIFIC_FORCE @ ECI_TO_FLU) + \
			ECIGRAV # + \
			# CORIOLIS_FORCE + \
			# CENTRIFUGAL_FORCE

			# STATE.
			if INTEGRATION_PASS == 0:

				# LOG DATA.
				MSL["FRAMES"] = populateFrames()
				MSL["STATE"] = populateState()
				lf.writeData(MSL["STATE"], MSL["LOGFILE"] )

				# CONSOLE REPORT.
				CHECK = round(TOF, 3).is_integer()
				if CHECK and LAST_TIME != TOF:
					print(f"TOF {TOF:.0f} ENU {ENUPOS}")
					LAST_TIME = TOF

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
				STATE_P0 = ECIPOS
				STATE_V0 = ECIVEL
				STATE_E0 = ECIEULER
				STATE_WDOT0 = BODYRATE

				V1 = ECIVEL
				A1 = ACC_0
				BODYRATE1 = BODYRATE
				BODYRATEDOT1 = BODYRATEDOT

				ECIPOS = STATE_P0 + V1 * (TIME_STEP / 1.0)
				ECIVEL = STATE_V0 + A1 * (TIME_STEP / 1.0)
				ECIEULER = STATE_E0 + BODYRATE1 * (TIME_STEP / 1.0)
				BODYRATE = STATE_WDOT0 + BODYRATEDOT1 * (TIME_STEP / 1.0)

				TOF += (TIME_STEP / 1.0)

				INTEGRATION_PASS = 0

			# STATE UPDATED IN THIS FUNCTION. IN ORDER.
			# ECI_TO_ECEF = np.zeros((3, 3))
			# GEODETIC = np.zeros(3)
			# ECEFPOS = np.zeros(3)
			# ECEFVEL = np.zeros(3)
			# ECEF_TO_ENU = np.zeros((3, 3))
			# ENUPOS = np.zeros(3)
			# ENUVEL = np.zeros(3)
			# ENU_TO_FLU = np.zeros((3, 3))
			# ECEF_TO_FLU = np.zeros((3, 3))
			# ECI_TO_FLU = np.zeros((3, 3))

			# ECI TO ECEF MATRIX.
			ECI_TO_ECEF = et.ECI_TO_ECEF_TM(TOF)
			OMEGA = npa([0.0, 0.0, WEII3])
			ECIVEL_DUE_TO_ROTATION = np.cross(OMEGA, ECIPOS)

			# LLA STATE.
			LLA = et.ECI_TO_LLA(ECIPOS, TOF)

			# ECEF STATE AND ECEF TO ENU MATRIX.
			ECEFPOS = ECI_TO_ECEF @ ECIPOS
			ECEFVEL = ECI_TO_ECEF @ (ECIVEL - ECIVEL_DUE_TO_ROTATION)
			ECEF_TO_ENU = ct.ORIENTATION_TO_LOCAL_TM(
				0.0,
				(np.pi / 2.0) + LLA[0],
				LLA[1]
			)

			# ENU STATE AND ENU TO BODY MATRIX..
			ENUPOS = ECEF_TO_ENU @ (ECEFPOS - ECEFPOS0)
			ENUVEL = ECEF_TO_ENU @ ECEFVEL
			AZ, EL = returnAlphaAndBeta(ENUVEL)
			ENU_TO_FLU = ct.FLIGHTPATH_TO_LOCAL_TM(AZ, -EL)
			ENUEULER = npa([0.0, EL, AZ])

			# ECEF TO BODY MATRIX.
			ECEF_TO_FLU = ENU_TO_FLU @ ECEF_TO_ENU
			ECEFEULER = ct.EULER_FROM_DCM(ECEF_TO_FLU)

			# ECI TO BODY MATRIX.
			ECI_TO_FLU = ECEF_TO_FLU @ ECI_TO_ECEF

		elif not ROTATING_EARTH_FLAG:

			# DERIVATIVES.
			BODYRATEDOT = np.zeros(3)
			BODYRATEDOT[0] = 0.0
			BODYRATEDOT[1] = (Q * REFERENCE_AREA * REFERENCE_DIAMETER * CM) / TMOI
			BODYRATEDOT[2] = (Q * REFERENCE_AREA * REFERENCE_DIAMETER * CN) / TMOI
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
				MSL["FRAMES"] = populateFrames()
				MSL["STATE"] = populateState()
				lf.writeData(MSL["STATE"], MSL["LOGFILE"] )

				# CONSOLE REPORT.
				CHECK = round(TOF, 3).is_integer()
				if CHECK and LAST_TIME != TOF:
					print(f"TOF {TOF:.0f} ENU {ENUPOS}")
					LAST_TIME = TOF

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
				STATE_WDOT0 = BODYRATE

				V1 = ENUVEL
				A1 = ACC_0
				BODYRATE1 = BODYRATE
				BODYRATEDOT1 = BODYRATEDOT

				ENUPOS = STATE_P0 + V1 * (TIME_STEP / 2.0)
				ENUVEL = STATE_V0 + A1 * (TIME_STEP / 2.0)
				ENUEULER = STATE_E0 + BODYRATE1 * (TIME_STEP / 2.0)
				BODYRATE = STATE_WDOT0 + BODYRATEDOT1 * (TIME_STEP / 2.0)

				TOF += (TIME_STEP / 2.0)

				INTEGRATION_PASS += 1

			elif INTEGRATION_PASS == 1:

				V2 = ENUVEL
				A2 = ACC_0
				BODYRATE2 = BODYRATE
				BODYRATEDOT2 = BODYRATEDOT

				ENUPOS = STATE_P0 + V2 * (TIME_STEP / 2.0)
				ENUVEL = STATE_V0 + A2 * (TIME_STEP / 2.0)
				ENUEULER = STATE_E0 + BODYRATE2 * (TIME_STEP / 2.0)
				BODYRATE = STATE_WDOT0 + BODYRATEDOT2 * (TIME_STEP / 2.0)

				INTEGRATION_PASS += 1

			elif INTEGRATION_PASS == 2:

				V3 = ENUVEL
				A3 = ACC_0
				BODYRATE3 = BODYRATE
				BODYRATEDOT3 = BODYRATEDOT

				ENUPOS = STATE_P0 + V3 * (TIME_STEP)
				ENUVEL = STATE_V0 + A3 * (TIME_STEP)
				ENUEULER = STATE_E0 + BODYRATE3 * (TIME_STEP)
				BODYRATE = STATE_WDOT0 + BODYRATEDOT3 * (TIME_STEP)

				TOF += (TIME_STEP / 2.0)

				INTEGRATION_PASS += 1

			elif INTEGRATION_PASS == 3:

				V4 = ENUVEL
				A4 = ACC_0
				BODYRATE4 = BODYRATE
				BODYRATEDOT4 = BODYRATEDOT

				ENUPOS = STATE_P0 + (TIME_STEP / 6.0) * (V1 + 2 * V2 + 2 * V3 + V4)
				ENUVEL = STATE_V0 + (TIME_STEP / 6.0) * (A1 + 2 * A2 + 2 * A3 + A4)
				ENUEULER = STATE_E0 + (TIME_STEP / 6.0) * (BODYRATE1 + 2 * BODYRATE2 + 2 * BODYRATE3 + BODYRATE4)
				BODYRATE = STATE_WDOT0 + (TIME_STEP / 6.0) * (BODYRATEDOT1 + 2 * BODYRATEDOT2 + 2 * BODYRATEDOT3 + BODYRATEDOT4)

				INTEGRATION_PASS = 0



if __name__ == "__main__":

	wallClockStart = time.time()

	# FIVEDOF
	MSL = Construct5DOFMissile(
		LLA_0=npa([38.8719, 77.0563, 0.0]),
		ENU_POS_0=np.zeros(3),
		ENU_AZ_0=20.0,
		ENU_EL_0=45.0,
		SPD_0=25.0,
		ID="MOCK_HELLFIRE5DOF_1"
	)
	MSL = Fly5DOF(
		MISSILE_INPUT_DICT=MSL,
		FLY_FOR_THIS_LONG=100.0,
		PITCH_FIN_DEFL_DEG_INPUT=-2.0,
		YAW_FIN_DEFL_DEG_INPUT=0.0,
		ROTATING_EARTH_FLAG=False
	)

	MSL = Construct5DOFMissile(
		LLA_0=npa([38.8719, 77.0563, 0.0]),
		ENU_POS_0=np.zeros(3),
		ENU_AZ_0=20.0,
		ENU_EL_0=45.0,
		SPD_0=25.0,
		ID="MOCK_HELLFIRE5DOF_2"
	)
	MSL = Fly5DOF(
		MISSILE_INPUT_DICT=MSL,
		FLY_FOR_THIS_LONG=100.0,
		PITCH_FIN_DEFL_DEG_INPUT=-2.0,
		YAW_FIN_DEFL_DEG_INPUT=0.0,
		ROTATING_EARTH_FLAG=True
	)

	wallClockEnd = time.time()
	print(f"RUN TIME : {wallClockEnd - wallClockStart} SECONDS")
