# Python libraries.
import time
import copy
import numpy as np
from numpy import array as npa
from numpy import linalg as la
np.set_printoptions(suppress=True, precision=2)

# Utility.
from utility.returnAzAndElevation import returnAlphaAndBeta
from utility.returnAzAndElevation import returnAzAndElevation
from utility import coordinateTransformations as ct
from utility.interpolationGivenTwoVectors import linearInterpolation
import utility.loggingFxns as lf

# Classes.
from classes.ATM1976 import ATM1976
from classes.MockHellfireMassAndMotor import MockHellfireMassAndMotor

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

# CONSTANTS.
WEII3 = 7.292115e-5 # Rotation speed of earth. Radians per second.
REARTH = 6370987.308 # Meters.
SMALL = 9.999999999999999547e-08
DEG_TO_RAD = 0.01745329251994319833

# Three degrees of freedom. Right, up, and pitch.
def Construct5DOFMissile(
	INITIAL_LLA,
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
	# BODY BEGINS UNPERTURBED
	TOF = 0.0 # Seconds.
	SPEED = INITIAL_AIRSPEED # Meters per second.
	SPECIFIC_FORCE = np.zeros(3) # Meters per second squared.
	ALPHA = 0.0
	SIDESLIP = 0.0
	BODY_RATE= np.zeros(3)

	# LLA.
	LLA = npa([np.radians(INITIAL_LLA[0]), np.radians(INITIAL_LLA[1]), INITIAL_LLA[2]])

	# ENU.
	ENUPOS = np.zeros(3)
	INITIAL_AZ = np.radians(INITIAL_AZIMUTH) # RADIANS.
	INITIAL_EL = np.radians(INITIAL_ELEVATION) # RADIANS.
	ENU_TO_FLU = ct.ORIENTATION_TO_LOCAL_TM(0.0, -INITIAL_EL, INITIAL_AZ)
	ENUVEL = INITIAL_AIRSPEED * (ENU_TO_FLU[0])
	ENUEULER = npa([0.0, INITIAL_EL, INITIAL_AZ])
	VEL_B = ENU_TO_FLU @ ENUVEL # METERS PER SECOND
	
	# ECEF.
	ECEFPOS = ct.LLA_TO_ECI(LLA, TOF) # Can use this function for initial earth position.
	ECEFPOS0 = ECEFPOS
	ECEF_TO_ENU = ct.ORIENTATION_TO_LOCAL_TM(
		0.0,
		(np.pi / 2.0) + LLA[0],
		LLA[1]
	)
	ECEF_TO_FLU = ENU_TO_FLU @ ECEF_TO_ENU
	ECEFVEL = VEL_B @ ECEF_TO_FLU
	ECEFEULER = ct.EULER_FROM_DCM(ECEF_TO_FLU)

	# ECI.
	ECIPOS = ct.LLA_TO_ECI(LLA, TOF)
	ECI_TO_ECEF = ct.ECI_TO_ECEF_TM(TOF)
	TEMP = ECI_TO_ECEF.transpose() @ ECEFVEL
	OMEGA = npa([0.0, 0.0, WEII3])
	ECIVEL_DUE_TO_ROTATION = np.cross(OMEGA, ECIPOS)
	ECIVEL = TEMP + ECIVEL_DUE_TO_ROTATION
	ECI_TO_FLU = ECEF_TO_FLU @ ECI_TO_ECEF
	ECIEULER = ct.EULER_FROM_DCM(ECI_TO_FLU)

	# DATA. ###############################################################################
	MISSILE = {

		"IDENTITY": ID,
		"LOGFILE": open(f"PY_5DOF_HIGH_FIDELITY_MOCK_HELLFIRE/output/{ID}.txt", "w"),
		"LETHALITY": "FLYING",
		"ATMOS": ATMOS,
		"MASS_AND_MOTOR": MASS_AND_MOTOR,
		"ECEFPOS0": ECEFPOS0,
		"TRANSFORMATION_MATRICES":{
			
			"ENU_TO_FLU": ENU_TO_FLU,
			"ECEF_TO_ENU": ECEF_TO_ENU,
			"ECEF_TO_FLU": ECEF_TO_FLU,
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
			"UDOT_0": SPECIFIC_FORCE[0],
			"VDOT_0": SPECIFIC_FORCE[1],
			"WDOT_0": SPECIFIC_FORCE[2],
			"ALPHA": ALPHA,
			"SIDESLIP": SIDESLIP,
			"PRATE": BODY_RATE[0],
			"QRATE": BODY_RATE[1],
			"RRATE": BODY_RATE[2],

			"LAT": LLA[0],
			"LON": LLA[1],
			"ALT": LLA[2],

			"ENUPOSX": ENUPOS[0],
			"ENUPOSY": ENUPOS[1],
			"ENUPOSZ": ENUPOS[2],
			"ENUVELX": ENUVEL[0],
			"ENUVELY": ENUVEL[1],
			"ENUVELZ": ENUVEL[2],
			"ENUPHI": ENUEULER[0],
			"ENUTHT": ENUEULER[1],
			"ENUPSI": ENUEULER[2],

			"ECEFPOSX": ECEFPOS[0],
			"ECEFPOSY": ECEFPOS[1],
			"ECEFPOSZ": ECEFPOS[2],
			"ECEFVELX": ECEFVEL[0],
			"ECEFVELY": ECEFVEL[1],
			"ECEFVELZ": ECEFVEL[2],
			"ECEFPHI": ECEFEULER[0],
			"ECEFTHT": ECEFEULER[1],
			"ECEFPSI": ECEFEULER[2],

			"ECIPOSX": ECIPOS[0],
			"ECIPOSY": ECIPOS[1],
			"ECIPOSZ": ECIPOS[2],
			"ECIVELX": ECIVEL[0],
			"ECIVELY": ECIVEL[1],
			"ECIVELZ": ECIVEL[2],
			"ECIPHI": ECIEULER[0],
			"ECITHT": ECIEULER[1],
			"ECIPSI": ECIEULER[2],

		}

	}

	lf.writeHeader(MISSILE["STATE"], MISSILE["LOGFILE"])
	lf.writeData(MISSILE["STATE"], MISSILE["LOGFILE"])

	return MISSILE

def Fly5DOF(
	MISSILE_INPUT_STATE_DICT,
	FLY_FOR_THIS_LONG,
	PITCH_FIN_DEFL_DEG_INPUT,
	YAW_FIN_DEFL_DEG_INPUT
):

	# CONSTANT INPUT. ###############################################################################
	MSL = copy.copy(MISSILE_INPUT_STATE_DICT)

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
	SPECIFIC_FORCE = np.zeros(3)
	SPECIFIC_FORCE[0] = MSL["STATE"]["UDOT_0"]
	SPECIFIC_FORCE[1] = MSL["STATE"]["VDOT_0"]
	SPECIFIC_FORCE[2] = MSL["STATE"]["WDOT_0"]
	ALPHA = MSL["STATE"]["ALPHA"]
	SIDESLIP = MSL["STATE"]["SIDESLIP"]
	BODY_RATE = np.zeros(3)
	BODY_RATE[0] = MSL["STATE"]["PRATE"]
	BODY_RATE[1] = MSL["STATE"]["QRATE"]
	BODY_RATE[2] = MSL["STATE"]["RRATE"]

	LLA = np.zeros(3)
	LLA[0] = MSL["STATE"]["LAT"]
	LLA[1] = MSL["STATE"]["LON"]
	LLA[2] = MSL["STATE"]["ALT"]

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

	ECEFPOS = np.zeros(3)
	ECEFPOS[0] = MSL["STATE"]["ECEFPOSX"]
	ECEFPOS[1] = MSL["STATE"]["ECEFPOSY"]
	ECEFPOS[2] = MSL["STATE"]["ECEFPOSZ"]
	ECEFVEL = np.zeros(3)
	ECEFVEL[0] = MSL["STATE"]["ECEFVELX"]
	ECEFVEL[1] = MSL["STATE"]["ECEFVELY"]
	ECEFVEL[2] = MSL["STATE"]["ECEFVELZ"]
	ECEFEULER = np.zeros(3)
	ECEFEULER[0] = MSL["STATE"]["ECEFPHI"]
	ECEFEULER[1] = MSL["STATE"]["ECEFTHT"]
	ECEFEULER[2] = MSL["STATE"]["ECEFPSI"]

	ECIPOS = np.zeros(3)
	ECIPOS[0] = MSL["STATE"]["ECIPOSX"]
	ECIPOS[1] = MSL["STATE"]["ECIPOSY"]
	ECIPOS[2] = MSL["STATE"]["ECIPOSZ"]
	ECIVEL = np.zeros(3)
	ECIVEL[0] = MSL["STATE"]["ECIVELX"]
	ECIVEL[1] = MSL["STATE"]["ECIVELY"]
	ECIVEL[2] = MSL["STATE"]["ECIVELZ"]
	ECIEULER = np.zeros(3)
	ECIEULER[0] = MSL["STATE"]["ECIPHI"]
	ECIEULER[1] = MSL["STATE"]["ECITHT"]
	ECIEULER[2] = MSL["STATE"]["ECIPSI"]

	ENU_TO_FLU = MSL["TRANSFORMATION_MATRICES"]["ENU_TO_FLU"]
	ECEF_TO_ENU = MSL["TRANSFORMATION_MATRICES"]["ECEF_TO_ENU"]
	ECEF_TO_FLU = MSL["TRANSFORMATION_MATRICES"]["ECEF_TO_FLU"]
	ECI_TO_ECEF = MSL["TRANSFORMATION_MATRICES"]["ECI_TO_ECEF"]
	ECI_TO_FLU = MSL["TRANSFORMATION_MATRICES"]["ECI_TO_FLU"]

	# INTEGRATION. ###############################################################################
	INTEGRATION_PASS = 0
	STATE_P0 = np.zeros(3)
	STATE_V0 = np.zeros(3)
	STATE_W0 = np.zeros(3)
	STATE_WDOT0 = np.zeros(3)

	V1 = np.zeros(3)
	A1 = np.zeros(3)
	WDOT1 = np.zeros(3)
	WDOTDOT1 = np.zeros(3)

	V2 = np.zeros(3)
	A2 = np.zeros(3)
	WDOT2 = np.zeros(3)
	WDOTDOT2 = np.zeros(3)

	V3 = np.zeros(3)
	A3 = np.zeros(3)
	WDOT3 = np.zeros(3)
	WDOTDOT3 = np.zeros(3)

	V4 = np.zeros(3)
	A4 = np.zeros(3)
	WDOT4 = np.zeros(3)
	WDOTDOT4 = np.zeros(3)

	# AIRFRAME. ###############################################################################
	CD_LOOKUP = [0.1, 0.5]
	MACH_LOOKUP = [0.5, 2.5]
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
			"UDOT_0": SPECIFIC_FORCE[0],
			"VDOT_0": SPECIFIC_FORCE[1],
			"WDOT_0": SPECIFIC_FORCE[2],
			"ALPHA": ALPHA,
			"SIDESLIP": SIDESLIP,
			"PRATE": BODY_RATE[0],
			"QRATE": BODY_RATE[1],
			"RRATE": BODY_RATE[2],

			"LAT": LLA[0],
			"LON": LLA[1],
			"ALT": LLA[2],

			"ENUPOSX": ENUPOS[0],
			"ENUPOSY": ENUPOS[1],
			"ENUPOSZ": ENUPOS[2],
			"ENUVELX": ENUVEL[0],
			"ENUVELY": ENUVEL[1],
			"ENUVELZ": ENUVEL[2],
			"ENUPHI": ENUEULER[0],
			"ENUTHT": ENUEULER[1],
			"ENUPSI": ENUEULER[2],

			"ECEFPOSX": ECEFPOS[0],
			"ECEFPOSY": ECEFPOS[1],
			"ECEFPOSZ": ECEFPOS[2],
			"ECEFVELX": ECEFVEL[0],
			"ECEFVELY": ECEFVEL[1],
			"ECEFVELZ": ECEFVEL[2],
			"ECEFPHI": ECEFEULER[0],
			"ECEFTHT": ECEFEULER[1],
			"ECEFPSI": ECEFEULER[2],

			"ECIPOSX": ECIPOS[0],
			"ECIPOSY": ECIPOS[1],
			"ECIPOSZ": ECIPOS[2],
			"ECIVELX": ECIVEL[0],
			"ECIVELY": ECIVEL[1],
			"ECIVELZ": ECIVEL[2],
			"ECIPHI": ECIEULER[0],
			"ECITHT": ECIEULER[1],
			"ECIPSI": ECIEULER[2],

		}

		return STATE

	# LOOP. ###############################################################################
	GO = True
	ROTATING_EARTH_FLAG = False
	while GO:

		# ATMOSPHERE.
		MSL["ATMOS"].update(LLA[2], SPEED)
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

		# BASIC DRAG MODEL. ###############################################################################
		CD = None # Non dimensional.
		if MACH >= MACH_LOOKUP[0]:
			CD = linearInterpolation(MACH, MACH_LOOKUP, CD_LOOKUP)
		else:
			CD = CD_LOOKUP[0]
		DRAG_FORCE = CD * REFERENCE_AREA * Q # Newtons.
		WIND_TO_BODY = ct.FLIGHTPATH_TO_LOCAL_TM(SIDESLIP, ALPHA) # Non dimensional.
		WIND_DRAG_FORCE = npa([-DRAG_FORCE, 0.0, 0.0]) # Newtons.
		BODY_DRAG = (WIND_TO_BODY @ WIND_DRAG_FORCE) / MASS # Meters per second squared.

		# ATTITUDE.
		SPEED = la.norm(ENUVEL)
		VEL_B = ENU_TO_FLU @ ENUVEL
		ALPHA, SIDESLIP = returnAlphaAndBeta(VEL_B)
		ALPHA *= -1
		SIDESLIP *= -1

		# AERODYNAMICS.
		CY = 2 * SIDESLIP + \
			(1.5 * PLANFORM_AREA * SIDESLIP * SIDESLIP) / REFERENCE_AREA + \
			(8 * WING_AREA * SIDESLIP) / (BETA * REFERENCE_AREA) + \
			(8 * TAIL_AREA * (SIDESLIP + YAW_FIN_DEFL_RAD)) / (BETA * REFERENCE_AREA)
		CN = 2 * SIDESLIP * ((XCG - NOSE_CENTER_OF_PRESSURE) / REFERENCE_DIAMETER) + \
			((1.5 * PLANFORM_AREA * SIDESLIP * SIDESLIP) / REFERENCE_AREA) * ((XCG - BODY_CENTER_OF_PRESSURE) / REFERENCE_DIAMETER) + \
			((8 * WING_AREA * SIDESLIP) / (BETA * REFERENCE_AREA)) * ((XCG - WING_CENTER_OF_PRESSURE) / REFERENCE_DIAMETER) + \
			((8 * TAIL_AREA * (SIDESLIP + YAW_FIN_DEFL_RAD)) / \
			(BETA * REFERENCE_AREA)) * ((XCG - CENTER_OF_DEFLECTION_FROM_NOSE) / REFERENCE_DIAMETER)
		CZ = 2 * ALPHA + \
			(1.5 * PLANFORM_AREA * ALPHA * ALPHA) / REFERENCE_AREA + \
			(8 * WING_AREA * ALPHA) / (BETA * REFERENCE_AREA) + \
			(8 * TAIL_AREA * (ALPHA + PITCH_FIN_DEFL_RAD)) / (BETA * REFERENCE_AREA)
		CM = 2 * ALPHA * ((XCG - NOSE_CENTER_OF_PRESSURE) / REFERENCE_DIAMETER) + \
			((1.5 * PLANFORM_AREA * ALPHA * ALPHA) / REFERENCE_AREA) * ((XCG - BODY_CENTER_OF_PRESSURE) / REFERENCE_DIAMETER) + \
			((8 * WING_AREA * ALPHA) / (BETA * REFERENCE_AREA)) * ((XCG - WING_CENTER_OF_PRESSURE) / REFERENCE_DIAMETER) + \
			((8 * TAIL_AREA * (ALPHA + PITCH_FIN_DEFL_RAD)) / \
			(BETA * REFERENCE_AREA)) * ((XCG - CENTER_OF_DEFLECTION_FROM_NOSE) / REFERENCE_DIAMETER)

		"""
		
		TRANSFORMATION MATRICES.
		UPDATE ORDER.

		ECI_TO_ECEF = np.zeros((3, 3))
		LLA = np.zeros(3)
		ECEFPOS = np.zeros(3)
		ECEFVEL = np.zeros(3)
		ECEF_TO_ENU = np.zeros((3, 3))
		ENUPOS = np.zeros(3)
		ENUVEL = np.zeros(3)
		ENU_TO_FLU = np.zeros((3, 3))
		ECEF_TO_FLU = np.zeros((3, 3))
		ECI_TO_FLU = np.zeros((3, 3))

		"""

		# ECI TO ECEF MATRIX.
		ECI_TO_ECEF = ct.ECI_TO_ECEF_TM(TOF)
		OMEGA = npa([0.0, 0.0, WEII3])
		ECIVEL_DUE_TO_ROTATION = np.cross(OMEGA, ECIPOS)

		# LLA.
		LLA = ct.ECI_TO_LLA(ECIPOS, TOF)

		# ECEF STATE AND ECEF TO ENU MATRIX.
		ECEFPOS = ECI_TO_ECEF @ ECIPOS
		ECEFVEL = ECI_TO_ECEF @ (ECIVEL - ECIVEL_DUE_TO_ROTATION)
		ECEF_TO_ENU = ct.ORIENTATION_TO_LOCAL_TM(
			0.0,
			(np.pi / 2.0) + LLA[0],
			LLA[1]
		)

		# ENU STATE AND ENU TO BODY MATRIX..
		ENUPOS = ECEF_TO_ENU @ (ECEFPOS - MSL["ECEFPOS0"])
		ENUVEL = ECEF_TO_ENU @ ECEFVEL
		ENU_AZ, ENU_EL = returnAzAndElevation(ENUVEL)
		ENU_TO_FLU = ct.FLIGHTPATH_TO_LOCAL_TM(ENU_AZ, -1.0 * ENU_EL)
		ENUEULER = npa([0.0, ENU_EL, ENU_AZ])

		# ECEF TO BODY MATRIX.
		ECEF_TO_FLU = ENU_TO_FLU @ ECEF_TO_ENU
		ECEFEULER = ct.EULER_FROM_DCM(ECEF_TO_FLU)

		# ECI TO BODY MATRIX.
		ECI_TO_FLU = ECEF_TO_FLU @ ECI_TO_ECEF

		# DERIVATIVES.
		BODYRATEDOT = np.zeros(3)
		BODYRATEDOT[0] = 0.0
		BODYRATEDOT[1] = (Q * REFERENCE_AREA * REFERENCE_DIAMETER * CM) / TMOI
		BODYRATEDOT[2] = (Q * REFERENCE_AREA * REFERENCE_DIAMETER * CN) / TMOI
		SPECIFIC_FORCE[0] = THRUST / MASS
		SPECIFIC_FORCE[1] = (Q * REFERENCE_AREA * CY) / MASS
		SPECIFIC_FORCE[2] = (Q * REFERENCE_AREA * CZ) / MASS
		# LOCAL_G1 = ct.ECI_GRAV(ECIPOS, TOF)
		# BODY_G1 = ECI_TO_FLU @ LOCAL_G1
		LOCAL_G = npa([0.0, 0.0, -1.0 * G])
		BODY_G = ENU_TO_FLU @ LOCAL_G
		SPECIFIC_FORCE += (BODY_G + BODY_DRAG)
		ECIACC = SPECIFIC_FORCE @ ECI_TO_FLU

		if INTEGRATION_PASS == 0:

			# ADD NEW TRANSFORMATION MATRICES.
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
			STATE_P0 = ECIPOS
			STATE_V0 = ECIVEL
			STATE_W0 = ECIEULER
			STATE_WDOT0 = BODY_RATE

			V1 = ECIVEL
			A1 = ECIACC
			WDOT1 = BODY_RATE
			WDOTDOT1 = BODYRATEDOT

			ECIPOS = STATE_P0 + V1 * (TIME_STEP / 2.0)
			ECIVEL = STATE_V0 + A1 * (TIME_STEP / 2.0)
			ECIEULER = STATE_W0 + WDOT1 * (TIME_STEP / 2.0)
			BODY_RATE = STATE_WDOT0 + WDOTDOT1 * (TIME_STEP / 2.0)

			TOF += (TIME_STEP / 2.0)

			INTEGRATION_PASS += 1

		elif INTEGRATION_PASS == 1:

			V2 = ECIVEL
			A2 = ECIACC
			WDOT2 = BODY_RATE
			WDOTDOT2 = BODYRATEDOT

			ECIPOS = STATE_P0 + V2 * (TIME_STEP / 2.0)
			ECIVEL = STATE_V0 + A2 * (TIME_STEP / 2.0)
			ECIEULER = STATE_W0 + WDOT2 * (TIME_STEP / 2.0)
			BODY_RATE = STATE_WDOT0 + WDOTDOT2 * (TIME_STEP / 2.0)
			
			INTEGRATION_PASS += 1

		elif INTEGRATION_PASS == 2:

			V3 = ECIVEL
			A3 = ECIACC
			WDOT3 = BODY_RATE
			WDOTDOT3 = BODYRATEDOT

			ECIPOS = STATE_P0 + V3 * (TIME_STEP)
			ECIVEL = STATE_V0 + A3 * (TIME_STEP)
			ECIEULER = STATE_W0 + WDOT3 * (TIME_STEP)
			BODY_RATE = STATE_WDOT0 + WDOTDOT3 * (TIME_STEP)

			TOF += (TIME_STEP / 2.0)

			INTEGRATION_PASS += 1

		elif INTEGRATION_PASS == 3:

			V4 = ECIVEL
			A4 = ECIACC
			WDOT4 = BODY_RATE
			WDOTDOT4 = BODYRATEDOT

			ECIPOS = STATE_P0 + (TIME_STEP / 6.0) * (V1 + 2 * V2 + 2 * V3 + V4)
			ECIVEL = STATE_V0 + (TIME_STEP / 6.0) * (A1 + 2 * A2 + 2 * A3 + A4)
			ECIEULER = STATE_W0 + (TIME_STEP / 6.0) * (WDOT1 + 2 * WDOT2 + 2 * WDOT3 + WDOT4)
			BODY_RATE = STATE_WDOT0 + (TIME_STEP / 6.0) * (WDOTDOT1 + 2 * WDOTDOT2 + 2 * WDOTDOT3 + WDOTDOT4)

			INTEGRATION_PASS = 0



if __name__ == "__main__":

	wallClockStart = time.time()

	# FIVEDOF
	LLA0 = npa([38.8719, 77.0563, 0.0])
	POS0 = np.zeros(3)
	AZ0 = 0
	EL0 = 45
	SPD0 = 10
	MSL = Construct5DOFMissile(LLA0, POS0, AZ0, EL0, SPD0, "MOCK_HELLFIRE5DOF")
	MSL = Fly5DOF(MSL, 100, 1.0, 0.0)

	wallClockEnd = time.time()
	print(f"RUN TIME : {wallClockEnd - wallClockStart} SECONDS")
