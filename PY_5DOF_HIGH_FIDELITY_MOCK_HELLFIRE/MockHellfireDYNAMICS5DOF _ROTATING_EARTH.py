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
	INITIAL_AZ = np.radians(INITIAL_AZIMUTH) # RADIANS.
	INITIAL_EL = np.radians(INITIAL_ELEVATION) # RADIANS.
	TOF = 0.0 # Seconds.
	FPA_TO_LOCAL = ct.ORIENTATION_TO_LOCAL_TM(0.0, -INITIAL_EL, INITIAL_AZ)
	ENUVEL = INITIAL_AIRSPEED * (FPA_TO_LOCAL[0])
	SPECIFIC_FORCE = np.zeros(3) # Meters per second squared..
	E_0 = npa([0.0, INITIAL_EL, INITIAL_AZ]) # Radians.
	EDOT_0 = np.zeros(3) # Radians per second.
	SPEED = la.norm(ENUVEL) # Meters per second.
	VEL_B = FPA_TO_LOCAL @ ENUVEL # Body velocity.
	ALPHA, SIDESLIP = returnAlphaAndBeta(VEL_B)

	# ORIGIN ###############################################################################
	LLA0 = np.zeros(3)
	ENU0 = np.zeros(3)
	ECEF0 = np.zeros(3)
	ECI0 = np.zeros(3)

	# LOCATION ###############################################################################
	LLA = np.zeros(3)
	ENUPOS = INITIAL_POSITION
	ENUVEL = ENUVEL
	ENU_TO_FLU = np.zeros((3, 3))
	ECEFPOS = np.zeros(3)
	ECEFVEL = np.zeros(3)
	ECEF_TO_ENU = np.zeros((3, 3))
	ECEF_TO_FLU = np.zeros((3, 3))
	ECIPOS = np.zeros(3)
	ECIVEL = np.zeros(3)
	ECI_TO_ECEF = np.zeros((3, 3))
	ECI_TO_FLU = np.zeros((3, 3))

	# LLA.
	LLA = npa([np.radians(INITIAL_LLA[0]), np.radians(INITIAL_LLA[1]), INITIAL_LLA[2]]) # Convert to lat, lon to radians.
	LLA0 = LLA

	# ENU.
	ENUPOS = ENUPOS
	ENUPOS0 = ENUPOS
	ENUVEL = ENUVEL
	ENU_AZ, ENU_EL = returnAzAndElevation(ENUVEL) # RADIANS
	ENU_TO_FLU = ct.FLIGHTPATH_TO_LOCAL_TM(ENU_AZ, -ENU_EL) # ND
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

	# ECI.
	ECIPOS0 = np.zeros(3)
	ECIPOS = np.zeros(3)
	ECIVEL = np.zeros(3)
	ECI_TO_ECEF = np.zeros((3, 3))
	ECI_TO_FLU = np.zeros((3, 3))

	ECIPOS = ct.LLA_TO_ECI(LLA, TOF)
	ECI_TO_ECEF = ct.ECI_TO_ECEF_TM(TOF)
	TEMP = ECI_TO_ECEF.transpose() @ ECEFVEL
	OMEGA = npa([0.0, 0.0, WEII3])
	ECIVEL_DUE_TO_ROTATION = np.cross(OMEGA, ECIPOS)
	ECIVEL = TEMP + ECIVEL_DUE_TO_ROTATION
	ECI_TO_FLU = ECEF_TO_FLU @ ECI_TO_ECEF

	# DATA. ###############################################################################
	MISSILE = {

		"IDENTITY": ID,
		"LOGFILE": open(f"PY_5DOF_HIGH_FIDELITY_MOCK_HELLFIRE/output/{ID}.txt", "w"),
		"LETHALITY": "FLYING",
		"ATMOS": ATMOS,
		"MASS_AND_MOTOR": MASS_AND_MOTOR,
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
			"POS_0X": ENUPOS[0],
			"POS_0Y": ENUPOS[1],
			"POS_0Z": ENUPOS[2],
			"VEL_0X": ENUVEL[0],
			"VEL_0Y": ENUVEL[1], 
			"VEL_0Z": ENUVEL[2], 
			"SPEED": SPEED,
			"UDOT_0": SPECIFIC_FORCE[0],
			"VDOT_0": SPECIFIC_FORCE[1],
			"WDOT_0": SPECIFIC_FORCE[2],
			"ALPHA": ALPHA,
			"SIDESLIP": SIDESLIP,
			"PHI_0": E_0[0],
			"THT_0": E_0[1],
			"PSI_0": E_0[2],
			"PHIDOT_0": EDOT_0[0],
			"THTDOT_0": EDOT_0[1],
			"PSIDOT_0": EDOT_0[2],

			"LAT": LLA[0],
			"LON": LLA[1],
			"ALT": LLA[2],
			"EPOS": ENUPOS[0],
			"NPOS": ENUPOS[1],
			"UPOS": ENUPOS[2],
			"EVEL": ENUVEL[0],
			"NVEL": ENUVEL[1],
			"UVEL": ENUVEL[2],
			"ECEFPOSX": ECEFPOS[0],
			"ECEFPOSY": ECEFPOS[1],
			"ECEFPOSZ": ECEFPOS[2],
			"ECEFVELX": ECEFVEL[0],
			"ECEFVELY": ECEFVEL[1],
			"ECEFVELZ": ECEFVEL[2],
			"ECIPOSX": ECIPOS[0],
			"ECIPOSY": ECIPOS[1],
			"ECIPOSZ": ECIPOS[2],
			"ECIVELX": ECIVEL[0],
			"ECIVELY": ECIVEL[1],
			"ECIVELZ": ECIVEL[2],

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
	POS_0 = np.zeros(3)
	POS_0[0] = MSL["STATE"]["POS_0X"]
	POS_0[1] = MSL["STATE"]["POS_0Y"]
	POS_0[2] = MSL["STATE"]["POS_0Z"]
	VEL_0 = np.zeros(3)
	VEL_0[0] = MSL["STATE"]["VEL_0X"]
	VEL_0[1] = MSL["STATE"]["VEL_0Y"]
	VEL_0[2] = MSL["STATE"]["VEL_0Z"]
	SPEED = MSL["STATE"]["SPEED"]
	SPECIFIC_FORCE = np.zeros(3)
	SPECIFIC_FORCE[0] = MSL["STATE"]["UDOT_0"]
	SPECIFIC_FORCE[1] = MSL["STATE"]["VDOT_0"]
	SPECIFIC_FORCE[2] = MSL["STATE"]["WDOT_0"]
	ALPHA = MSL["STATE"]["ALPHA"]
	SIDESLIP = MSL["STATE"]["SIDESLIP"]
	E_0 = np.zeros(3)
	E_0[0] = MSL["STATE"]["PHI_0"]
	E_0[1] = MSL["STATE"]["THT_0"]
	E_0[2] = MSL["STATE"]["PSI_0"]
	EDOT_0 = np.zeros(3)
	EDOT_0[0] = MSL["STATE"]["PHIDOT_0"]
	EDOT_0[1] = MSL["STATE"]["THTDOT_0"]
	EDOT_0[2] = MSL["STATE"]["PSIDOT_0"]

	LLA = np.zeros(3)
	LLA[0] = MSL["STATE"]["LAT"]
	LLA[1] = MSL["STATE"]["LON"]
	LLA[2] = MSL["STATE"]["ALT"]
	ENUPOS = np.zeros(3)
	ENUPOS[0] = MSL["STATE"]["EPOS"]
	ENUPOS[1] = MSL["STATE"]["NPOS"]
	ENUPOS[2] = MSL["STATE"]["UPOS"]
	ENUVEL = np.zeros(3)
	ENUVEL[0] = MSL["STATE"]["EVEL"]
	ENUVEL[1] = MSL["STATE"]["NVEL"]
	ENUVEL[2] = MSL["STATE"]["UVEL"]
	ENU_TO_FLU = MSL["TRANSFORMATION_MATRICES"]["ENU_TO_FLU"]
	ECEFPOS = np.zeros(3)
	ECEFPOS[0] = MSL["STATE"]["ECEFPOSX"]
	ECEFPOS[1] = MSL["STATE"]["ECEFPOSY"]
	ECEFPOS[2] = MSL["STATE"]["ECEFPOSZ"]
	ECEFVEL = np.zeros(3)
	ECEFVEL[0] = MSL["STATE"]["ECEFVELX"]
	ECEFVEL[1] = MSL["STATE"]["ECEFVELY"]
	ECEFVEL[2] = MSL["STATE"]["ECEFVELZ"]
	ECEF_TO_ENU = MSL["TRANSFORMATION_MATRICES"]["ECEF_TO_ENU"]
	ECEF_TO_FLU = MSL["TRANSFORMATION_MATRICES"]["ECEF_TO_FLU"]
	ECIPOS = np.zeros(3)
	ECIPOS[0] = MSL["STATE"]["ECIPOSX"]
	ECIPOS[1] = MSL["STATE"]["ECIPOSY"]
	ECIPOS[2] = MSL["STATE"]["ECIPOSZ"]
	ECIVEL = np.zeros(3)
	ECIVEL[0] = MSL["STATE"]["ECIVELX"]
	ECIVEL[1] = MSL["STATE"]["ECIVELY"]
	ECIVEL[2] = MSL["STATE"]["ECIVELZ"]
	ECI_TO_ECEF = MSL["TRANSFORMATION_MATRICES"]["ECI_TO_ECEF"]
	ECI_TO_FLU = MSL["TRANSFORMATION_MATRICES"]["ECI_TO_FLU"]

	# INTEGRATION. ###############################################################################
	INTEGRATION_PASS = 0
	STATE_P0 = POS_0
	STATE_V0 = VEL_0
	STATE_E0 = E_0
	STATE_EDOT0 = EDOT_0

	V1 = np.zeros(2)
	A1 = np.zeros(2)
	EDOT1 = 0.0
	EDOTDOT1 = 0.0

	V2 = np.zeros(2)
	A2 = np.zeros(2)
	EDOT2 = 0.0
	EDOTDOT2 = 0.0

	V3 = np.zeros(2)
	A3 = np.zeros(2)
	EDOT3 = 0.0
	EDOTDOT3 = 0.0

	V4 = np.zeros(2)
	A4 = np.zeros(2)
	EDOT4 = 0.0
	EDOTDOT4 = 0.0

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
			"POS_0X": POS_0[0],
			"POS_0Y": POS_0[1],
			"POS_0Z": POS_0[2],
			"VEL_0X": VEL_0[0],
			"VEL_0Y": VEL_0[1], 
			"VEL_0Z": VEL_0[2], 
			"SPEED": SPEED,
			"UDOT_0": SPECIFIC_FORCE[0],
			"VDOT_0": SPECIFIC_FORCE[1],
			"WDOT_0": SPECIFIC_FORCE[2],
			"ALPHA": ALPHA,
			"SIDESLIP": SIDESLIP,
			"PHI_0": E_0[0],
			"THT_0": E_0[1],
			"PSI_0": E_0[2],
			"PHIDOT_0": EDOT_0[0],
			"THTDOT_0": EDOT_0[1],
			"PSIDOT_0": EDOT_0[2],

			"LAT": LLA[0],
			"LON": LLA[1],
			"ALT": LLA[2],
			"EPOS": ENUPOS[0],
			"NPOS": ENUPOS[1],
			"UPOS": ENUPOS[2],
			"EVEL": ENUVEL[0],
			"NVEL": ENUVEL[1],
			"UVEL": ENUVEL[2],
			"ECEFPOSX": ECEFPOS[0],
			"ECEFPOSY": ECEFPOS[1],
			"ECEFPOSZ": ECEFPOS[2],
			"ECEFVELX": ECEFVEL[0],
			"ECEFVELY": ECEFVEL[1],
			"ECEFVELZ": ECEFVEL[2],
			"ECIPOSX": ECIPOS[0],
			"ECIPOSY": ECIPOS[1],
			"ECIPOSZ": ECIPOS[2],
			"ECIVELX": ECIVEL[0],
			"ECIVELY": ECIVEL[1],
			"ECIVELZ": ECIVEL[2],

		}

		return STATE

	# LOOP. ###############################################################################
	GO = True
	while GO:

		# ATMOSPHERE.
		MSL["ATMOS"].update(POS_0[2], SPEED)
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
		FPA_TO_LOCAL = ct.ORIENTATION_TO_LOCAL_TM(E_0[0], -1.0 * E_0[1], E_0[2])
		SPEED = la.norm(VEL_0)
		VEL_B = FPA_TO_LOCAL @ VEL_0
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

		# DERIVATIVES.
		EDOTDOT_0 = np.zeros(3)
		EDOTDOT_0[0] = 0.0
		EDOTDOT_0[1] = (Q * REFERENCE_AREA * REFERENCE_DIAMETER * CM) / TMOI
		EDOTDOT_0[2] = (Q * REFERENCE_AREA * REFERENCE_DIAMETER * CN) / TMOI
		SPECIFIC_FORCE[0] = THRUST / MASS
		SPECIFIC_FORCE[1] = (Q * REFERENCE_AREA * CY) / MASS
		SPECIFIC_FORCE[2] = (Q * REFERENCE_AREA * CZ) / MASS
		LOCAL_G = npa([0.0, 0.0, -1.0 * G])
		BODY_G = FPA_TO_LOCAL @ LOCAL_G
		SPECIFIC_FORCE += (BODY_G + BODY_DRAG)
		ACC_0 = SPECIFIC_FORCE @ FPA_TO_LOCAL

		# LOCATION.

		# STATE.
		if INTEGRATION_PASS == 0:

			# LOG DATA.
			MSL["STATE"] = populateState()
			lf.writeData(MSL["STATE"], MSL["LOGFILE"] )

			# END CHECK.
			if TOF > MAX_TIME:
				MSL["LETHALITY"] = "MAX_TIME"
				return MSL
			if POS_0[2] < 0.0:
				print(f"GROUND - TOF : {TOF:.2f}, ENU : {POS_0}, MACH : {MACH:.2f}")
				MSL["LETHALITY"] = "GROUND"
				return MSL
			if np.isnan(np.sum(POS_0)):
				print(f"NAN - TOF : {TOF:.2f}, ENU : {POS_0}, MACH : {MACH:.2f}")
				MSL["LETHALITY"] = "NAN"
				return MSL

			# BEGIN INTEGRATION PASS.
			STATE_P0 = POS_0
			STATE_V0 = VEL_0
			STATE_E0 = E_0
			STATE_EDOT0 = EDOT_0

			V1 = VEL_0
			A1 = ACC_0
			EDOT1 = EDOT_0
			EDOTDOT1 = EDOTDOT_0

			POS_0 = STATE_P0 + V1 * (TIME_STEP / 2.0)
			VEL_0 = STATE_V0 + A1 * (TIME_STEP / 2.0)
			E_0 = STATE_E0 + EDOT1 * (TIME_STEP / 2.0)
			EDOT_0 = STATE_EDOT0 + EDOTDOT1 * (TIME_STEP / 2.0)

			TOF += (TIME_STEP / 2.0)

			INTEGRATION_PASS += 1

		elif INTEGRATION_PASS == 1:

			V2 = VEL_0
			A2 = ACC_0
			EDOT2 = EDOT_0
			EDOTDOT2 = EDOTDOT_0

			POS_0 = STATE_P0 + V2 * (TIME_STEP / 2.0)
			VEL_0 = STATE_V0 + A2 * (TIME_STEP / 2.0)
			E_0 = STATE_E0 + EDOT2 * (TIME_STEP / 2.0)
			EDOT_0 = STATE_EDOT0 + EDOTDOT2 * (TIME_STEP / 2.0)

			INTEGRATION_PASS += 1

		elif INTEGRATION_PASS == 2:

			V3 = VEL_0
			A3 = ACC_0
			EDOT3 = EDOT_0
			EDOTDOT3 = EDOTDOT_0

			POS_0 = STATE_P0 + V3 * (TIME_STEP)
			VEL_0 = STATE_V0 + A3 * (TIME_STEP)
			E_0 = STATE_E0 + EDOT3 * (TIME_STEP)
			EDOT_0 = STATE_EDOT0 + EDOTDOT3 * (TIME_STEP)

			TOF += (TIME_STEP / 2.0)

			INTEGRATION_PASS += 1

		elif INTEGRATION_PASS == 3:

			V4 = VEL_0
			A4 = ACC_0
			EDOT4 = EDOT_0
			EDOTDOT4 = EDOTDOT_0

			POS_0 = STATE_P0 + (TIME_STEP / 6.0) * (V1 + 2 * V2 + 2 * V3 + V4)
			VEL_0 = STATE_V0 + (TIME_STEP / 6.0) * (A1 + 2 * A2 + 2 * A3 + A4)
			E_0 = STATE_E0 + (TIME_STEP / 6.0) * (EDOT1 + 2 * EDOT2 + 2 * EDOT3 + EDOT4)
			EDOT_0 = STATE_EDOT0 + (TIME_STEP / 6.0) * (EDOTDOT1 + 2 * EDOTDOT2 + 2 * EDOTDOT3 + EDOTDOT4)

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
	MSL = Fly5DOF(MSL, 100, -3.0, 0.0)

	wallClockEnd = time.time()
	print(f"RUN TIME : {wallClockEnd - wallClockStart} SECONDS")
