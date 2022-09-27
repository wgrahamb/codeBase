import numpy as np
from numpy import array as npa
from numpy import linalg as la
import utility.loggingFxns as lf
import utility.coordinateTransformations as ct
from utility.ATM_IMPERIAL import ATM_IMPERIAL
from utility.interpolationGivenTwoVectors import linearInterpolation
np.set_printoptions(precision=2, suppress=True)

# INPUTS.
ALT = 10000 # FEET
SPD = 1200 # FEET PER SEC
DEFL_D = 0 # DEGREES
DEFL = np.radians(DEFL_D) # RADIANS

# MISSILE CONSTANTS.
REF_DIAM = 0.23 # FEET
REF_LNGTH = 4.59 # FEET
REF_AREA = np.pi * (REF_DIAM ** 2) / 4

# AERODYNAMIC METHOD ONE, AIRFRAME LINEARIZATION.
# AIRFRAME.
NOSE_LNGTH = 1.11 # FEET
WNG_HLF_SPN = 0 # FEET
WNG_TIP_CHRD = 0 # FEET
WNG_ROOT_CHRD = 0 # FEET
TAIL_HLF_SPN = 0.04 # FEET
TAIL_TIP_CHRD = 0.27 # FEET
TAIL_ROOT_CHRD = 0.27 # FEET
BASE_OF_NOSE_TO_WNG = 0 # FEET
XCD = REF_LNGTH - (TAIL_TIP_CHRD / 2.0) # FEET, FROM NOSE.

# BODY.
WNG_AREA = 0.5 * WNG_HLF_SPN * (WNG_TIP_CHRD + WNG_ROOT_CHRD)
TAIL_AREA = 0.5 * TAIL_HLF_SPN * (TAIL_TIP_CHRD + TAIL_ROOT_CHRD)
NOSE_AREA = NOSE_LNGTH * REF_DIAM
PLANFORM_AREA = (REF_LNGTH - NOSE_LNGTH) * REF_DIAM + \
	0.667 * NOSE_LNGTH * REF_DIAM
XCP_NOSE = 0.67 * NOSE_LNGTH
XCP_WNG = NOSE_LNGTH + BASE_OF_NOSE_TO_WNG + \
	0.7 * WNG_ROOT_CHRD - 0.2 * WNG_TIP_CHRD
AN = 0.67 * NOSE_LNGTH * REF_DIAM
AB = (REF_LNGTH - NOSE_LNGTH) * REF_DIAM
XCP_BODY = (0.67 * AN * NOSE_LNGTH + \
	AB * (NOSE_LNGTH + 0.5 * (REF_LNGTH - NOSE_LNGTH))) / (AN + AB)

# AERODYNAMIC METHOD TWO, TABLES.
# 70 MM ROCKET TABLES.
MACHS = [0.0, 0.6, 0.9, 1.15, 1.3, 1.6, 2.48, 2.97, 100.0]
CMQS = [1060.0, 1060.0, 1460.0, 1385.0, 1193.0, 1069.0, 850.0, 800.0, 800.0]
CNAS = [8.19, 8.19, 8.94, 9.34, 8.88, 8.14, 7.51, 7.22, 7.22]
XCPS = [36.822500000000005, 36.575, 38.114999999999995, 39.875, 39.93, \
	38.0875, 36.8775, 36.739999999999995, 36.739999999999995] # inches from nose

# AERODYNAMICS.
CMQ = linearInterpolation(0.0, MACHS, CMQS)
CNA = linearInterpolation(0.0, MACHS, CNAS)
XCP = linearInterpolation(0.0, MACHS, XCPS)

# ATMOSPHERE.
ATM = ATM_IMPERIAL()
ATM.update(ALT, SPD)
RHO = ATM.rho
G = ATM.g
Q = ATM.q
A = ATM.a
MACH = ATM.mach
BETA = np.sqrt(MACH ** 2 - 1) # ND

# MASS AND MOTOR. (NO MOTOR FOR NOW.)
T1S = [0.0, 1.112, 1000.0] # seconds
XCGS = [29.96, 33.55, 33.55] # inches, from base
TMOIS = [6248.0, 5008.0, 5008.0] # lbm * in^2
WEIGHTS = [22.95, 15.73, 15.73] # lbm
T2S = [0.0, 0.012, 0.037, 0.062, 0.187, 0.412, 0.437, 0.462, 0.487, 0.512, 0.537, \
	0.562, 0.862, 0.887, 0.912, 0.937, 0.962, \
	0.987, 1.037, 1.062, 1.087, 1.112, 1.113, 100.0] # seconds
THRUSTS = [0.0, 1304.3, 1400.0, 1439.1, 1245.7, 1109.0, 1267.2, 1276.9, 1451.8, \
	1457.7, 1267.2, 1234.0, 1522.2, 1485.0, 1611.1, \
	1654.1, 1780.1, 1792.8, 1463.5, 1070.8, 491.4, 146.6, 0.0, 0.0]

THRUST = linearInterpolation(0.0, T2S, THRUSTS)
XCG = REF_LNGTH - (linearInterpolation(0.0, T1S, XCGS) / 12.0) # FT
MASS = linearInterpolation(0.0, T1S, WEIGHTS) # LBM
TMOI = (linearInterpolation(0.0, T1S, TMOIS)) / (144.0 * 32.2) # LBF - FT - S^2

# DERIVATIVES.
SPECIFIC_FORCE = np.zeros(2)
ACC = np.zeros(2)
ADOT = 0.0
QDOT = 0.0
WDOT = 0.0

# STATE.
INT_PASS = 0
POS = npa([0.0, ALT])
VEL = npa([SPD, 0.0])
THT = 0.0
RATE = 0.0
ALPHA = 0.0
POS0 = np.zeros(2)
VEL0 = np.zeros(2)
THT0 = 0.0
RATE0 = 0.0
ALPHA0 = 0.0

# SIM CONTROL.
TOF = 0.0
DT = 1.0 / 1000.0
MAXT = 10

# DATA
def populateState():
	STATE = {
		"TOF": TOF,
		"RNG": POS[0],
		"ALT": POS[1],
		"THETA": THT,
		"RATE": RATE,
		"ALPHA": ALPHA,
		"ALPHADOT": ADOT,
		"RATEDOT": QDOT,
		"WDOT": WDOT
	}
	return STATE

STATE = populateState()
LOGFILE = open("PY_6DOF_70MM_ROCKET/data/log.txt", "w")
lf.writeHeader(STATE, LOGFILE)
lf.writeData(STATE, LOGFILE)

LASTT = 0.0
while TOF <= MAXT:

	# MASS AND MOTOR. (NO MOTOR FOR NOW.)
	THRUST = linearInterpolation(0.0, T2S, THRUSTS) # LBF
	XCG = REF_LNGTH - (linearInterpolation(TOF, T1S, XCGS) / 12.0) # FT
	MASS = linearInterpolation(TOF, T1S, WEIGHTS) # LBM
	TMOI = (linearInterpolation(TOF, T1S, TMOIS)) / (144.0 * 32.2) # LBF - FT - S^2

	# ATMOSPHERE.
	SPD = la.norm(VEL)
	ATM.update(POS[1], SPD)
	RHO = ATM.rho
	G = ATM.g
	Q = ATM.q
	A = ATM.a
	MACH = ATM.mach
	if MACH > 1:
		BETA = np.sqrt(MACH ** 2 - 1) # ND
	else:
		BETA = MACH

	# CN AND CM.
	CN = 2 * ALPHA + \
		(1.5 * PLANFORM_AREA * ALPHA * ALPHA) / REF_AREA + \
		(8 * WNG_AREA * ALPHA) / (BETA * REF_AREA) + \
		(8 * TAIL_AREA * (ALPHA + DEFL)) / (BETA * REF_AREA)
	CM = 2 * ALPHA * ((XCG - XCP_NOSE) / REF_DIAM) + \
		((1.5 * PLANFORM_AREA * ALPHA * ALPHA) / REF_AREA) * \
		((XCG - XCP_BODY) / REF_DIAM) + \
		((8 * WNG_AREA * ALPHA) / (BETA * REF_AREA)) * \
		((XCG - XCP_WNG) / REF_DIAM) + \
		((8 * TAIL_AREA * (ALPHA + DEFL)) / (BETA * REF_AREA)) * \
		((XCG - XCD) / REF_DIAM)

	# DERIVATIVES.
	ADOT = RATE - (SPECIFIC_FORCE[1] / SPD) # RADS PER S
	QDOT = (Q * REF_AREA * REF_DIAM * CM) / TMOI # RADS PER S^2
	WDOT = ((Q * REF_AREA * CN) * (G / MASS)) # FT PER S^2

	SPECIFIC_FORCE = npa([0.0, WDOT])
	LOCALG = npa([0.0, -1.0 * G])
	LOCAL_TO_BODY_TM = ct.BODY_TO_RANGE_AND_ALTITUDE(-1.0 * THT)
	BODYG = LOCAL_TO_BODY_TM @ LOCALG
	SPECIFIC_FORCE += BODYG
	ACC = (SPECIFIC_FORCE @ LOCAL_TO_BODY_TM)

	# STATE.
	if INT_PASS == 0:

		# DATA.
		STATE = populateState()
		lf.writeData(STATE, LOGFILE)

		# REPORT.
		if round(TOF, 3).is_integer() and LASTT <= TOF:
			print(f"{TOF:.0f} POS {POS}")
			LASTT = TOF

		INT_PASS += 1
		TOF += (DT / 2.0)

		POS0 = POS
		VEL0 = VEL
		THT0 = THT
		RATE0 = RATE
		ALPHA0 = ALPHA

		POS += VEL * (DT / 2.0)
		VEL += ACC * (DT / 2.0)
		THT += RATE * (DT / 2.0)
		RATE += QDOT * (DT / 2.0)
		ALPHA += ADOT * (DT / 2.0)

	else:

		INT_PASS = 0
		TOF += (DT / 2.0)

		POS = POS0 + VEL * (DT / 2.0)
		VEL = VEL0 + ACC * (DT / 2.0)
		THT = THT0 + RATE * (DT / 2.0)
		RATE = RATE0 + QDOT * (DT / 2.0)
		ALPHA = ALPHA0 + ADOT * (DT / 2.0)

		POS0 = np.zeros(2)
		VEL0 = np.zeros(2)
		THT0 = 0.0
		RATE0 = 0.0
		ALPHA0 = 0.0

















