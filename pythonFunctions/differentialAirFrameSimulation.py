import numpy as np
from numpy import array as npa
from numpy import linalg as la
import loggingFxns as lf
import coordinateTransformations as ct
from atmNASA_imperial import atmNASA_imperial
import copy

# CONSTANTS.
RAD2DEG = 57.3

# INPUTS.
ALT = 10000 # FEET
SPD = 1200 # FEET PER SEC
DEFL_D = 1 # DEGREES
DEFL = np.radians(DEFL_D) # RADIANS

# AIRFRAME.
REF_DIAM = 1 # FEET
NOSE_LNGTH = 3 # FEET
REF_LNGTH = 20 # FEET
WNG_HLF_SPN = 2 # FEET
WNG_TIP_CHRD = 0 # FEET
WNG_ROOT_CHRD = 6 # FEET
TAIL_HLF_SPN = 2 # FEET
TAIL_TIP_CHRD = 0 # FEET
TAIL_ROOT_CHRD = 2 # FEET
BASE_OF_NOSE_TO_WNG = 4 # FEET
XCG = 10 # FEET, FROM NOSE.
XCD = 19.5 # FEET, FROM NOSE.

# ATMOSPHERE.
ATM = atmNASA_imperial()
ATM.update(ALT, SPD)
RHO = ATM.rho
G = ATM.g
Q = ATM.q
A = ATM.a
MACH = ATM.mach
BETA = np.sqrt(MACH ** 2 - 1) # ND

# MASS AND MOTOR. (NO MOTOR FOR NOW.)
MASS = 1000 # LBM
TMOI = (MASS * (3 * ((0.5 * REF_DIAM) ** 2) \
	+ REF_LNGTH ** 2)) / (12 * G) # LBF - FT - S^2

# BODY.
WNG_AREA = 0.5 * WNG_HLF_SPN * (WNG_TIP_CHRD + WNG_ROOT_CHRD)
TAIL_AREA = 0.5 * TAIL_HLF_SPN * (TAIL_TIP_CHRD + TAIL_ROOT_CHRD)
REF_AREA = np.pi * (REF_DIAM ** 2) / 4
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
TEMP1 = (XCG - XCP_WNG) / REF_DIAM
TEMP2 = (XCG - XCD) / REF_DIAM
TEMP3 = (XCG - XCP_BODY) / REF_DIAM
TEMP4 = (XCG - XCP_NOSE) / REF_DIAM
Y1 = 2 * TEMP4 + 8 * WNG_AREA * TEMP1 / (BETA * REF_AREA) + 8 * TAIL_AREA * TEMP2 / (BETA * REF_AREA)
Y2 = 1.5 * PLANFORM_AREA * TEMP3 / REF_AREA
Y3 = 8 * TAIL_AREA * TEMP2 * DEFL / (BETA * REF_AREA)
ATRIM = (-Y1 - np.sqrt((Y1 ** 2) - 4 * Y2 * Y3)) / (2 * Y2)
CNA = 2 + 1.5 * PLANFORM_AREA * ATRIM / REF_AREA + 8 * WNG_AREA / (BETA * REF_AREA) + 8 * TAIL_AREA / (BETA * REF_AREA)
CND = 8 * TAIL_AREA / (BETA * REF_AREA)
ZA = -1 * G * Q * REF_AREA * CNA / (MASS * SPD)
ZD = -1 * G * Q * REF_AREA * CND / (MASS * SPD)
CMAP = 2 * TEMP4 + 1.5 * PLANFORM_AREA * ATRIM * TEMP3 / REF_AREA + 8 * WNG_AREA * TEMP1 / (BETA * REF_AREA)
CMA = CMAP + 8 * TAIL_AREA * TEMP2 / (BETA * REF_AREA)
CMD = 8 * TAIL_AREA * TEMP2 / (BETA * REF_AREA)
MA = Q * REF_AREA * REF_DIAM * CMA / TMOI
MD = Q * REF_AREA * REF_DIAM * CMD / TMOI
OMEGAZ = np.sqrt((MA * ZD - MD * ZA) / ZD)
OMEGAAF = np.sqrt(-1 * MA)
ZETAAF = ZA * OMEGAAF / (2 * MA)
K1 = -1 * SPD * ((MA * ZD - ZA * MD) / (1845 * MA))
K2 = K1
K3 = 1845 * K1 / SPD
TA = MD / (MA * ZD - ZA * MD)

# DERIVATIVES.
RATE = 0.0
SPECIFIC_FORCE = np.zeros(2)
ACC = np.zeros(2)
ADOT = 0.0
WDOT = 0.0

# MISSILE STATE.
POS = npa([0.0, ALT])
VEL = npa([SPD, 0.0])
THT = 0.0
ALPHA = 0.0
E = 0.0
EDOT = 0.0

INT_PASS = 0
POS0 = None
VEL0 = None
THT0 = None
ALPHA0 = None
E0 = None
EDOT0 = None

# SIM CONTROL.
TOF = 0.0
DT = 0.001
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
		"WDOT": WDOT
	}
	return STATE

# STATE = populateState()
# LOGFILE = open("PY_6DOF_70MM_ROCKET/data/log1.txt", "w")
# lf.writeHeader(STATE, LOGFILE)
# lf.writeData(STATE, LOGFILE)

LASTT = None
while TOF <= MAXT:

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

	# DERIVATIVES.
	EDOTDOT = (OMEGAAF ** 2) * (DEFL_D - E - 2 * ZETAAF * EDOT / OMEGAAF)
	ADOT = RATE - ((SPECIFIC_FORCE[1] / SPD) * RAD2DEG) # DEG PER SEC
	RATE = K3 * (E + TA * EDOT) # DEG PER SEC
	WDOT = K1 * (E - (EDOTDOT / (OMEGAZ ** 2))) * G # FT PER S^2

	SPECIFIC_FORCE = npa([0.0, WDOT])
	LOCALG = npa([0.0, -1.0 * G])
	LOCAL_TO_BODY_TM = ct.BODY_TO_RANGE_AND_ALTITUDE(-1.0 * THT)
	BODYG = LOCAL_TO_BODY_TM @ LOCALG
	SPECIFIC_FORCE += BODYG
	ACC = (SPECIFIC_FORCE @ LOCAL_TO_BODY_TM)

	if INT_PASS == 0:

		INT_PASS += 1

		POS0 = copy.deepcopy(POS)
		VEL0 = copy.deepcopy(VEL)
		THT0 = copy.deepcopy(THT)
		ALPHA0 = copy.deepcopy(ALPHA)
		E0 = copy.deepcopy(E)
		EDOT0 = copy.deepcopy(EDOT)

		TOF += (DT / 2.0)
		POS += VEL * (DT / 2.0)
		VEL += ACC * (DT / 2.0)
		THT += RATE * (DT / 2.0)
		ALPHA += ADOT * (DT / 2.0)
		E += EDOT * (DT / 2.0)
		EDOT += EDOTDOT * (DT / 2.0)

		# # DATA.
		# STATE = populateState()
		# lf.writeData(STATE, LOGFILE)

		# REPORT.
		if round(TOF, 3).is_integer() and LASTT != TOF:
			print(f"{TOF:.0f} POS {POS}")

	else:

		INT_PASS = 0

		TOF += (DT / 2.0)
		POS = POS0 + VEL * DT
		VEL = VEL0 + ACC * DT
		THT = THT0 + RATE * DT
		ALPHA = ALPHA0 + ADOT * DT
		E = E0 + EDOT * DT
		EDOT = EDOT0 + EDOTDOT * DT

		POS0 = None
		VEL0 = None
		THT0 = None
		ALPHA0 = None
		E0 = None
		EDOT0 = None