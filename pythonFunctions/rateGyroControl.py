import numpy as np
from numpy import array as npa
from numpy import linalg as la
import pandas as pd
import matplotlib.pyplot as plt
import copy
import matplotlib
matplotlib.use('WebAgg')

# INPUTS
ALT = 1000 # FEET
SPD = 1200 # FEET PER SEC
COMMAND = 10 # Gs

# MISSILE CONSTANTS
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
XCG = 10 # FEET
XCD = 19.5 # FEET

# AIRFRAME
WNG_AREA = 0.5 * WNG_HLF_SPN * (WNG_TIP_CHRD + WNG_ROOT_CHRD)
TAIL_AREA = 0.5 * TAIL_HLF_SPN * (TAIL_TIP_CHRD + TAIL_ROOT_CHRD)
REF_AREA = np.pi * (REF_DIAM ** 2) / 4
NOSE_AREA = NOSE_LNGTH * REF_DIAM
PLANFORM_AREA = (REF_LNGTH - NOSE_LNGTH) * REF_DIAM + 0.667 * \
	NOSE_LNGTH * REF_DIAM
XCP_NOSE = 0.67 * NOSE_LNGTH
XCP_WNG = NOSE_LNGTH + BASE_OF_NOSE_TO_WNG + 0.7 * \
	WNG_ROOT_CHRD - 0.2 * WNG_TIP_CHRD
AN = 0.67 * NOSE_LNGTH * REF_DIAM
AB = (REF_LNGTH - NOSE_LNGTH) * REF_DIAM
XCP_BODY = (0.67 * AN * NOSE_LNGTH + AB * (NOSE_LNGTH + 0.5 * \
	(REF_LNGTH - NOSE_LNGTH))) / (AN + AB)

# ATMOSPHERE AND MASS PROPERTIES
if ALT <= 30000:
	RHO = 0.002378 * np.exp(-ALT / 30000)
else:
	RHO = 0.0034 * np.exp(-ALT / 22000)
G = 32.2 # FEET PER S^2 >>> ASSUMED CONSTANT
A = 1000 # FEET PER SECOND >>> ASSUMED CONSTANT
Q = 0.5 * RHO * SPD * SPD
MACH = SPD / A
BETA = np.sqrt(MACH ** 2 - 1)
MASS = 1000 # LBM
TMOI = (MASS * (3 * ((0.5 * REF_DIAM) \
	** 2) + REF_LNGTH ** 2)) / (12 * G)

# AERODYNAMICS
TEMP1 = (XCG - XCP_WNG) / REF_DIAM
TEMP2 = (XCG - XCD) / REF_DIAM
TEMP3 = (XCG - XCP_BODY) / REF_DIAM
TEMP4 = (XCG - XCP_NOSE) / REF_DIAM

Y1 = 2 + 8 * WNG_AREA / (BETA * REF_AREA) + 8 * TAIL_AREA / (BETA * REF_AREA)
Y2 = 1.5 * PLANFORM_AREA / REF_AREA
Y3 = 8 * TAIL_AREA / (BETA * REF_AREA)
Y4 = 2 * TEMP4 + 8 * WNG_AREA * TEMP1 / (BETA * REF_AREA) + \
	8 * TAIL_AREA * TEMP2 / (BETA * REF_AREA)
Y5 = 1.5 * PLANFORM_AREA * TEMP3 / REF_AREA
Y6 = 8 * TAIL_AREA * TEMP2 / (BETA * REF_AREA)
P2 = Y2 - (Y3 * Y5) / Y6
P3 = Y1 - (Y3 * Y4) / Y6

CNTRIM = MASS * COMMAND / (Q * REF_AREA)
ATRIM = (-1 * P3 + np.sqrt(P3 * P3 + 4 * P2 * CNTRIM)) / (2 * P2)
DTRIM = (-1 * Y4 * ATRIM - Y5 * ATRIM * ATRIM) / Y6

CNA = 2 + 1.5 * PLANFORM_AREA * ATRIM / REF_AREA + 8 * WNG_AREA / \
	(BETA * REF_AREA) + 8 * TAIL_AREA / (BETA * REF_AREA)
CND = 8 * TAIL_AREA / (BETA * REF_AREA)
CMAP = 2 * TEMP4 + 1.5 * PLANFORM_AREA * ATRIM * TEMP3 / \
	REF_AREA + 8 * WNG_AREA * TEMP1 / (BETA * REF_AREA)
CMA = CMAP + 8 * TAIL_AREA * TEMP2 / (BETA * REF_AREA)
CMD = 8 * TAIL_AREA * TEMP2 / (BETA * REF_AREA)

ZA = -1 * G * Q * REF_AREA * CNA / (MASS * SPD)
ZD = -1 * G * Q * REF_AREA * CND / (MASS * SPD)
MA = Q * REF_AREA * REF_DIAM * CMA / TMOI
MD = Q * REF_AREA * REF_DIAM * CMD / TMOI

OMEGAZ = np.sqrt((MA * ZD - MD * ZA) / ZD)
OMEGAAF = np.sqrt(-1 * MA)
ZETAAF = ZA * OMEGAAF / (2 * MA)
KR = 0.15
K1 = -1 * SPD * ((MA * ZD - ZA * MD) / (1845 * MA))
TA = MD / (MA * ZD - MD * ZA)
K3 = 1845 * K1 / SPD
KDC = (1 - KR * K3) / (K1 * KR)

# STATE
E = 0.0
EDOT = 0.0
INT_PASS = 0
E0 = None
EDOT0 = None

# SIMULATION CONTROL
TOF = 0.0
DT = 0.01
MAXT = 3

# DATA
DATA = {"TOF": [], "WDOT": [], "CMD": [], "DEFL": []}

# LOOP
while TOF <= MAXT:

	THD = K3 * (E + TA * EDOT)
	DEFL = KR * (KDC * COMMAND + THD)
	EDOTDOT = (OMEGAAF ** 2) * (DEFL - E - 2 * ZETAAF * EDOT / OMEGAAF)
	WDOT = K1 * (E - (EDOTDOT / (OMEGAZ ** 2)))

	if INT_PASS == 0:

		# DATA
		DATA["TOF"].append(TOF)
		DATA["WDOT"].append(WDOT)
		DATA["CMD"].append(COMMAND)
		DATA["DEFL"].append(DEFL)

		# STATE
		INT_PASS += 1
		E0 = copy.deepcopy(E)
		EDOT0 = copy.deepcopy(EDOT)
		TOF += (DT / 2.0)
		E += EDOT * (DT / 2.0)
		EDOT += EDOTDOT * (DT / 2.0)

	else:

		# STATE.
		INT_PASS = 0
		TOF += (DT / 2.0)
		E = E0 + EDOT * DT
		EDOT = EDOT0 + EDOTDOT * DT
		E0 = None
		EDOT0 = None

DF = pd.DataFrame(DATA)
plt.plot(DF.iloc[:]["TOF"], DF.iloc[:]["WDOT"], \
	label="ACHIEVED", color="b")
plt.plot(DF.iloc[:]["TOF"], DF.iloc[:]["CMD"], \
	label="COMMANDED", color="r")
plt.plot(DF.iloc[:]["TOF"], DF.iloc[:]["DEFL"], \
	label="DEFL", color="g")
plt.legend()
plt.show()