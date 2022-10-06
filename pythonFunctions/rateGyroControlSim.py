
# pip installs
import numpy as np
from numpy import array as npa
from numpy import linalg as la
import pandas as pd
import matplotlib.pyplot as plt
import copy
import matplotlib
matplotlib.use('WebAgg')
np.set_printoptions(suppress=True, precision=2)

# utility
from atmNASA_imperial import atmNASA_imperial
import coordinateTransformations as ct

RAD2DEG = 57.3

# INPUTS
ALT = 1000 # FEET
INPUT_VEL = npa([100.0, 1500.0])
TGT_POS = npa([18000.0, 18000.0])
TGT_VEL = npa([-350.0, -500.0])

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

# ATMOSPHERE
ATM = atmNASA_imperial()
ATM.update(ALT, la.norm(INPUT_VEL))
RHO = ATM.rho
G = ATM.g
A = ATM.a
P = ATM.p
Q = ATM.q
MACH = ATM.mach

# MASS PROPERTIES
MASS = 1000 # LBM
TMOI = (MASS * (3 * ((0.5 * REF_DIAM) \
	** 2) + REF_LNGTH ** 2)) / (12 * G)

# STATE
POS = npa([0.0, ALT])
VEL = INPUT_VEL
THT = np.arctan2(VEL[1], VEL[0]) * RAD2DEG
E = 0.0
EDOT = 0.0

INT_PASS = 0
POS0 = None
VEL0 = None
THT0 = None
E0 = None
EDOT0 = None

# SIMULATION CONTROL
TOF = 0.0
DT = 0.001
MAXT = 100

# DATA
DATA = {
	"TOF": [],
	"X": [],
	"Y": [],
	"TGTX": [],
	"TGTY": [],
	"THT": [],
	"THD": [],
	"WDOT": [],
	"CMD": [],
	"DEFL": []
}

# LOOP
LASTT = None
while TOF <= MAXT:

	# UPDATE TARGET
	TGT_POS += TGT_VEL * (DT / 2.0)

	# ATTITUDE
	THT_2_BODY_TM = ct.BODY_TO_RANGE_AND_ALTITUDE(THT * (-1.0 / RAD2DEG))
	SPD = la.norm(VEL)

	# GUIDANCE
	RELPOS = TGT_POS - POS
	RELVEL = TGT_VEL - VEL
	BODY2TGT = THT_2_BODY_TM @ RELPOS
	CLOSING_VEL = THT_2_BODY_TM @ RELVEL
	CLOSING_SPEED = la.norm(CLOSING_VEL)
	T1 = np.cross(BODY2TGT, CLOSING_VEL)
	T2 = np.dot(BODY2TGT, BODY2TGT)
	OMEGA = T1 / T2
	COMMAND = (3 * OMEGA * CLOSING_SPEED) / G # Gs
	LIMIT = 20
	SIGN_COMM = np.sign(COMMAND)
	if np.abs(COMMAND) > LIMIT:
		COMMAND = SIGN_COMM * LIMIT

	# ATMOSPHERE
	ATM.update(POS[1], la.norm(INPUT_VEL))
	RHO = ATM.rho
	G = ATM.g
	A = ATM.a
	P = ATM.p
	Q = ATM.q
	MACH = ATM.mach
	BETA = np.sqrt(MACH ** 2 - 1)

	# AERODYNAMICS
	TEMP1 = (XCG - XCP_WNG) / REF_DIAM
	TEMP2 = (XCG - XCD) / REF_DIAM
	TEMP3 = (XCG - XCP_BODY) / REF_DIAM
	TEMP4 = (XCG - XCP_NOSE) / REF_DIAM
	CNTRIM = MASS * COMMAND / (Q * REF_AREA)
	Y1 = 2 + 8 * WNG_AREA / (BETA * REF_AREA) + 8 * TAIL_AREA / \
		(BETA * REF_AREA)
	Y2 = 1.5 * PLANFORM_AREA / REF_AREA
	Y3 = 8 * TAIL_AREA / (BETA * REF_AREA)
	Y4 = 2 * TEMP4 + 8 * WNG_AREA * TEMP1 / (BETA * REF_AREA) + \
		8 * TAIL_AREA * TEMP2 / (BETA * REF_AREA)
	Y5 = 1.5 * PLANFORM_AREA * TEMP3 / REF_AREA
	Y6 = 8 * TAIL_AREA * TEMP2 / (BETA * REF_AREA)
	P2 = Y2 - (Y3 * Y5) / Y6
	P3 = Y1 - (Y3 * Y4) / Y6
	ATRIM = (-1 * P3 + np.sqrt(P3 * P3 + 4 * P2 * CNTRIM)) / (2 * P2)
	DTRIM = (-1 * Y4 * ATRIM - Y5 * ATRIM * ATRIM) / Y6
	CNA = 2 + 1.5 * PLANFORM_AREA * ATRIM / REF_AREA + 8 * WNG_AREA / \
		(BETA * REF_AREA) + 8 * TAIL_AREA / (BETA * REF_AREA)
	CND = 8 * TAIL_AREA / (BETA * REF_AREA)
	ZA = -1 * G * Q * REF_AREA * CNA / (MASS * SPD)
	ZD = -1 * G * Q * REF_AREA * CND / (MASS * SPD)
	CMAP = 2 * TEMP4 + 1.5 * PLANFORM_AREA * ATRIM * TEMP3 / \
		REF_AREA + 8 * WNG_AREA * TEMP1 / (BETA * REF_AREA)
	CMA = CMAP + 8 * TAIL_AREA * TEMP2 / (BETA * REF_AREA)
	CMD = 8 * TAIL_AREA * TEMP2 / (BETA * REF_AREA)
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

	# DERIVATIVES
	THD = K3 * (E + TA * EDOT) # DEG PER SEC
	DEFL = KR * (KDC * COMMAND + THD) # DEG
	LIMIT = 25
	SIGN_DEFL = np.sign(DEFL)
	if np.abs(DEFL) > LIMIT:
		DEFL = SIGN_DEFL * LIMIT
	EDOTDOT = (OMEGAAF ** 2) * (DEFL - E - 2 * ZETAAF * EDOT / OMEGAAF)
	WDOT = K1 * (E - (EDOTDOT / (OMEGAZ ** 2))) # Gs
	SPECIFIC_FORCE = npa([0.0, WDOT * G])
	ACC = SPECIFIC_FORCE @ THT_2_BODY_TM

	if INT_PASS == 0:

		# DATA
		DATA["TOF"].append(TOF)
		DATA["X"].append(POS[0])
		DATA["Y"].append(POS[1])
		DATA["TGTX"].append(TGT_POS[0])
		DATA["TGTY"].append(TGT_POS[1])
		DATA["THT"].append(THT)
		DATA["THD"].append(THD)
		DATA["WDOT"].append(WDOT)
		DATA["CMD"].append(COMMAND)
		DATA["DEFL"].append(DEFL)

		# REPORT
		if round(TOF, 3).is_integer() and LASTT != TOF:
			print(f"{TOF:.0f} {POS}")
			LASTT = TOF

		# END CHECK
		if BODY2TGT[0] < 5.0:
			print(f"{TOF:.2f} {POS} MISS: {BODY2TGT}")
			break

		# STATE
		INT_PASS += 1

		POS0 = copy.deepcopy(POS)
		VEL0 = copy.deepcopy(VEL)
		THT0 = copy.deepcopy(THT)
		E0 = copy.deepcopy(E)
		EDOT0 = copy.deepcopy(EDOT)

		TOF += (DT / 2.0)
		POS += VEL * (DT / 2.0)
		VEL += ACC * (DT / 2.0)
		THT += THD * (DT / 2.0)
		E += EDOT * (DT / 2.0)
		EDOT += EDOTDOT * (DT / 2.0)

	else:

		# STATE.
		INT_PASS = 0

		TOF += (DT / 2.0)
		POS = POS0 + VEL * DT
		VEL = VEL0 + ACC * DT
		THT = THT0 + THD * DT
		E = E0 + EDOT * DT
		EDOT = EDOT0 + EDOTDOT * DT

		POS0 = None
		VEL0 = None
		THT0 = None
		E0 = None
		EDOT0 = None

# plot
DF = pd.DataFrame(DATA)
fig = plt.figure()
startIndex = 0
stopIndex = -1

# Window one
trajectory = fig.add_subplot(131, projection="3d")
trajectory.view_init(elev=30, azim=135)
trajectory.set_title("Trajectory")
trajectory.set_xlabel("East")
trajectory.set_ylabel("North")
trajectory.set_zlabel("Up")
xMin = min(list(DF.iloc[startIndex:stopIndex]["X"]) + \
	list(DF.iloc[startIndex:stopIndex]["TGTX"]))
xMax = max(list(DF.iloc[startIndex:stopIndex]["X"]) + \
	list(DF.iloc[startIndex:stopIndex]["TGTX"]))
yMin = 0.0
yMax = 0.0
zMin = min(list(DF.iloc[startIndex:stopIndex]["Y"]) + \
	list(DF.iloc[startIndex:stopIndex]["TGTY"]))
zMax = max(list(DF.iloc[startIndex:stopIndex]["Y"]) + \
	list(DF.iloc[startIndex:stopIndex]["TGTY"]))
trajectory.set_box_aspect(
	(
		np.ptp([xMin - 1000, xMax + 1000]), 
		np.ptp([yMin - 1000, yMax + 1000]), 
		np.ptp([zMin, zMax + 1000]),
	)
)
trajectory.set_xlim([xMin - 1000, xMax + 1000])
trajectory.set_ylim([yMin - 1000, yMax + 1000])
trajectory.set_zlim([zMin, zMax + 1000])
trajectory.plot(
	DF.iloc[startIndex:stopIndex]["X"],
	np.linspace(-0.5, 0.5, len(DF.iloc[startIndex:stopIndex]["X"])),
	DF.iloc[startIndex:stopIndex]["Y"],
	color="b"
)
trajectory.plot(
	DF.iloc[startIndex:stopIndex]["TGTX"],
	np.linspace(0, 0.1, len(DF.iloc[startIndex:stopIndex]["TGTX"])),
	DF.iloc[startIndex:stopIndex]["TGTY"],
	color="r"
)

# Window two
acc = fig.add_subplot(132)
acc.set_title("Performance")
acc.plot(DF.iloc[:]["TOF"], DF.iloc[:]["WDOT"], \
	label="ACHIEVED Gs", color="b")
acc.plot(DF.iloc[:]["TOF"], DF.iloc[:]["CMD"], \
	label="COMMANDED Gs", color="r")
acc.plot(DF.iloc[:]["TOF"], DF.iloc[:]["DEFL"], \
	label="DEFL DEG", color="g")
acc.legend(fontsize="xx-small")

# Window three
rate = fig.add_subplot(133)
rate.set_title("Pitch")
rate.plot(DF.iloc[:]["TOF"], DF.iloc[:]["THT"], \
	label="THETA DEG", color="b")
rate.plot(DF.iloc[:]["TOF"], DF.iloc[:]["THD"], \
	label="THETA DOT DEG PER SEC", color = "r")
rate.legend(fontsize="xx-small")

plt.show()