import numpy as np
import copy
from numpy import array as npa
from numpy import linalg as la
import utility.loggingFxns as lf
import utility.coordinateTransformations as ct
from utility.ATM_IMPERIAL import ATM_IMPERIAL
from utility.interpolationGivenTwoVectors import linearInterpolation
import matplotlib.pyplot as plt
import pandas as pd
import data.matPlotLibColors as mc
import matplotlib
matplotlib.use('WebAgg')
np.set_printoptions(precision=2, suppress=True)

# INPUTS.
ALT = 10 # FEET
SPD = 10 # FEET PER SEC
EL = 45.0 # DEG
AZ = 0.0 # DEG

# MISSILE CONSTANTS.
REF_DIAM = 0.23 # FEET
REF_LNGTH = 4.59 # FEET
REF_AREA = np.pi * (REF_DIAM ** 2) / 4 # ft^2
BURNOUT = 1.112 # seconds

# AERODYNAMIC TABLES.
# 70 MM ROCKET TABLES.
MACHS_1 = [0.0, 0.6, 0.9, 1.15, 1.3, 1.6, 2.48, 2.97, 100.0]
CMQS = [1060.0, 1060.0, 1460.0, 1385.0, 1193.0, 1069.0, 850.0, 800.0, 800.0]
CNAS = [8.19, 8.19, 8.94, 9.34, 8.88, 8.14, 7.51, 7.22, 7.22]
XCPS = [36.822500000000005, 36.575, 38.114999999999995, 39.875, 39.93, \
	38.0875, 36.8775, 36.739999999999995, 36.739999999999995] # inches from nose

MACHS_2 = [0.0, 0.78, 0.82, 0.9, 0.94, 1.0, 1.03, 1.06, 1.1, 1.15, 1.18, \
	1.28, 1.34, 1.48, 1.58, 1.71, 1.94, 2.2, 2.4, 2.6, 3.0, 100.0]
CD_ON = [0.55, 0.55, 0.576, 0.629, 0.65, 0.685, 0.699, 0.71, 0.727, 0.742, \
	0.747, 0.76, 0.757, 0.753, 0.742, 0.724, 0.681, 0.65, 0.628, 0.612, 0.6, 0.6]
CD_OFF = [0.7, 0.7, 0.73, 0.809, 0.863, 0.96, 0.977, 0.989, 1.0, 1.008, 1.01, \
	1.012, 1.005, 0.97, 0.97, 0.94, 0.875, 0.8711, 0.765, 0.73, 0.7, 0.7]

MACHS_3 = [0.0, 0.6, 0.9, 1.0, 1.1, 1.15, 1.3, 1.6, 1.9, 2.2, 2.5, 3.0]
CLS = [-0.12, -0.12, -0.12, -0.1, -0.08, -0.07, -0.02, -0.04, \
	-0.05, -0.06, -0.06, -0.07]
CLDELTAS = [2.92,
2.98,
3.09,
3.21,
3.49,
3.67,
4.01,
3.90,
3.49,
3.09,
2.81,
2.29
]
CLPS = [
	-5.60,
-6.10,
-6.40,
-6.90,
-7.80,
-8.05,
-8.15,
-8,
-7.60,
-7.10,
-6.70,
-6
]

# AERODYNAMICS.
CMQ = linearInterpolation(0.0, MACHS_1, CMQS)
CNA = linearInterpolation(0.0, MACHS_1, CNAS)
XCP = linearInterpolation(0.0, MACHS_1, XCPS) / 12.0
CD = linearInterpolation(0.0, MACHS_2, CD_ON)
CL = linearInterpolation(0.0, MACHS_3, CLS)
CLD = linearInterpolation(0.0, MACHS_3, CLDELTAS)
CLP = linearInterpolation(0.0, MACHS_3, CLPS)

# ATMOSPHERE.
ATM = ATM_IMPERIAL()
ATM.update(ALT, SPD)
RHO = ATM.rho
G = ATM.g
Q = ATM.q
A = ATM.a
MACH = ATM.mach

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

# ATTITUDE.
ENU2FLU = ct.ORIENTATION_TO_LOCAL_TM(0.0, -1.0 * np.radians(EL), np.radians(AZ))

# DERIVATIVES.
SPECIFIC_FORCE = np.zeros(3) # feet/s^2
ACC = np.zeros(3) # feet/s^2
PHIDOT = 0.0 # rads/s
THTDOT = 0.0 # rads/s
PSIDOT = 0.0 # rads/s
PDOT = 0.0 # rads/s^2
QDOT = 0.0 # rads/s^2
RDOT = 0.0 # rads/s^2
ADOT = 0.0 # alpha dot, rads/s
BDOT = 0.0 # beta dot, rads/s

# STATE.
POS = npa([0.0, 0.0, ALT]) # feet
VEL = npa([SPD, 0.0, 0.0]) @ ENU2FLU # ft/s
PHI = 0.0 # rads
THT = np.radians(EL) # rads
PSI = np.radians(AZ) # rads
ROLLRATE = 0.0 # rads/s
PITCHRATE = 0.0 # rads/s
YAWRATE = 0.0 # rads/s
ALPHA = 0.0 # radians
BETA = 0.0 # radians

INT_PASS = 0
POS0 = None
VEL0 = None
PHI0 = None
THT0 = None
PSI0 = None
ROLLRATE0 = None
PITCHRATE0 = None
YAWRATE0 = None
ALPHA0 = None
BETA0 = None

# SIM CONTROL.
TOF = 0.0
DT = 1.0 / 600.0
MAXT = 10

# DATA
def populateState():
	STATE = {
		"TOF": TOF,
		"RNG": POS[0],
		"CROSSRNG": POS[1],
		"ALT": POS[2],
		"PHI": PHI,
		"THETA": THT,
		"PSI": PSI,
		"ROLLRATE": ROLLRATE,
		"PITCHRATE": PITCHRATE,
		"YAWRATE": YAWRATE,
		"ALPHA": ALPHA,
		"BETA": BETA,
		"ALPHADOT": ADOT,
		"BETADOT": BDOT,
		"PDOT": PDOT,
		"QDOT": QDOT,
		"RDOT": RDOT,
		"UDOT": SPECIFIC_FORCE[0],
		"VDOT": SPECIFIC_FORCE[1],
		"WDOT": SPECIFIC_FORCE[2]
	}
	return STATE

STATE = populateState()
LOGFILE = open("PY_6DOF_70MM_ROCKET/data/log.txt", "w")
lf.writeHeader(STATE, LOGFILE)
lf.writeData(STATE, LOGFILE)

LASTT = 0.0
while True:

	# ATTITUDE.
	ENU2FLU = ct.ORIENTATION_TO_LOCAL_TM(PHI, -1.0 * THT, PSI)

	# MASS AND MOTOR. (NO MOTOR FOR NOW.)
	THRUST = linearInterpolation(TOF, T2S, THRUSTS) # LBF
	XCG = REF_LNGTH - (linearInterpolation(TOF, T1S, XCGS) / 12.0) # FT
	MASS = linearInterpolation(TOF, T1S, WEIGHTS) # LBM
	TMOI = (linearInterpolation(TOF, T1S, TMOIS)) / (144.0 * 32.2) # LBF - FT - S^2

	# ATMOSPHERE.
	SPD = la.norm(VEL)
	ATM.update(POS[2], SPD)
	RHO = ATM.rho
	G = ATM.g
	Q = ATM.q
	A = ATM.a
	MACH = ATM.mach

	# AERODYNAMICS
	CMQ = linearInterpolation(MACH, MACHS_1, CMQS)
	CNA = linearInterpolation(MACH, MACHS_1, CNAS)
	XCP = linearInterpolation(MACH, MACHS_1, XCPS) / 12.0
	CZ = CNA * ALPHA
	CM = CZ * (XCG - XCP) / REF_DIAM + \
		(REF_DIAM / (2 * SPD)) * (CMQ / 1) * PITCHRATE # divided CMQ by 2.0
	CY = CNA * BETA
	CN = CY * (XCG - XCP) / REF_DIAM + \
		(REF_DIAM / (2 * SPD)) * (CMQ / 1) * YAWRATE

	CD = None
	if TOF < BURNOUT:
		CD = linearInterpolation(MACH, MACHS_2, CD_ON)
	else:
		CD = linearInterpolation(MACH, MACHS_2, CD_OFF)
	DRAG_FORCE = CD * REF_AREA * Q # Newtons.
	WIND_TO_BODY = ct.FLIGHTPATH_TO_LOCAL_TM(BETA, ALPHA)
	WIND_DRAG_FORCE = npa([-DRAG_FORCE, 0.0, 0.0])
	BODY_DRAG = ((WIND_TO_BODY @ WIND_DRAG_FORCE) / MASS) * 32.2

	CL = linearInterpolation(0.0, MACHS_3, CLS)
	CLD = linearInterpolation(0.0, MACHS_3, CLDELTAS)
	CLP = linearInterpolation(0.0, MACHS_3, CLPS)

	# DERIVATIVES.
	ADOT = PITCHRATE - (SPECIFIC_FORCE[2] / SPD) # RADS PER S
	BDOT = YAWRATE - (SPECIFIC_FORCE[1] / SPD) # RADS PER S
	PHIDOT = 7.0 # rads per sec
	THTDOT = PITCHRATE * np.cos(0.0) - YAWRATE * np.sin(0.0)
	PSIDOT = (PITCHRATE * np.sin(0.0) + YAWRATE * np.cos(0.0)) / np.cos(THT)
	PDOT = 0.0 # rads per s^2
	QDOT = (Q * REF_AREA * REF_DIAM * CM) / TMOI # RADS PER S^2
	RDOT = (Q * REF_AREA * REF_DIAM * CN) / TMOI # RADS PER S^2
	WDOT = ((Q * REF_AREA * CZ) * (32.2 / MASS)) # FT PER S^2
	VDOT = ((Q * REF_AREA * CY) * (32.2 / MASS)) # FT PER S^2

	ACC_THRUST = (THRUST / MASS) * 32.2
	SPECIFIC_FORCE = npa([ACC_THRUST, VDOT, WDOT])
	LOCALG = npa([0.0, 0.0, -1.0 * G])
	BODY_GRAV = ENU2FLU @ LOCALG
	SPECIFIC_FORCE += BODY_GRAV
	SPECIFIC_FORCE += BODY_DRAG
	ACC = (SPECIFIC_FORCE @ ENU2FLU)

	if TOF > 1.0:
		pause = None

	# STATE.
	if INT_PASS == 0:

		# DATA.
		STATE = populateState()
		lf.writeData(STATE, LOGFILE)

		# REPORT.
		if round(TOF, 3).is_integer() and LASTT <= TOF:
			print(f"{TOF:.0f} POS {POS} MACH {MACH:.2f} PHI {PHI:.2f}")
			LASTT = TOF

		# END CHECK.
		if TOF > MAXT:
			print(f"{TOF:.3f} POS {POS} MACH {MACH:.2f}")
			print("TIME")
			break
		if POS[2] < 0.0:
			print(f"{TOF:.3f} POS {POS} MACH {MACH:.2f}")
			print("GROUND")
			break
		if np.isnan(POS[1]):
			print(f"{TOF:.3f} POS {POS} MACH {MACH:.2f}")
			print("NAN")
			break

		INT_PASS += 1
		TOF += (DT / 2.0)

		POS0 = copy.deepcopy(POS)
		VEL0 = copy.deepcopy(VEL)
		PHI0 = copy.deepcopy(PHI)
		THT0 = copy.deepcopy(THT)
		PSI0 = copy.deepcopy(PSI)
		ROLLRATE0 = copy.deepcopy(ROLLRATE)
		PITCHRATE0 = copy.deepcopy(PITCHRATE)
		YAWRATE0 = copy.deepcopy(YAWRATE)
		ALPHA0 = copy.deepcopy(ALPHA)
		BETA0 = copy.deepcopy(BETA)

		POS += VEL * (DT / 2.0)
		VEL += ACC * (DT / 2.0)
		PHI += PHIDOT * (DT / 2.0)
		THT += THTDOT * (DT / 2.0)
		PSI += PSIDOT * (DT / 2.0)
		ROLLRATE += PDOT * (DT / 2.0)
		PITCHRATE += QDOT * (DT / 2.0)
		YAWRATE += RDOT * (DT / 2.0)
		ALPHA += ADOT * (DT / 2.0)
		BETA += BDOT * (DT / 2.0)

		if PHI > (2 * np.pi):
			TEMP = PHI - (2 * np.pi)
			PHI = 0.0 + TEMP
		elif PHI < 0.0:
			TEMP = 360.0 - PHI
			PHI = TEMP

	else:

		INT_PASS = 0
		TOF += (DT / 2.0)

		POS = POS0 + VEL * DT
		VEL = VEL0 + ACC * DT
		PHI = PHI0 + PHIDOT * DT
		THT = THT0 + THTDOT * DT
		PSI = PSI0 + PSIDOT * DT
		ROLLRATE = ROLLRATE0 + PDOT * DT
		PITCHRATE = PITCHRATE0 + QDOT * DT
		YAWRATE = YAWRATE0 + RDOT * DT
		ALPHA = ALPHA0 + ADOT * DT
		BETA = BETA0 + BDOT * DT

		POS0 = None
		VEL0 = None
		PHI0 = None
		THT0 = None
		PSI0 = None
		ROLLRATE0 = None
		PITCHRATE0 = None
		YAWRATE0 = None
		ALPHA0 = None
		BETA0 = None

		if PHI > (2 * np.pi):
			TEMP = PHI - (2 * np.pi)
			PHI = 0.0 + TEMP
		elif PHI < 0.0:
			TEMP = 360.0 - PHI
			PHI = TEMP

# plot
f = open("PY_6DOF_70MM_ROCKET/data/log.txt", "r")
df = pd.read_csv(f, delim_whitespace=True)
fig = plt.figure()
startIndex = 0
stopIndex = -1
colors = mc.matPlotLibColors()
trajectory = fig.add_subplot(221, projection="3d")
trajectory.set_title("Trajectory")
trajectory.set_xlabel("East")
trajectory.set_ylabel("North")
trajectory.set_zlabel("Up")
xMin = min(list(df.iloc[startIndex:stopIndex]["RNG"]))
xMax = max(list(df.iloc[startIndex:stopIndex]["RNG"]))
yMin = min(list(df.iloc[startIndex:stopIndex]["CROSSRNG"]))
yMax = max(list(df.iloc[startIndex:stopIndex]["CROSSRNG"]))
zMin = min(list(df.iloc[startIndex:stopIndex]["ALT"]))
zMax = max(list(df.iloc[startIndex:stopIndex]["ALT"]))
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
trajectory.plot(df.iloc[startIndex:stopIndex]["RNG"], \
	df.iloc[startIndex:stopIndex]["CROSSRNG"], \
	df.iloc[startIndex:stopIndex]["ALT"], color="b")
pitchrate = fig.add_subplot(222)
pitchrate.plot(df.iloc[startIndex:stopIndex]["TOF"], \
	df.iloc[startIndex:stopIndex]["PITCHRATE"], \
	color=colors.pop(0))
pitchrate.set_xlabel("TOF")
pitchrate.set_ylabel("PITCHRATE, RADS PER SEC")
alpha = fig.add_subplot(223)
alpha.plot(df.iloc[startIndex:stopIndex]["TOF"], \
	df.iloc[startIndex:stopIndex]["ALPHA"], \
	color=colors.pop(0))
alpha.set_xlabel("TOF")
alpha.set_ylabel("ALPHA, RADS")
wdot = fig.add_subplot(224)
wdot.plot(df.iloc[startIndex:stopIndex]["TOF"], \
	df.iloc[startIndex:stopIndex]["WDOT"], \
	color=colors.pop(0))
wdot.set_xlabel("TOF")
wdot.set_ylabel("WDOT, FT PER S^2")
plt.show()














