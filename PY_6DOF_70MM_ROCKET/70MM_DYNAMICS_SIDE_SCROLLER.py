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
ALT = 1000 # FEET
SPD = 10 # FEET PER SEC
EL = 45.0 # DEG

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

# AERODYNAMICS.
CMQ = linearInterpolation(0.0, MACHS_1, CMQS)
CNA = linearInterpolation(0.0, MACHS_1, CNAS)
XCP = linearInterpolation(0.0, MACHS_1, XCPS) / 12.0
CD = linearInterpolation(0.0, MACHS_2, CD_ON)

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
LOCAL_TO_BODY_TM = ct.BODY_TO_RANGE_AND_ALTITUDE(-1.0 * np.radians(EL))

# DERIVATIVES.
SPECIFIC_FORCE = np.zeros(2)
ACC = np.zeros(2)
ADOT = 0.0
QDOT = 0.0
WDOT = 0.0

# STATE.
POS = npa([0.0, ALT])
VEL = npa([SPD, 0.0]) @ LOCAL_TO_BODY_TM
THT = np.radians(EL)
RATE = 0.0
ALPHA = 0.0

INT_PASS = 0
POS0 = None
VEL0 = None
THT0 = None
RATE0 = None
ALPHA0 = None

# SIM CONTROL.
TOF = 0.0
DT = 1.0 / 1000.0
MAXT = 100

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

	# ATTITUDE.
	LOCAL_TO_BODY_TM = ct.BODY_TO_RANGE_AND_ALTITUDE(-1.0 * THT)

	# MASS AND MOTOR. (NO MOTOR FOR NOW.)
	THRUST = linearInterpolation(TOF, T2S, THRUSTS) # LBF
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

	# AERODYNAMICS
	CMQ = linearInterpolation(MACH, MACHS_1, CMQS)
	CNA = linearInterpolation(MACH, MACHS_1, CNAS)
	XCP = linearInterpolation(MACH, MACHS_1, XCPS) / 12.0
	CN = CNA * ALPHA

	# I divided the damping by two to enable the missile
	# to fly ballistically. I'm hoping that by adding
	# the rolling moment it will have a similar affect.
	CM = CN * (XCG - XCP) / REF_DIAM + \
		(REF_DIAM / (2 * SPD)) * (CMQ / 2) * RATE 

	CD = None
	if TOF < BURNOUT:
		CD = linearInterpolation(MACH, MACHS_2, CD_ON)
	else:
		CD = linearInterpolation(MACH, MACHS_2, CD_OFF)
	DRAG_FORCE = CD * REF_AREA * Q # Newtons.
	WIND_TO_BODY = ct.BODY_TO_RANGE_AND_ALTITUDE(ALPHA)
	WIND_DRAG_FORCE = npa([-DRAG_FORCE, 0.0])
	BODY_DRAG = ((WIND_TO_BODY @ WIND_DRAG_FORCE) / MASS) * 32.2

	# DERIVATIVES.
	ADOT = RATE - (SPECIFIC_FORCE[1] / SPD) # RADS PER S
	QDOT = (Q * REF_AREA * REF_DIAM * CM) / TMOI # RADS PER S^2
	WDOT = ((Q * REF_AREA * CN) * (G / MASS)) # FT PER S^2

	ACC_THRUST = (THRUST / MASS) * 32.2
	SPECIFIC_FORCE = npa([ACC_THRUST, WDOT])
	LOCALG = npa([0.0, -1.0 * G])
	BODYG = LOCAL_TO_BODY_TM @ LOCALG
	SPECIFIC_FORCE += BODYG
	SPECIFIC_FORCE += BODY_DRAG
	ACC = (SPECIFIC_FORCE @ LOCAL_TO_BODY_TM)

	if TOF > 1.0:
		pause = None

	# STATE.
	if INT_PASS == 0:

		# DATA.
		STATE = populateState()
		lf.writeData(STATE, LOGFILE)

		# REPORT.
		if round(TOF, 3).is_integer() and LASTT <= TOF:
			print(f"{TOF:.0f} POS {POS} MACH {MACH}")
			LASTT = TOF

		# END CHECK.
		if POS[1] < 0.0:
			print("GROUND")
			break
		if np.isnan(POS[1]):
			print("NAN")
			break

		INT_PASS += 1
		TOF += (DT / 2.0)

		POS0 = copy.deepcopy(POS)
		VEL0 = copy.deepcopy(VEL)
		THT0 = copy.deepcopy(THT)
		RATE0 = copy.deepcopy(RATE)
		ALPHA0 = copy.deepcopy(ALPHA)

		POS += VEL * (DT / 2.0)
		VEL += ACC * (DT / 2.0)
		THT += RATE * (DT / 2.0)
		RATE += QDOT * (DT / 2.0)
		ALPHA += ADOT * (DT / 2.0)

	else:

		INT_PASS = 0
		TOF += (DT / 2.0)

		POS = POS0 + VEL * DT
		VEL = VEL0 + ACC * DT
		THT = THT0 + RATE * DT
		RATE = RATE0 + QDOT * DT
		ALPHA = ALPHA0 + ADOT * DT

		POS0 = None
		VEL0 = None
		THT0 = None
		RATE0 = None
		ALPHA0 = None

# REPORT.
if round(TOF, 3).is_integer() and LASTT <= TOF:
	print(f"{TOF:.0f} POS {POS}")
	LASTT = TOF

f = open("PY_6DOF_70MM_ROCKET/data/log.txt", "r")
df = pd.read_csv(f, delim_whitespace=True)

fig = plt.figure()

startIndex = 0
stopIndex = -1

colors = mc.matPlotLibColors()

ax1 = fig.add_subplot(221)
ax1.plot(df.iloc[startIndex:stopIndex]["RNG"], df.iloc[startIndex:stopIndex]["ALT"], \
	color=colors.pop(0))
ax1.set_title("TRAJECTORY, FEET")

ax2 = fig.add_subplot(222)
ax2.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["RATE"], \
	color=colors.pop(0))
ax2.set_xlabel("TOF")
ax2.set_ylabel("RATE, RADS PER SEC")

ax3 = fig.add_subplot(223)
ax3.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["ALPHA"], \
	color=colors.pop(0))
ax3.set_xlabel("TOF")
ax3.set_ylabel("ALPHA, RADS")

ax4 = fig.add_subplot(224)
ax4.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["WDOT"], \
	color=colors.pop(0))
ax4.set_xlabel("TOF")
ax4.set_ylabel("WDOT, FT PER S^2")

plt.show()














