import numpy as np
from numpy import array as npa
from numpy import linalg as la
import matplotlib
import matplotlib.pyplot as plt
import pandas as pd
import copy

import utility.loggingFxns as lf
import utility.coordinateTransformations as ct
from utility.ATM_IMPERIAL import ATM_IMPERIAL
from utility.interpolationGivenTwoVectors import linearInterpolation
import data.matPlotLibColors as mc

matplotlib.use('WebAgg')
np.set_printoptions(precision=2, suppress=True)

# INPUTS.
ALT = 10000 # FEET
SPD = 300 # FEET PER SEC
EL = 1.0 # DEG
AZ = 0.0 # DEG
MAXT = 100 # SECONDS
DT = 1.0 / 600.0 # SECONDS

# MISSILE CONSTANTS.
REF_DIAM = 0.23 # FEET
REF_LNGTH = 4.59 # FEET
REF_AREA = np.pi * (REF_DIAM ** 2) / 4 # FT^2
BURNOUT = 1.112 # SECONDS

# AERODYNAMIC TABLES.
MACHS_1 = [0.0, 0.6, 0.9, 1.15, 1.3, 1.6, 2.48, 2.97, 100.0] # ND
CMQS = [1060.0, 1060.0, 1460.0, 1385.0,
	1193.0, 1069.0, 850.0, 800.0, 800.0] # PER RAD
CNAS = [8.19, 8.19, 8.94, 9.34, 8.88, 8.14, 7.51, 7.22, 7.22] # PER RAD
XCPS = [36.822500000000005, 36.575, 38.114999999999995, 39.875, 39.93,
	38.0875, 36.8775, 36.739999999999995, 36.739999999999995] # INCHES FROM NOSE

MACHS_2 = [0.0, 0.78, 0.82, 0.9, 0.94, 1.0, 1.03, 1.06, 1.1, 1.15, 1.18,
	1.28, 1.34, 1.48, 1.58, 1.71, 1.94, 2.2, 2.4, 2.6, 3.0, 100.0] # ND
CD_ON = [0.55, 0.55, 0.576, 0.629, 0.65, 0.685, 0.699, 0.71, 0.727, 0.742,
	0.747, 0.76, 0.757, 0.753, 0.742, 0.724,
	0.681, 0.65, 0.628, 0.612, 0.6, 0.6] # ND
CD_OFF = [0.7, 0.7, 0.73, 0.809, 0.863, 0.96, 0.977, 0.989, 1.0, 1.008, 1.01,
	1.012, 1.005, 0.97, 0.97, 0.94, 0.875, 0.8711, 0.765, 0.73, 0.7, 0.7] # ND

MACHS_3 = [0.0, 0.6, 0.9, 1.0, 1.1, 1.15, 1.3, 1.6, 1.9, 2.2, 2.5, 3.0] # ND
CLS = [-0.12, -0.12, -0.12, -0.1, -0.08, -0.07, -0.02, -0.04,
	-0.05, -0.06, -0.06, -0.07] # ND
CLPS = [-5.60, -6.10, -6.40, -6.90, -7.80, -8.05, -8.15,
	-8, -7.60, -7.10, -6.70, -6] # PER RAD

# AERODYNAMICS.
CMQ = linearInterpolation(0.0, MACHS_1, CMQS) # PER RAD
CNA = linearInterpolation(0.0, MACHS_1, CNAS) # PER RAD
XCP = linearInterpolation(0.0, MACHS_1, XCPS) / 12.0 # FEET FROM NOSE
CD = linearInterpolation(0.0, MACHS_2, CD_ON) # ND
CL = linearInterpolation(0.0, MACHS_3, CLS) # ND
CLP = linearInterpolation(0.0, MACHS_3, CLPS) # PER RAD

# ATMOSPHERE.
ATM = ATM_IMPERIAL()
ATM.update(ALT, SPD)
RHO = ATM.rho # SL PER FT^3
G = ATM.g # FT PER S^2
Q = ATM.q # PSF
A = ATM.a # FT PER S
MACH = ATM.mach # ND

# MASS AND MOTOR.
T1S = [0.0, 1.112, 1000.0] # SECONDS
XCGS = [29.96, 33.55, 33.55] # INCHES FROM BASE
TMOIS = [6248.0, 5008.0, 5008.0] # LBM - IN^2
AMOIS = [26.2, 19.7, 19.7] # LBM - IN^2
WEIGHTS = [22.95, 15.73, 15.73] # LBM

T2S = [0.0, 0.012, 0.037, 0.062, 0.187, 0.412, 0.437, 0.462, 0.487, 0.512, 0.537,
	0.562, 0.862, 0.887, 0.912, 0.937, 0.962,
	0.987, 1.037, 1.062, 1.087, 1.112, 1.113, 100.0] # SECONDS
THRUSTS = [0.0, 1304.3, 1400.0, 1439.1, 1245.7, 1109.0, 1267.2, 1276.9, 1451.8,
	1457.7, 1267.2, 1234.0, 1522.2, 1485.0, 1611.1,
	1654.1, 1780.1, 1792.8, 1463.5, 1070.8, 491.4, 146.6, 0.0, 0.0] # LBF
TORQUES = [0.0, 39.1, 42.0, 43.2, 37.4, 35.7, 38.0, 38.3, 43.6, 43.7,
	38.0, 37.0, 45.7, 44.6, 48.3, 49.6, 53.4, 53.8,
	43.9, 32.1, 14.3, 4.4, 0.0, 0.0] # LBF - IN

XCG = REF_LNGTH - (linearInterpolation(0.0, T1S, XCGS) / 12.0) # FT
TMOI = (linearInterpolation(0.0, T1S, TMOIS)) / (144.0 * 32.2) # LBF - FT - S^2
AMOI = (linearInterpolation(0.0, T1S, AMOIS)) / (144.0 * 32.2) # LBF - FT - S^2
MASS = linearInterpolation(0.0, T1S, WEIGHTS) # LBM
THRUST = linearInterpolation(0.0, T2S, THRUSTS) # LBF
TORQUE = linearInterpolation(0.0, T2S, TORQUES) / 12.0 # LBF - FT

# ATTITUDE.
ENU2FLU = ct.ORIENTATION_TO_LOCAL_TM(0.0, \
	-1.0 * np.radians(EL), np.radians(AZ)) # ND

# DERIVATIVES.
SPECIFIC_FORCE = np.zeros(3) # FT PER S^2
ACC = np.zeros(3) # FT PER S^2
PHIDOT = 0.0 # RADS PER SEC
THTDOT = 0.0 # RADS PER SEC
PSIDOT = 0.0 # RADS PER SEC
PDOT = 0.0 # RADS PER S^2
QDOT = 0.0 # RADS PER S^2
RDOT = 0.0 # RADS PER S^2
ADOT = 0.0 # RADS PER S
BDOT = 0.0 # RADS PER S

# STATE.
TOF = 0.0 # SECONDS
POS = npa([0.0, 0.0, ALT]) # FT
VEL = npa([SPD, 0.0, 0.0]) @ ENU2FLU # FT PER S
PHI = 0.0 # RADS
THT = np.radians(EL) # RADS
PSI = np.radians(AZ) # RADS
ROLLRATE = 0.0 # RADS PER S
PITCHRATE = 0.0 # RADS PER S
YAWRATE = 0.0 # RADS PER S
ALPHA = 0.0 # RADS
BETA = 0.0 # RADS

INT_PASS = 0 # RK2 INTEGRATION
POS0 = None # FT
VEL0 = None # FT PER S
PHI0 = None # RADS
THT0 = None # RADS
PSI0 = None # RADS
ROLLRATE0 = None # RADS PER S
PITCHRATE0 = None # RADS PER S
YAWRATE0 = None # RADS PER S
ALPHA0 = None # RADS
BETA0 = None # RADS

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

# LOG
STATE = populateState()
LOGFILE = open("PY_6DOF_70MM_ROCKET/data/log.txt", "w")
lf.writeHeader(STATE, LOGFILE)
lf.writeData(STATE, LOGFILE)

LASTT = None # SECONDS
while True:

	# ATTITUDE.
	ENU2FLU = ct.ORIENTATION_TO_LOCAL_TM(PHI, -1.0 * THT, PSI) # ND

	# MASS AND MOTOR.
	XCG = REF_LNGTH - (linearInterpolation(TOF, T1S, XCGS) / 12.0) # FT FROM NOSE
	TMOI = (linearInterpolation(TOF, T1S, TMOIS)) / (144.0 * 32.2) # LBF - FT - S^2
	AMOI = (linearInterpolation(TOF, T1S, AMOIS)) / (144.0 * 32.2) # LBF - FT - S^2
	MASS = linearInterpolation(TOF, T1S, WEIGHTS) # LBM
	THRUST = linearInterpolation(TOF, T2S, THRUSTS) # LBF
	TORQUE = linearInterpolation(TOF, T2S, TORQUES) / 12.0 # LBF - FT

	ACC_THRUST = (THRUST / MASS) * 32.2 # FT PER S^2
	ANGACC_TORQUE = (2 * TORQUE / (MASS * (REF_DIAM / 2) * \
		(REF_DIAM / 2))) * 32.2 # RADS PER S^2

	# ATMOSPHERE.
	SPD = la.norm(VEL)
	ATM.update(POS[2], SPD)
	RHO = ATM.rho # SL PER FT^3
	G = ATM.g # FT PER S^2
	Q = ATM.q # PSF
	A = ATM.a # FT PER S
	MACH = ATM.mach # ND
	LOCALG = npa([0.0, 0.0, -1.0 * G]) # FT PER S^2
	BODY_GRAV = ENU2FLU @ LOCALG # FT PER S^2

	# AERODYNAMICS. #
	# PITCH AND YAW.
	# DIVIDED CMQ BY 2.0, ALLOWS FOR BALLISTIC FLIGHT.
	CMQ = linearInterpolation(MACH, MACHS_1, CMQS) # PER RAD
	CNA = linearInterpolation(MACH, MACHS_1, CNAS) # PER RAD
	XCP = linearInterpolation(MACH, MACHS_1, XCPS) / 12.0 # FT FROM NOSE
	CZ = CNA * ALPHA # ND
	CM = CZ * (XCG - XCP) / REF_DIAM + \
		(REF_DIAM / (2 * SPD)) * (CMQ / 2) * PITCHRATE # ND
	CY = CNA * BETA # ND
	CN = CY * (XCG - XCP) / REF_DIAM + \
		(REF_DIAM / (2 * SPD)) * (CMQ / 2) * YAWRATE # ND

	# DRAG.
	CD = None # ND
	if TOF < BURNOUT:
		CD = linearInterpolation(MACH, MACHS_2, CD_ON) # ND
	else:
		CD = linearInterpolation(MACH, MACHS_2, CD_OFF) # ND
	DRAG_FORCE = CD * REF_AREA * Q # LBF
	WIND_TO_BODY = ct.FLIGHTPATH_TO_LOCAL_TM(BETA, ALPHA) # ND
	WIND_DRAG_FORCE = npa([-DRAG_FORCE, 0.0, 0.0]) # LBF
	BODY_DRAG = ((WIND_TO_BODY @ WIND_DRAG_FORCE) / MASS) * 32.2 # FT PER S^2

	# ROLL.
	CL = linearInterpolation(MACH, MACHS_3, CLS) # ND
	CLP = linearInterpolation(MACH, MACHS_3, CLPS) # PER RAD
	CLL = CL + CLP * ROLLRATE * REF_DIAM / (2 * SPD) # ND

	# DERIVATIVES.
	WDOT = ((Q * REF_AREA * CZ) * (32.2 / MASS)) # FT PER S^2
	VDOT = ((Q * REF_AREA * CY) * (32.2 / MASS)) # FT PER S^2
	SPECIFIC_FORCE = npa([ACC_THRUST, VDOT, WDOT]) # FT PER S^2
	SPECIFIC_FORCE += BODY_GRAV # ADD GRAVITY
	SPECIFIC_FORCE += BODY_DRAG # ADD DRAG
	ACC = (SPECIFIC_FORCE @ ENU2FLU) # FT PER S^2
	PHIDOT = ROLLRATE + (PITCHRATE * np.sin(PHI) + YAWRATE * np.cos(PHI))\
		 * np.tan(THT) # RADS PER SEC
	THTDOT = PITCHRATE * np.cos(PHI) - YAWRATE * np.sin(PHI) # RADS PER SEC
	PSIDOT = (PITCHRATE * np.sin(PHI) + YAWRATE * np.cos(PHI)) \
		/ np.cos(THT) # RADS PER SEC
	PDOT = ((Q * REF_AREA * REF_DIAM * CLL) / AMOI) + ANGACC_TORQUE # RADS PER S^2
	QDOT = (Q * REF_AREA * REF_DIAM * CM) / TMOI # RADS PER S^2
	RDOT = (Q * REF_AREA * REF_DIAM * CN) / TMOI # RADS PER S^2
	ADOT = PITCHRATE - (SPECIFIC_FORCE[2] / SPD) # RADS PER S
	BDOT = YAWRATE - (SPECIFIC_FORCE[1] / SPD) # RADS PER S

	# STATE.
	if INT_PASS == 0:

		# DATA.
		STATE = populateState()
		lf.writeData(STATE, LOGFILE)

		# REPORT.
		if round(TOF, 3).is_integer() and LASTT != TOF:
			print(f"{TOF:.0f} POS {POS} MACH {MACH:.2f}")
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

		# BEGIN INTEGRATION PASS.
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
			TEMP = (2 * np.pi) - PHI
			PHI = TEMP

# SET UP MATPLOTLIB FIG.
DATFILE = open("PY_6DOF_70MM_ROCKET/data/log.txt", "r")
DF = pd.read_csv(DATFILE, delim_whitespace=True)
FIG = plt.figure()
START = 0
STOP = -1
COLORS = mc.matPlotLibColors()

# TRAJECTORY.
TRAJ = FIG.add_subplot(221, projection="3d")
TRAJ.set_title("TRAJECTORY")
TRAJ.set_xlabel("EAST.")
TRAJ.set_ylabel("NORTH.")
TRAJ.set_zlabel("UP.")
XMIN = min(list(DF.iloc[START:STOP]["RNG"]))
XMAX = max(list(DF.iloc[START:STOP]["RNG"]))
YMIN = min(list(DF.iloc[START:STOP]["CROSSRNG"]))
YMAX = max(list(DF.iloc[START:STOP]["CROSSRNG"]))
ZMIN = min(list(DF.iloc[START:STOP]["ALT"]))
ZMAX = max(list(DF.iloc[START:STOP]["ALT"]))
TRAJ.set_box_aspect(
	(
		np.ptp([XMIN - 1000, XMAX + 1000]), 
		np.ptp([YMIN - 5000, YMAX + 5000]), 
		np.ptp([ZMIN, ZMAX + 1000]),
	)
)
TRAJ.set_xlim([XMIN - 1000, XMAX + 1000])
TRAJ.set_ylim([YMIN - 5000, YMAX + 5000])
TRAJ.set_zlim([ZMIN, ZMAX + 1000])
TRAJ.plot(
	DF.iloc[START:STOP]["RNG"],
	DF.iloc[START:STOP]["CROSSRNG"],
	DF.iloc[START:STOP]["ALT"],
	color="b"
)

# PITCH RATE.
PITCHRATE = FIG.add_subplot(222)
PITCHRATE.set_xlabel("TOF")
PITCHRATE.set_ylabel("Q, RADS PER SEC.")
PITCHRATE.plot(
	DF.iloc[START:STOP]["TOF"],
	DF.iloc[START:STOP]["PITCHRATE"],
	color=COLORS.pop(0)
)

# ROLL RATE.
ROLLRATE = FIG.add_subplot(223)
ROLLRATE.set_xlabel("TOF")
ROLLRATE.set_ylabel("P, HERTZ.")
ROLLRATE.plot(
	DF.iloc[START:STOP]["TOF"],
	DF.iloc[START:STOP]["ROLLRATE"] / 6.28,
	color=COLORS.pop(0)
)

# YAW RATE.
YAWRATE = FIG.add_subplot(224)
YAWRATE.set_xlabel("TOF")
YAWRATE.set_ylabel("R, RADS PER SEC.")
YAWRATE.plot(
	DF.iloc[START:STOP]["TOF"],
	DF.iloc[START:STOP]["YAWRATE"],
	color=COLORS.pop(0)
)

# SHOW.
plt.show()














