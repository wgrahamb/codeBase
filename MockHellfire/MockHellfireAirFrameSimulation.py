# Python libraries.
import numpy as np
from numpy import array as npa
from numpy import linalg as la
import pandas as pd
import matplotlib.pyplot as plt

# Utility.
from utility.returnAzAndElevation import returnEl
from utility import coordinateTransformations as ct

# Classes.
from classes.Atmosphere import Atmosphere
from classes.MockHellfireMassAndMotor import MockHellfireMassAndMotor

"""

TO DO:
REAL MOTOR MODEL.

MOCK HELLFIRE DIMENSIONS

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
UNCORRECTED_CENTER_OF_DEFLECTION_FROM_NOSE 1.8059 M
UNCORRECTED_REFERENCE_LENGTH 1.85026 m

"""

# CONSTANTS.
TIME_STEP = 0.001 # Seconds.
MAX_TIME = 3 # Seconds.
MM_TO_M = 1.0 / 1000.0
RAD_TO_DEG = 57.2957795130823
INITIAL_ALTITUDE = 1000 # Meters.
INITIAL_AIRSPEED = 130 # Meters per second.
FIN_DEFLECTION_DEGREES = 0 # Degrees.
FIN_DEFLECTION_RADIANS = np.radians(FIN_DEFLECTION_DEGREES) # Radians.

# MOCK HELLFIRE PARAMETERS.
REFERENCE_DIAMETER = 0.18 # Meters.
NOSE_LENGTH = 0.249733 # Meters.
# REFERENCE_LENGTH = 1.6 # Meters.
REFERENCE_LENGTH = 1.85026 # Meters.
WING_HALF_SPAN = 66.1175 * MM_TO_M / 2.0 # Meters.
WING_TIP_CHORD = 91.047 * MM_TO_M # Meters.
WING_ROOT_CHORD = 0.123564 # Meters.
TAIL_HALF_SPAN = 71.3548 * MM_TO_M / 2.0 # Meters.
TAIL_TIP_CHORD = 0.387894 # Meters.
TAIL_ROOT_CHORD = 0.48084 # Meters.
DISTANCE_FROM_BASE_OF_NOSE_TO_WING = 0.323925 # Meters.
# CENTER_OF_DEFLECTION_FROM_NOSE = 1.8059 - NOSE_LENGTH # Meters (correction here due to oversight in drawing).
CENTER_OF_DEFLECTION_FROM_NOSE = 1.8059 # Meters.
WING_AREA = 0.5 * WING_HALF_SPAN * (WING_TIP_CHORD + WING_ROOT_CHORD) # Meters squared.
TAIL_AREA = 0.5 * TAIL_HALF_SPAN * (TAIL_TIP_CHORD + TAIL_ROOT_CHORD) # Meters squared.
REFERENCE_AREA = np.pi * (REFERENCE_DIAMETER ** 2) / 4 # Meters squared.
NOSE_AREA = NOSE_LENGTH * REFERENCE_DIAMETER # Meters squared.
PLANFORM_AREA = (REFERENCE_LENGTH - NOSE_LENGTH) * REFERENCE_DIAMETER + 0.667 * NOSE_LENGTH * REFERENCE_DIAMETER # Meters squared.
NOSE_CENTER_OF_PRESSURE = 0.67 * NOSE_LENGTH # Meters.
WING_CENTER_OF_PRESSURE = NOSE_LENGTH + DISTANCE_FROM_BASE_OF_NOSE_TO_WING + 0.7 * WING_ROOT_CHORD - 0.2 * WING_TIP_CHORD # Meters.
AN = 0.67 * NOSE_LENGTH * REFERENCE_DIAMETER # Meters squared.
AB = (REFERENCE_LENGTH - NOSE_LENGTH) * REFERENCE_DIAMETER # Meters squared.
BODY_CENTER_OF_PRESSURE = (0.67 * AN * NOSE_LENGTH + AB * (NOSE_LENGTH + 0.5 * (REFERENCE_LENGTH - NOSE_LENGTH))) / (AN + AB) # Meters.

# ATMOSPHERE.
ATMOS = Atmosphere()
ATMOS.update(INITIAL_ALTITUDE, INITIAL_AIRSPEED)
RHO = ATMOS.rho # Pascals.
Q = ATMOS.q # Pascals.
P = ATMOS.p
A = ATMOS.a
G = ATMOS.g
MACHSPEED = ATMOS.mach
if MACHSPEED > 1:
	BETA = np.sqrt(MACHSPEED ** 2 - 1) # Non dimensional.
else:
	BETA = 0.1 # Non dimensional.

# MASS AND MOTOR PROPERTIES.
MASS_AND_MOTOR = MockHellfireMassAndMotor()
MASS_AND_MOTOR.update(0.0, P)
XCG = MASS_AND_MOTOR.CG_VALUES[0]
MASS = MASS_AND_MOTOR.MASS
TMOI = MASS_AND_MOTOR.TRANSVERSE_MOI
THRUST = MASS_AND_MOTOR.THRUST

# STATE. #########################################
TOF = 0.0 # Seconds.
TDOT_0 = 0.0 # Radians per second.
T_0 = 0.0
POS_0 = npa([0.0, INITIAL_ALTITUDE]) # Meters.
VEL_0 = npa([INITIAL_AIRSPEED, 0.0])
SPEED = la.norm(VEL_0)
VEL_B = npa([INITIAL_AIRSPEED, 0.0])
ALPHA = 0.0 # Radians.
THETA_TO_LOCAL = ct.BODY_TO_RANGE_AND_ALTITUDE(-1.0 * T_0)

DATA = {

	"TOF": [],
	"TDOT_0": [],
	"T_0": [],
	"NORMAL_ACCEL": [],
	"POSITION": [],
	"ALPHA": []

}

while TOF <= MAX_TIME:

	# ATMOSPHERE.
	ATMOS.update(POS_0[1], SPEED)
	RHO = ATMOS.rho # Pascals.
	Q = ATMOS.q # Pascals.
	P = ATMOS.p
	A = ATMOS.a
	G = ATMOS.g
	MACHSPEED = ATMOS.mach
	if MACHSPEED > 1:
		BETA = np.sqrt(MACHSPEED ** 2 - 1) # Non dimensional.
	else:
		BETA = 1 # Non dimensional.

	# MASS AND MOTOR.
	MASS_AND_MOTOR.update(TOF, P)
	XCG = MASS_AND_MOTOR.XCG
	MASS = MASS_AND_MOTOR.MASS
	TMOI = MASS_AND_MOTOR.TRANSVERSE_MOI
	THRUST = MASS_AND_MOTOR.THRUST

	# NORMAL COEFFICIENT AND PITCHING MOMENT COEFFICIENT CALCULATION, NON DIMENSIONAL.
	CN = 2 * ALPHA + (1.5 * PLANFORM_AREA * ALPHA * ALPHA) / REFERENCE_AREA + (8 * WING_AREA * ALPHA) / (BETA * REFERENCE_AREA) + (8 * TAIL_AREA * (ALPHA + FIN_DEFLECTION_RADIANS)) / (BETA * REFERENCE_AREA)
	CM = 2 * ALPHA * ((XCG - NOSE_CENTER_OF_PRESSURE) / REFERENCE_DIAMETER) + ((1.5 * PLANFORM_AREA * ALPHA * ALPHA) / REFERENCE_AREA) * ((XCG - BODY_CENTER_OF_PRESSURE) / REFERENCE_DIAMETER) + ((8 * WING_AREA * ALPHA) / (BETA * REFERENCE_AREA)) * ((XCG - WING_CENTER_OF_PRESSURE) / REFERENCE_DIAMETER) + ((8 * TAIL_AREA * (ALPHA + FIN_DEFLECTION_RADIANS)) / (BETA * REFERENCE_AREA)) * ((XCG - CENTER_OF_DEFLECTION_FROM_NOSE) / REFERENCE_DIAMETER)

	# DERIVATIVES.
	TDOTDOT_0 = -(Q * REFERENCE_AREA * REFERENCE_DIAMETER * CM) / TMOI
	WDOT_0 = (Q * REFERENCE_AREA * CN) / MASS
	UDOT_0 = (THRUST) / MASS
	SPECIFIC_FORCE = npa([0.0, WDOT_0])
	LOCAL_GRAVITY = npa([0.0, -1.0 * G])
	ACC_0 = (SPECIFIC_FORCE @ THETA_TO_LOCAL) + LOCAL_GRAVITY

	# ORIENT.
	THETA_TO_LOCAL = ct.BODY_TO_RANGE_AND_ALTITUDE(-1.0 * T_0)
	SPEED = la.norm(VEL_0)
	VEL_B = THETA_TO_LOCAL @ VEL_0
	ALPHA = returnEl(VEL_B[1], VEL_B[0])

	# EULER INTEGRATION.
	T_0 += TDOT_0 * TIME_STEP
	TDOT_0 += TDOTDOT_0 * TIME_STEP
	POS_0 += VEL_0 * TIME_STEP
	VEL_0 += ACC_0 * TIME_STEP
	TOF += TIME_STEP

	# STORE DATA AT CURRENT CONDITIONS
	DATA["TOF"].append(TOF)
	DATA["TDOT_0"].append(TDOT_0)
	DATA["T_0"].append(T_0)
	DATA["NORMAL_ACCEL"].append(WDOT_0)
	DATA["POSITION"].append(POS_0[1])
	DATA["ALPHA"].append(ALPHA)

# PLOT
DF = pd.DataFrame(DATA)

FIGURE = plt.figure()

THETA_DOT_PLOT = FIGURE.add_subplot(231)
THETA_DOT_PLOT.set_xlabel("TIME OF FLIGHT")
THETA_DOT_PLOT.set_title("TDOT_0 - DEGREES PER SECOND")
THETA_DOT_PLOT.plot(DF.iloc[:]["TOF"], DF.iloc[:]["TDOT_0"] * RAD_TO_DEG, color="b", label="LINEAR AIRFRAME")
THETA_DOT_PLOT.legend(fontsize="small")

THETA_PLOT = FIGURE.add_subplot(232)
THETA_PLOT.set_xlabel("TIME OF FLIGHT")
THETA_PLOT.set_title("T_0 - DEGREES")
THETA_PLOT.plot(DF.iloc[:]["TOF"], DF.iloc[:]["T_0"] * RAD_TO_DEG, color="b", label="LINEAR AIRFRAME")
THETA_PLOT.legend(fontsize="small")

NORMAL_ACCEL_PLOT = FIGURE.add_subplot(233)
NORMAL_ACCEL_PLOT.set_xlabel("TIME OF FLIGHT")
NORMAL_ACCEL_PLOT.set_title("NORMAL ACCEL - (M/S^2)")
NORMAL_ACCEL_PLOT.plot(DF.iloc[:]["TOF"], DF.iloc[:]["NORMAL_ACCEL"], color="b", label="LINEAR AIRFRAME")
NORMAL_ACCEL_PLOT.legend(fontsize="small")

ALTITUDE = FIGURE.add_subplot(234)
ALTITUDE.set_xlabel("TIME OF FLIGHT")
ALTITUDE.set_title("ALTITUDE - METERS")
ALTITUDE.plot(DF.iloc[:]["TOF"], DF.iloc[:]["POSITION"], color="b", label="LINEAR AIRFRAME")
ALTITUDE.legend(fontsize="small")

ALPHA = FIGURE.add_subplot(235)
ALPHA.set_xlabel("TIME OF FLIGHT")
ALPHA.set_title("ALPHA - DEGREES")
ALPHA.plot(DF.iloc[:]["TOF"], DF.iloc[:]["ALPHA"] * RAD_TO_DEG, color="b", label="LINEAR AIRFRAME")
ALPHA.legend(fontsize="small")

plt.get_current_fig_manager().full_screen_toggle()
plt.show()