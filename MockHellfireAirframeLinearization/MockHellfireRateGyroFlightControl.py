"""

TO DO:

1) GO THROUGH THE ENTIRE CODE AND CHECK UNITS. ***VERY IMPORTANT***
	A) MOVE AERODYNAMICS FROM CLASS TO AIRFRAME SIMULATION TO COMPARE WITH THE FIRST TWO.
	B) THEN PUT BACK IN CLASS.
2) ANY MODULE THAT IS SET UP TO HANDLE 2D OR 3D, CHANGE TO ONLY 2D FOR UNIFORMITY.
3) FOR MISSILE MOTION, FIGURE TRANSFORMATION MATRICES.
4) ADD THRUST TO MOTION MODULE.
5) ADD DRAG PROFILE TO AERODYNAMICS.
6) EACH MODULE NEEDS A "NEXT UPDATE TIME."
7) VOTING SYSTEM FOR DYNAMICS UPDATE.
8) CONTROL MODULE.

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
FINAL_CG_FROM_NOSE 0.249733 M
EXTENDED_CENTER_OF_DEFLECTION_FROM_NOSE 1.8059 M
EXTENDED_REFERENCE_LENGTH 1.85026 m

"""

# Python libraries.
import numpy as np
from numpy import array as npa
from numpy import linalg as la
import pandas as pd
import matplotlib.pyplot as plt
from ambiance import Atmosphere as atm
np.printoptions(suppress=True, precision=4)

# Utility.
from utility.matPlotLibColors import matPlotLibColors as colors
from utility.trapezoidIntegrate import integrate

# Classes.
from classes.Atmosphere import Atmosphere
from classes.MockHellfireMassPropertiesAndMotor import MockHellfireMassPropertiesAndMotor
from classes.MockHellfireAerodynamics import MockHellfireAerodynamics
from classes.MockHellfireDynamicMotionDriver import MockHellfireDynamicMotionDriver
from classes.SecondOrderActuator import SecondOrderActuator
from classes.Target import Target
from classes.KinematicTruthSeeker import KinematicTruthSeeker
from classes.ProportionalGuidance import ProportionalGuidance

# CONSTANTS.
SIM_TIME_STEP = 0.001 # Seconds.
MANEUVER_TIME = 1.5 # Seconds.
BURN_TIME = 3.1 # Seconds.
MAX_TIME = 10 # Seconds.
MM_TO_M = 1.0 / 1000.0
RAD_TO_DEG = 57.2957795130823
DEG_TO_RAD = 1.0 / 57.2957795130823
STANDARD_GRAVITY = 9.81 # Meters per second squared.
REFERENCE_DIAMETER = 0.18 # Meters.
REFERENCE_AREA = np.pi * (REFERENCE_DIAMETER ** 2) / 4 # Meters squared.
REFERENCE_LENGTH = 1.85026 # Meters.
INITIAL_HORIZONTAL_AIRSPEED = 100.0 # Meters per second.
INITIAL_VERTICAL_AIRSPEED = 10.0 # Meters per second.
INITIAL_ALTITUDE = 400 # Meters.
INITIAL_TARGET_POSITION = npa([400.0, INITIAL_ALTITUDE + 200.0]) # Meters.
INITIAL_TARGET_VELOCITY = npa([0.0, 0.0]) # Meters per second.

# CREATE CLASSES.
AtmosphereObject = Atmosphere()
MassAndMotorProperties = MockHellfireMassPropertiesAndMotor()
AerodynamicsObject = MockHellfireAerodynamics()
INITIAL_POSITION = npa([0.0, INITIAL_ALTITUDE])
INITIAL_VELOCITY = npa([INITIAL_HORIZONTAL_AIRSPEED, INITIAL_VERTICAL_AIRSPEED])
MissileMotionObject = MockHellfireDynamicMotionDriver(INITIAL_POSITION=INITIAL_POSITION, INITIAL_VELOCITY=INITIAL_VELOCITY)
ActuatorObject = SecondOrderActuator()
TargetObject = Target(
	INITIAL_TARGET_POSITION=INITIAL_TARGET_POSITION,
	INITIAL_TARGET_VELOCITY=INITIAL_TARGET_VELOCITY,
	TIME_STEP=SIM_TIME_STEP
)
SeekerObject = KinematicTruthSeeker()
GuidanceLaw = ProportionalGuidance()

### STATE. ###
### DYNAMICS
DYNAMICS_STATE_STORAGE_DICTIONARY = {
	"COMMANDED_ACCEL": [], # Gs
	"RHO": [], # Kilograms per meter cubed.
	"PRESSURE": [], # Pascals.
	"SPEED_OF_SOUND": [], # Meters per second.
	"GRAVITY": [], # Meters per second squared.
	"DYNAMIC_PRESSURE": [], # Pascals.
	"MACHSPEED": [], # Non dimensional
	"THRUST": [], # Newtons.
	"XCG": [], # Meters from nose.
	"MASS": [], # Kilograms.
	"TRANSVERSE_MOI": [], # Kilograms times meters squared.
	"OMEGA_Z": [],
	"OMEGA_AF": [],
	"ZETA_AF": [],
	"KR": [],
	"K1": [],
	"TA": [],
	"K3": [],
	"E": [],
	"EDOT": [],
	"UDOT": [], # Meters per second squared.
	"WDOT": [], # Meters per second squared.
	"U": [], # Meters per second.
	"W": [], # Meters per second.
	"X": [], # Meters.
	"Z": [], # Meters.
	"THETA": [], # Radians.
	"ALPHA": [], # Radians.
	"ALPHA_DOT": [], # Radians per second.
	"THETA_DOT": [], # Radians per second.
	"NORMAL_SPECIFIC_FORCE": [], # Meters per second squared.
	"TOF": [] # Seconds.
}

# ATMOSPHERE.
AtmosphereObject.update(altitudeMeters=INITIAL_ALTITUDE, speedMperSec=INITIAL_HORIZONTAL_AIRSPEED)
RHO = AtmosphereObject.rho # Kilograms per meter cubed.
PRESSURE = AtmosphereObject.p # Pascals.
SPEED_OF_SOUND = AtmosphereObject.a # Meters per second.
GRAVITY = AtmosphereObject.gravity # Meters per second squared.
DYNAMIC_PRESSURE = AtmosphereObject.q # Pascals.
MACHSPEED = AtmosphereObject.mach # Non dimensional.

# MASS AND MOTOR.
MassAndMotorProperties.update(timeOfFlight=0.0, pressure=PRESSURE)
THRUST = MassAndMotorProperties.THRUST # Newtons.
XCG = MassAndMotorProperties.XCG # Meters from nose.
MASS = MassAndMotorProperties.MASS # Kilograms.
TRANSVERSE_MOMENT_OF_INERTIA = MassAndMotorProperties.TRANSVERSE_MOI # Kilograms times meters squared.

# AERODYNAMICS
AerodynamicsObject.update(
	VELOCITY=npa([INITIAL_HORIZONTAL_AIRSPEED, 0.0]), # Meters per second.
	XCG=XCG, # Meters from nose.
	MACHSPEED=MACHSPEED, # Non dimensional.
	MASS=MASS, # Kilograms.
	COMMAND=0, # Gs.
	DYNAMIC_PRESSURE=DYNAMIC_PRESSURE, # Pascals.
	TRANSVERSE_MOMENT_OF_INERTIA=TRANSVERSE_MOMENT_OF_INERTIA # Kilograms times meters squared.
)
OMEGA_Z = AerodynamicsObject.OMEGA_Z
OMEGA_AF = AerodynamicsObject.OMEGA_AF
ZETA_AF = AerodynamicsObject.ZETA_AF
KR = AerodynamicsObject.KR
K1 = AerodynamicsObject.K1
TA = AerodynamicsObject.TA
K3 = AerodynamicsObject.K3

# MISSILE MOTION
MissileMotionObject.update(
	OMEGA_Z = OMEGA_Z,
	OMEGA_AF=OMEGA_AF,
	ZETA_AF=ZETA_AF,
	KR=KR,
	K1=K1,
	TA=TA,
	K3=K3,
	DEFLECTION=0.0, # Degrees.
	GRAVITY=GRAVITY, # Meters per second.
	MAX_TIME=0.0 # Seconds.
)
E = MissileMotionObject.E
EDOT = MissileMotionObject.EDOT
ACCELERATION = MissileMotionObject.ACCELERATION # Meters per second squared.
POSITION = MissileMotionObject.POSITION # Meters.
VELOCITY = MissileMotionObject.VELOCITY # Meters per second.
THETA = MissileMotionObject.THETA * DEG_TO_RAD # Radians.
ALPHA = MissileMotionObject.ALPHA # Radians.
ALPHA_DOT = MissileMotionObject.ALPHA_DOT # Radians per second.
THETA_DOT = MissileMotionObject.THETA_DOT * DEG_TO_RAD # Radians per second.
NORMAL_SPECIFIC_FORCE = MissileMotionObject.NORMAL_SPECIFIC_FORCE # Meters per second squared.
TOF = MissileMotionObject.TOF # Seconds.

DYNAMICS_STATE_CURRENT_DICTIONARY = {
	"COMMANDED_ACCEL": 0.0,
	"RHO": RHO,
	"PRESSURE": PRESSURE,
	"SPEED_OF_SOUND": SPEED_OF_SOUND,
	"GRAVITY": GRAVITY,
	"DYNAMIC_PRESSURE": DYNAMIC_PRESSURE,
	"MACHSPEED": MACHSPEED,
	"THRUST": THRUST,
	"XCG": XCG,
	"MASS": MASS,
	"TRANSVERSE_MOI": TRANSVERSE_MOMENT_OF_INERTIA,
	"OMEGA_Z": OMEGA_Z,
	"OMEGA_AF": OMEGA_AF,
	"ZETA_AF": ZETA_AF,
	"KR": KR,
	"K1": K1,
	"TA": TA,
	"K3": K3,
	"E": E,
	"EDOT": EDOT,
	"UDOT": ACCELERATION[0],
	"WDOT": ACCELERATION[1],
	"U": VELOCITY[0],
	"W": VELOCITY[1],
	"X": POSITION[0],
	"Z": POSITION[1],
	"THETA": THETA,
	"ALPHA": ALPHA,
	"ALPHA_DOT": ALPHA_DOT,
	"THETA_DOT": THETA_DOT,
	"NORMAL_SPECIFIC_FORCE": NORMAL_SPECIFIC_FORCE,
	"TOF": TOF
}

def storeDynamicsState():
	for index, key in enumerate(DYNAMICS_STATE_STORAGE_DICTIONARY.keys()):
		DYNAMICS_STATE_STORAGE_DICTIONARY[f"{key}"].append(DYNAMICS_STATE_CURRENT_DICTIONARY[f"{key}"])

### TARGET STATE
TARGET_STATE_STORAGE_DICTIONARY = {
	"TGT_TOF": [], # Seconds.
	"TGT_X": [], # Meters.
	"TGT_Z": [], # Meters.
	"TGT_VX": [], # Meters per second.
	"TGT_VZ": [], # Meters per second.
	"CLOSING_SPEED": [], # Meters per second.
	"MSL_TGT_RELPOS": [], # Meters.
	"LINE_OF_SIGHT_RATE": []# Per second.
}
# TARGET.
TargetObject.update()
TGT_TOF = TargetObject.targetTimeOfFlight # Seconds.
TGT_POS = TargetObject.targetRightUpPosition # Meters.
TGT_VEL = TargetObject.targetRightUpVelocity # Meters per second.
# SEEKER.
SeekerObject.update(
	THETA_RADS=THETA,
	PSI_RADS=0.0,
	MSL_POS=POSITION,
	MSL_VEL=VELOCITY,
	TGT_POS=TGT_POS,
	TGT_VEL=TGT_VEL,
	DIMENSION_FLAG=2
)
CLOSING_SPEED = SeekerObject.CLOSING_SPEED
MSL_TO_TGT_RELPOS = SeekerObject.MSL_TO_TGT_RELPOS
LINE_OF_SIGHT_RATE = SeekerObject.LINE_OF_SIGHT_RATE
TARGET_STATE_CURRENT_DICTIONARY = {
	"TGT_TOF": TGT_TOF,
	"TGT_X": TGT_POS[0],
	"TGT_Z": TGT_POS[1],
	"TGT_VX": TGT_VEL[0],
	"TGT_VZ": TGT_VEL[1],
	"CLOSING_SPEED": CLOSING_SPEED,
	"MSL_TGT_RELPOS": MSL_TO_TGT_RELPOS,
	"LINE_OF_SIGHT_RATE": LINE_OF_SIGHT_RATE
}

def storeTargetState():
	for index, key in enumerate(TARGET_STATE_STORAGE_DICTIONARY.keys()):
		TARGET_STATE_STORAGE_DICTIONARY[f"{key}"].append(TARGET_STATE_CURRENT_DICTIONARY[f"{key}"])

### ACTUATOR
ACTUATOR_STATE_STORAGE_DICTIONARY = {
	"TOF": [],
	"DEFLECTION_COMMAND": [],
	"DEFLECTION": []
}
ActuatorObject.update(0.0, 0.0)
DEFLECTION = ActuatorObject.DEFLECTION # Degrees.
ACTUATOR_STATE_CURRENT_DICTIONARY = {
	"TOF": TOF,
	"DEFLECTION_COMMAND": 0.0,
	"DEFLECTION": ActuatorObject.DEFLECTION
}

def storeActuatorState():
	for index, key in enumerate(ACTUATOR_STATE_STORAGE_DICTIONARY.keys()):
		ACTUATOR_STATE_STORAGE_DICTIONARY[f"{key}"].append(ACTUATOR_STATE_CURRENT_DICTIONARY[f"{key}"])

### GUIDANCE AND CONTRO
# GUIDANCE.
GuidanceLaw.update(
	CLOSING_SPEED=CLOSING_SPEED,
	MSL_TO_TARGET_RELPOS=MSL_TO_TGT_RELPOS,
	LINE_OF_SIGHT_RATE=LINE_OF_SIGHT_RATE
)
# FIRST_ACCEL_COMMAND_IN_GS = 2 # Gs.
# SECOND_ACCEL_COMMAND_IN_GS = -2 # Gs
COMMAND = 0.0 #Gs.
# CONTROL.
DEFLECTION_COMMAND = 0.0 # Degrees.

storeDynamicsState()
storeTargetState()
storeActuatorState()

GO = True
while GO:

	### DYNAMICS. ###
	# ATMOSPHERE.
	AtmosphereObject.update(altitudeMeters=POSITION[1], speedMperSec=la.norm(VELOCITY))
	RHO = AtmosphereObject.rho
	PRESSURE = AtmosphereObject.p
	SPEED_OF_SOUND = AtmosphereObject.a
	GRAVITY = AtmosphereObject.gravity
	DYNAMIC_PRESSURE = AtmosphereObject.q
	MACHSPEED = AtmosphereObject.mach

	# MASS AND MOTOR.
	MassAndMotorProperties.update(timeOfFlight=TOF, pressure=PRESSURE)
	THRUST = MassAndMotorProperties.THRUST
	XCG = MassAndMotorProperties.XCG
	MASS = MassAndMotorProperties.MASS
	TRANSVERSE_MOMENT_OF_INERTIA = MassAndMotorProperties.TRANSVERSE_MOI # Kilograms times meters squared.

	# AERODYNAMICS - TO DO - ADD DRAG PROFILE FOR ROUND NOSE.
	AerodynamicsObject.update(
		VELOCITY=VELOCITY,
		XCG=XCG,
		MACHSPEED=MACHSPEED,
		MASS=MASS,
		COMMAND=COMMAND,
		DYNAMIC_PRESSURE=DYNAMIC_PRESSURE,
		TRANSVERSE_MOMENT_OF_INERTIA=TRANSVERSE_MOMENT_OF_INERTIA
	)
	OMEGA_Z = AerodynamicsObject.OMEGA_Z
	OMEGA_AF = AerodynamicsObject.OMEGA_AF
	ZETA_AF = AerodynamicsObject.ZETA_AF
	KR = AerodynamicsObject.KR
	K1 = AerodynamicsObject.K1
	TA = AerodynamicsObject.TA
	K3 = AerodynamicsObject.K3

	# MISSILE MOTION.
	# DIFFERENTIAL EQUATIONS TO SOLVE FOR THE DERIVATIVES IN THE PITCH.
	MissileMotionObject.update(
		OMEGA_Z = OMEGA_Z,
		OMEGA_AF=OMEGA_AF,
		ZETA_AF=ZETA_AF,
		KR=KR,
		K1=K1,
		TA=TA,
		K3=K3,
		DEFLECTION=DEFLECTION,
		GRAVITY=GRAVITY,
		MAX_TIME=TOF + SIM_TIME_STEP
	)
	E = MissileMotionObject.E
	EDOT = MissileMotionObject.EDOT
	ACCELERATION = MissileMotionObject.ACCELERATION # Meters per second squared.
	POSITION = MissileMotionObject.POSITION # Meters.
	VELOCITY = MissileMotionObject.VELOCITY # Meters per second.
	THETA = MissileMotionObject.THETA * DEG_TO_RAD # Radians.
	ALPHA = MissileMotionObject.ALPHA # Radians.
	ALPHA_DOT = MissileMotionObject.ALPHA_DOT # Radians per second.
	THETA_DOT = MissileMotionObject.THETA_DOT * DEG_TO_RAD # Radians per second.
	NORMAL_SPECIFIC_FORCE = MissileMotionObject.NORMAL_SPECIFIC_FORCE # Meters per second squared.
	TOF = MissileMotionObject.TOF # Seconds.

	### TARGET STATES ###
	# TARGET.
	TargetObject.update()
	TGT_TOF = TargetObject.targetTimeOfFlight
	TGT_POS = TargetObject.targetRightUpPosition
	TGT_VEL = TargetObject.targetRightUpVelocity
	# SEEKER.
	SeekerObject.update(
		THETA_RADS=THETA,
		PSI_RADS=0.0,
		MSL_POS=POSITION,
		MSL_VEL=VELOCITY,
		TGT_POS=TGT_POS,
		TGT_VEL=TGT_VEL,
		DIMENSION_FLAG=2
	)
	CLOSING_SPEED = SeekerObject.CLOSING_SPEED
	MSL_TO_TGT_RELPOS = SeekerObject.MSL_TO_TGT_RELPOS
	LINE_OF_SIGHT_RATE = SeekerObject.LINE_OF_SIGHT_RATE

	### ACTUATOR ###
	ActuatorObject.update(DEFLECTION_COMMAND=DEFLECTION_COMMAND, MAX_TIME = TOF + SIM_TIME_STEP)
	DEFLECTION = ActuatorObject.DEFLECTION

	### GUIDANCE AND CONTROL ###
	# GUIDANCE.
	GuidanceLaw.update(
		CLOSING_SPEED=CLOSING_SPEED,
		MSL_TO_TARGET_RELPOS=MSL_TO_TGT_RELPOS,
		LINE_OF_SIGHT_RATE=LINE_OF_SIGHT_RATE
	)
	COMMAND = GuidanceLaw.COMMAND / GRAVITY
	# COMMAND = 0.0
	# if TOF < MANEUVER_TIME:
	# 	COMMAND = FIRST_ACCEL_COMMAND_IN_GS
	# else:
	# 	COMMAND = SECOND_ACCEL_COMMAND_IN_GS

	# CONTROL.
	KDC = (1 - KR * K3) / (K1 * KR)
	DEFLECTION_COMMAND = KR * (KDC * COMMAND + THETA_DOT)

	### OVERHEAD ###
	# PERFORMANCE AND TERMINATION CHECK.
	MISS_DISTANCE = la.norm(MSL_TO_TGT_RELPOS)
	if TOF > MAX_TIME:
		print("MAX TIME EXCEEDED")
		GO = False
	if np.isnan(np.sum(POSITION)):
		print("NOT A NUMBER")
		GO = False
	if POSITION[1] < 0:
		print("GROUND COLLISION")
		GO = False
	if MISS_DISTANCE < 5.0:
		print(f"SUCCESSFUL INTERCEPT {MISS_DISTANCE} {MSL_TO_TGT_RELPOS}")
		GO = False
	if MSL_TO_TGT_RELPOS[0] < 0:
		print(f"POINT OF CLOSEST APPROACH PASSED {MISS_DISTANCE} {MSL_TO_TGT_RELPOS}")
		GO = False

	# CONSOLE REPORT.
	if round(TOF, 3).is_integer():
		print(f"{np.ceil(TOF)} {POSITION} {VELOCITY}")

	# STORE DATA AT CURRENT CONDITIONS
	DYNAMICS_STATE_CURRENT_DICTIONARY = {
		"COMMANDED_ACCEL": COMMAND,
		"RHO": RHO,
		"PRESSURE": PRESSURE,
		"SPEED_OF_SOUND": SPEED_OF_SOUND,
		"GRAVITY": GRAVITY,
		"DYNAMIC_PRESSURE": DYNAMIC_PRESSURE,
		"MACHSPEED": MACHSPEED,
		"THRUST": THRUST,
		"XCG": XCG,
		"MASS": MASS,
		"TRANSVERSE_MOI": TRANSVERSE_MOMENT_OF_INERTIA,
		"OMEGA_Z": OMEGA_Z,
		"OMEGA_AF": OMEGA_AF,
		"ZETA_AF": ZETA_AF,
		"KR": KR,
		"K1": K1,
		"TA": TA,
		"K3": K3,
		"E": E,
		"EDOT": EDOT,
		"UDOT": ACCELERATION[0],
		"WDOT": ACCELERATION[1], # Meters per second^2.
		"U": VELOCITY[0],
		"W": VELOCITY[1],
		"X": POSITION[0],
		"Z": POSITION[1],
		"THETA": THETA,
		"ALPHA": ALPHA,
		"ALPHA_DOT": ALPHA_DOT,
		"THETA_DOT": THETA_DOT,
		"NORMAL_SPECIFIC_FORCE": NORMAL_SPECIFIC_FORCE, # Gs
		"TOF": TOF
	}
	storeDynamicsState()

	TARGET_STATE_CURRENT_DICTIONARY = {
		"TGT_TOF": TGT_TOF,
		"TGT_X": TGT_POS[0],
		"TGT_Z": TGT_POS[1],
		"TGT_VX": TGT_VEL[0],
		"TGT_VZ": TGT_VEL[1],
		"CLOSING_SPEED": CLOSING_SPEED,
		"MSL_TGT_RELPOS": MSL_TO_TGT_RELPOS,
		"LINE_OF_SIGHT_RATE": LINE_OF_SIGHT_RATE
	}
	storeTargetState()

	ACTUATOR_STATE_CURRENT_DICTIONARY = {
		"TOF": TOF,
		"DEFLECTION_COMMAND": DEFLECTION_COMMAND,
		"DEFLECTION": DEFLECTION
	}
	storeActuatorState()

# OUTPUT
print(f"DIFFERENTIAL EQUATIONS OF MOTION FINISHED.")

# PLOT
DATA_DYN= pd.DataFrame(DYNAMICS_STATE_STORAGE_DICTIONARY)
DATA_TGT = pd.DataFrame(TARGET_STATE_STORAGE_DICTIONARY)
DATA_ACT = pd.DataFrame(ACTUATOR_STATE_STORAGE_DICTIONARY)
FIGURE = plt.figure()
START_INDEX = 0
STOP_INDEX = -1
COLOR_LIST = colors()
INDEX = 0

scale = True
if scale:
	xMin = min(
		list(DATA_DYN[START_INDEX:STOP_INDEX]["X"]) + \
		list(DATA_TGT.iloc[START_INDEX:STOP_INDEX]["TGT_X"])
	)
	xMax = max(
		list(DATA_DYN[START_INDEX:STOP_INDEX]["X"]) + \
		list(DATA_TGT.iloc[START_INDEX:STOP_INDEX]["TGT_X"])
	)
	zMin = min(
		list(DATA_DYN[START_INDEX:STOP_INDEX]["Z"]) + \
		list(DATA_TGT.iloc[START_INDEX:STOP_INDEX]["TGT_Z"])
	)
	zMax = max(
		list(DATA_DYN[START_INDEX:STOP_INDEX]["Z"]) + \
		list(DATA_TGT.iloc[START_INDEX:STOP_INDEX]["TGT_Z"])
	)
	BUFFER = 100
	# PLOT1.set_box_aspect(
	# 	(
	# 		np.ptp([xMin - BUFFER, xMax + BUFFER]),
	# 		np.ptp([zMin - BUFFER, zMax + BUFFER])
	# 	)
	# )
	# PLOT1.set_aspect("float")

	XLIM_RANGE = (xMax + BUFFER) - (xMin - BUFFER)
	ZLIM_RANGE = (zMax + BUFFER) - (zMin - BUFFER)
	RATIO = ZLIM_RANGE / XLIM_RANGE
	

PLOT1 = FIGURE.add_subplot(221)
if scale:
	PLOT1.set_xlim([xMin - BUFFER, xMax + BUFFER])
	PLOT1.set_ylim([zMin - BUFFER, zMax + BUFFER])
	PLOT1.set_aspect(RATIO)
	# PLOT1.set_figheight(RATIO)
	# PLOT1.set_figwidth(1 / RATIO)
PLOT1.plot(DATA_DYN.iloc[START_INDEX:STOP_INDEX]["X"], DATA_DYN.iloc[START_INDEX:STOP_INDEX]["Z"], label="MSL POS", color=COLOR_LIST[INDEX])
INDEX += 1
if DATA_TGT.iloc[STOP_INDEX]["TGT_X"] == DATA_TGT.iloc[START_INDEX]["TGT_X"]:
	PLOT1.scatter(DATA_TGT.iloc[STOP_INDEX]["TGT_X"], DATA_TGT.iloc[STOP_INDEX]["TGT_Z"], label="TGT POS", color=COLOR_LIST[INDEX])
else:
	PLOT1.plot(DATA_TGT.iloc[START_INDEX:STOP_INDEX]["TGT_X"],DATA_TGT[START_INDEX:STOP_INDEX]["TGT_Z"], label="TGT POS", color=COLOR_LIST[INDEX])
INDEX += 1
PLOT1.set_xlabel("EAST")
PLOT1.set_ylabel("UP")
PLOT1.set_title("POSITION")
PLOT1.legend(fontsize="xx-small")

PLOT2 = FIGURE.add_subplot(222)
PLOT2.plot(DATA_DYN.iloc[START_INDEX:STOP_INDEX]["TOF"], DATA_DYN.iloc[START_INDEX:STOP_INDEX]["THETA"], label="THETA RADIANS", color=COLOR_LIST[INDEX])
INDEX += 1
PLOT2.plot(DATA_DYN.iloc[START_INDEX:STOP_INDEX]["TOF"], DATA_DYN.iloc[START_INDEX:STOP_INDEX]["THETA_DOT"], label="THETA DOT RADS PER SEC", color=COLOR_LIST[INDEX])
INDEX += 1
PLOT2.plot(DATA_DYN.iloc[START_INDEX:STOP_INDEX]["TOF"], DATA_DYN.iloc[START_INDEX:STOP_INDEX]["ALPHA"], label="ALPHA RADIANS", color=COLOR_LIST[INDEX])
INDEX += 1
PLOT2.plot(DATA_DYN.iloc[START_INDEX:STOP_INDEX]["TOF"], DATA_DYN.iloc[START_INDEX:STOP_INDEX]["ALPHA_DOT"], label="ALPHA DOT RADS PER SEC", color=COLOR_LIST[INDEX])
INDEX += 1
PLOT2.set_xlabel("TIME OF FLIGHT")
PLOT2.set_title("PITCH")
PLOT2.legend(fontsize="small")

PLOT3 = FIGURE.add_subplot(223)
PLOT3.plot(DATA_ACT.iloc[START_INDEX:STOP_INDEX]["TOF"], DATA_ACT.iloc[START_INDEX:STOP_INDEX]["DEFLECTION"], label="DEFLECTION", color=COLOR_LIST[INDEX])
INDEX += 1
PLOT3.plot(DATA_ACT.iloc[START_INDEX:STOP_INDEX]["TOF"], DATA_ACT.iloc[START_INDEX:STOP_INDEX]["DEFLECTION_COMMAND"], label="DEFLECTION COMMAND", color=COLOR_LIST[INDEX], alpha=0.5)
INDEX += 1
PLOT3.set_xlabel("TIME OF FLIGHT")
PLOT3.set_title("ACTUATOR")
PLOT3.legend(fontsize="small")

PLOT4 = FIGURE.add_subplot(224)
PLOT4.plot(DATA_DYN.iloc[START_INDEX:STOP_INDEX]["TOF"], DATA_DYN.iloc[START_INDEX:STOP_INDEX]["COMMANDED_ACCEL"], label="ACCELERATION COMMAND", color=COLOR_LIST[INDEX])
INDEX += 1
PLOT4.plot(DATA_DYN.iloc[START_INDEX:STOP_INDEX]["TOF"], DATA_DYN.iloc[START_INDEX:STOP_INDEX]["NORMAL_SPECIFIC_FORCE"], label="NORMAL SPECIFIC FORCE", color=COLOR_LIST[INDEX], alpha=0.5)
INDEX += 1
PLOT4.plot(DATA_DYN.iloc[START_INDEX:STOP_INDEX]["TOF"], DATA_DYN.iloc[START_INDEX:STOP_INDEX]["W"], label="NORMAL VELOCITY", color=COLOR_LIST[INDEX])
INDEX += 1
PLOT4.set_xlabel("TIME OF FLIGHT")
PLOT4.set_title("MANEUVERABILITY")
PLOT4.legend(fontsize="small")

FIGURE.subplots_adjust(top=0.9, bottom=0.1, left=0.1, hspace=0.4)
plt.get_current_fig_manager().full_screen_toggle()
plt.show()
