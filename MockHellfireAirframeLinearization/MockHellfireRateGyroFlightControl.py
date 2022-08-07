# Python libraries.
import numpy as np
from numpy import array as npa
from numpy import linalg as la
import pandas as pd
import matplotlib.pyplot as plt
from ambiance import Atmosphere as atm

# Utility.
from utility.matPlotLibColors import matPlotLibColors as colors
from utility.trapezoidIntegrate import integrate

# Classes.
from classes.Atmosphere import Atmosphere
from classes.MockHellfireMassPropertiesAndMotor import MockHellfireMassPropertiesAndMotor
from classes.MockHellfireAerodynamics import MockHellfireAerodynamics
from classes.MockHellfireDynamicMotionDriver import MockHellfireDynamicMotionDriver
from classes.MockHellFireActuator import SecondOrderActuator

"""

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
EXTENDED_CENTER_OF_DEFLECTION_FROM_NOSE 1.8059 M
EXTENDED_REFERENCE_LENGTH 1.85026 m

"""

# CONSTANTS.
TIME_STEP = 0.001 # Seconds.
MANEUVER_TIME = 1.5 # Seconds.
BURN_TIME = 3.1 # Seconds.
MAX_TIME = 3 # Seconds.
MM_TO_M = 1.0 / 1000.0
RAD_TO_DEG = 57.2957795130823
DEG_TO_RAD = 1.0 / 57.2957795130823
STANDARD_GRAVITY = 9.81 # Meters per second squared.
REFERENCE_DIAMETER = 0.18 # Meters.
REFERENCE_AREA = np.pi * (REFERENCE_DIAMETER ** 2) / 4 # Meters squared.
REFERENCE_LENGTH = 1.85026 # Meters.
INITIAL_AIRSPEED = 130.0 # Meters per second.
INITIAL_ALTITUDE = 1000 # Meters.

# CREATE CLASSES.
AtmosphereObject = Atmosphere()
MassAndMotorProperties = MockHellfireMassPropertiesAndMotor()
AerodynamicsObject = MockHellfireAerodynamics()
INITIAL_POSITION = npa([0.0, INITIAL_ALTITUDE])
INITIAL_VELOCITY = npa([INITIAL_AIRSPEED, 0.0])
MissileMotionObject = MockHellfireDynamicMotionDriver(INITIAL_POSITION=INITIAL_POSITION, INITIAL_VELOCITY=INITIAL_VELOCITY)
ActuatorObject = SecondOrderActuator()

### STATE. ###

### DYNAMICS
DYNAMICS_STATE_STORAGE_DICTIONARY = {
	"COMMANDED_ACCEL": [],
	"RHO": [],
	"PRESSURE": [],
	"SPEED_OF_SOUND": [],
	"GRAVITY": [],
	"DYNAMIC_PRESSURE": [],
	"MACHSPEED": [],
	"THRUST": [],
	"XCG": [],
	"MASS": [],
	"TRANSVERSE_MOI": [],
	"OMEGA_Z": [],
	"OMEGA_AF": [],
	"ZETA_AF": [],
	"KR": [],
	"K1": [],
	"TA": [],
	"K3": [],
	"E": [],
	"EDOT": [],
	"UDOT": [],
	"WDOT": [],
	"U": [],
	"W": [],
	"X": [],
	"Z": [],
	"THETA": [],
	"ALPHA": [],
	"ALPHA_DOT": [],
	"THETA_DOT": [],
	"NORMAL_SPECIFIC_FORCE": [],
	"TOF": []
}

# ATMOSPHERE.
AtmosphereObject.update(altitudeMeters=INITIAL_ALTITUDE, speedMperSec=INITIAL_AIRSPEED)
RHO = AtmosphereObject.rho
PRESSURE = AtmosphereObject.p
SPEED_OF_SOUND = AtmosphereObject.a
GRAVITY = AtmosphereObject.gravity
DYNAMIC_PRESSURE = AtmosphereObject.q
MACHSPEED = AtmosphereObject.mach

# MASS AND MOTOR.
MassAndMotorProperties.update(timeOfFlight=0.0, pressure=PRESSURE)
THRUST = MassAndMotorProperties.THRUST
XCG = MassAndMotorProperties.XCG
MASS = MassAndMotorProperties.MASS
TRANSVERSE_MOMENT_OF_INERTIA = MassAndMotorProperties.TRANSVERSE_MOI # Kilograms times meters squared.

# AERODYNAMICS
AerodynamicsObject.update(
	VELOCITY=npa([INITIAL_AIRSPEED, 0.0]),
	XCG=XCG,
	MACHSPEED=MACHSPEED,
	MASS=MASS,
	COMMAND=0,
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

# MISSILE MOTION
MissileMotionObject.update(
	OMEGA_Z = OMEGA_Z,
	OMEGA_AF=OMEGA_AF,
	ZETA_AF=ZETA_AF,
	KR=KR,
	K1=K1,
	TA=TA,
	K3=K3,
	DEFLECTION=0.0,
	GRAVITY=GRAVITY,
	MAX_TIME=0.0
)
E = MissileMotionObject.E
EDOT = MissileMotionObject.EDOT
ACCELERATION = MissileMotionObject.ACCELERATION # Meters per second squared.
POSITION = MissileMotionObject.POSITION # Meters.
VELOCITY = MissileMotionObject.VELOCITY # Meters per second.
THETA = MissileMotionObject.THETA # Radians.
ALPHA = MissileMotionObject.ALPHA # Radians.
ALPHA_DOT = MissileMotionObject.ALPHA_DOT # Radians per second.
THETA_DOT = MissileMotionObject.THETA_DOT # Radians per second.
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

### ACTUATOR ###
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

### TARGET STATES ###
# TARGET.
# SEEKER.

### GUIDANCE AND CONTROL ###
# GUIDANCE.
FIRST_ACCEL_COMMAND_IN_GS = 2 # Gs.
SECOND_ACCEL_COMMAND_IN_GS = -2 # Gs
COMMAND = FIRST_ACCEL_COMMAND_IN_GS #Gs.
# CONTROL.
DEFLECTION_COMMAND = 0.0 # Degrees.

storeDynamicsState()

GO = True
while TOF <= MAX_TIME:

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

	# AERODYNAMICS - ADD DRAG IMPLEMENTATION.
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
		MAX_TIME=TOF + TIME_STEP
	)
	E = MissileMotionObject.E
	EDOT = MissileMotionObject.EDOT
	ACCELERATION = MissileMotionObject.ACCELERATION # Meters per second squared.
	POSITION = MissileMotionObject.POSITION # Meters.
	VELOCITY = MissileMotionObject.VELOCITY # Meters per second.
	THETA = MissileMotionObject.THETA # Radians.
	ALPHA = MissileMotionObject.ALPHA # Radians.
	ALPHA_DOT = MissileMotionObject.ALPHA_DOT # Radians per second.
	THETA_DOT = MissileMotionObject.THETA_DOT # Radians per second.
	NORMAL_SPECIFIC_FORCE = MissileMotionObject.NORMAL_SPECIFIC_FORCE # Meters per second squared.
	TOF = MissileMotionObject.TOF # Seconds.

	### TARGET STATES ###
	# TARGET.
	# SEEKER.

	### ACTUATOR ###
	ActuatorObject.update(DEFLECTION_COMMAND=DEFLECTION_COMMAND, MAX_TIME = TOF + TIME_STEP)
	DEFLECTION = ActuatorObject.DEFLECTION

	### GUIDANCE AND CONTROL ###
	# GUIDANCE.
	COMMAND = 0.0
	if TOF < MANEUVER_TIME:
		COMMAND = FIRST_ACCEL_COMMAND_IN_GS
	else:
		COMMAND = SECOND_ACCEL_COMMAND_IN_GS

	# CONTROL.
	KDC = (1 - KR * K3) / (K1 * KR)
	DEFLECTION_COMMAND = KR * (KDC * COMMAND + THETA_DOT)

	### OVERHEAD ###
	# PERFORMANCE AND TERMINATION CHECK.
	# DATA LOG.

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
DATA_ACT = pd.DataFrame(ACTUATOR_STATE_STORAGE_DICTIONARY)
FIGURE = plt.figure()
START_INDEX = 0
STOP_INDEX = -1
COLOR_LIST = colors()
INDEX = 0

PLOT1 = FIGURE.add_subplot(221)
PLOT1.plot(DATA_DYN.iloc[START_INDEX:STOP_INDEX]["TOF"], DATA_DYN.iloc[START_INDEX:STOP_INDEX]["Z"], label="ALTITUDE METERS", color=COLOR_LIST[INDEX])
INDEX += 1
PLOT1.set_xlabel("TIME OF FLIGHT")
PLOT1.set_title("ALTITUDE")
PLOT1.legend(fontsize="small")

PLOT2 = FIGURE.add_subplot(222)
PLOT2.plot(DATA_DYN.iloc[START_INDEX:STOP_INDEX]["TOF"], DATA_DYN.iloc[START_INDEX:STOP_INDEX]["THETA"] * DEG_TO_RAD, label="THETA RADIANS", color=COLOR_LIST[INDEX])
INDEX += 1
PLOT2.plot(DATA_DYN.iloc[START_INDEX:STOP_INDEX]["TOF"], DATA_DYN.iloc[START_INDEX:STOP_INDEX]["THETA_DOT"] * DEG_TO_RAD, label="THETA DOT RADS PER SEC", color=COLOR_LIST[INDEX])
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
PLOT3.plot(DATA_ACT.iloc[START_INDEX:STOP_INDEX]["TOF"], DATA_ACT.iloc[START_INDEX:STOP_INDEX]["DEFLECTION_COMMAND"], label="DEFLECTION COMMAND", color=COLOR_LIST[INDEX])
INDEX += 1
PLOT3.set_xlabel("TIME OF FLIGHT")
PLOT3.set_title("ACTUATOR")
PLOT3.legend(fontsize="small")

PLOT4 = FIGURE.add_subplot(224)
PLOT4.plot(DATA_DYN.iloc[START_INDEX:STOP_INDEX]["TOF"], DATA_DYN.iloc[START_INDEX:STOP_INDEX]["COMMANDED_ACCEL"], label="ACCELERATION COMMAND", color=COLOR_LIST[INDEX])
INDEX += 1
PLOT4.plot(DATA_DYN.iloc[START_INDEX:STOP_INDEX]["TOF"], DATA_DYN.iloc[START_INDEX:STOP_INDEX]["NORMAL_SPECIFIC_FORCE"], label="NORMAL SPECIFIC FORCE", color=COLOR_LIST[INDEX])
INDEX += 1
PLOT4.plot(DATA_DYN.iloc[START_INDEX:STOP_INDEX]["TOF"], DATA_DYN.iloc[START_INDEX:STOP_INDEX]["W"], label="NORMAL VELOCITY", color=COLOR_LIST[INDEX])
INDEX += 1
PLOT4.set_xlabel("TIME OF FLIGHT")
PLOT4.set_title("MANEUVERABILITY")
PLOT4.legend(fontsize="small")

FIGURE.subplots_adjust(top=0.9, bottom=0.1, left=0.1, hspace=0.4)
plt.get_current_fig_manager().full_screen_toggle()
plt.show()