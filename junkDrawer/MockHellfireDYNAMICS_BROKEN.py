"""


TO DO:

1) GO THROUGH THE ENTIRE CODE AND CHECK UNITS. ***VERY IMPORTANT***
	A) MAKE SURE ALL STATE PARAMETERS ARE BEING POPULATED EACH LOOP. ADD GAMMA DOT.
2) CREATE AN ANIMATION TO WATCH THE ORIENTATION OF THE MISSILE AS IT FLYS.
3) ADD DRAG PROFILE TO AERODYNAMICS.
4) VOTING SYSTEM FOR DYNAMICS UPDATE.
5) BASE CLASS - EACH MODULE NEEDS A "NEXT UPDATE TIME."
6) TARGET AND SEEKER MODELS.
7) GUIDANCE AND CONTROL MODULES.
8) ADD AN I.N.S.


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
STARTING_CG_FROM_NOSE 0.644605 M
FINAL_CG_FROM_NOSE 0.249733 M
EXTENDED_CENTER_OF_DEFLECTION_FROM_NOSE 1.8059 M
EXTENDED_REFERENCE_LENGTH 1.85026 M


"""

# MATH CONSTANTS
MM_TO_M = 1.0 / 1000.0
RAD_TO_DEG = 57.2957795130823
DEG_TO_RAD = 1.0 / RAD_TO_DEG

# Python libraries.
import numpy as np
from numpy import array as npa
from numpy import linalg as la
import pandas as pd
import matplotlib.pyplot as plt
from ambiance import Atmosphere as atm
np.set_printoptions(suppress=True, precision=2)

# Utility.
from utility.returnAzAndElevation import returnEl
from utility import coordinateTransformations as ct

# Classes.
from classes.Atmosphere import Atmosphere
from classes.MockHellfireMassAndMotor import MockHellfireMassAndMotor

class AirframeSimulation:

	def __init__(
		self,
		FLAG,
		FIN_DEFLECTION_DEGREES,
		MAX_TIME, TIME_STEP,
		INITIAL_MISSILE_RANGE,
		INITIAL_MISSILE_ALTITUDE,
		INITIAL_HRZ_VEL,
		INITIAL_VRT_VEL
	):

		# THE INIT FOR THIS SIMULATION IS THE INPUT FOR THE UPDATE IN THE ABSTRACTED VERSION.
		self.FLAG = FLAG
		self.FIN_DEFLECTION_DEGREES = FIN_DEFLECTION_DEGREES # Degrees.
		self.FIN_DEFLECTION_RADIANS = np.radians(self.FIN_DEFLECTION_DEGREES) # Radians.
		self.MAX_TIME = MAX_TIME # Seconds.
		self.TIME_STEP = TIME_STEP # Seconds. Variable.
		INITIAL_RANGE = INITIAL_MISSILE_RANGE # Meters.
		INITIAL_ALTITUDE = INITIAL_MISSILE_ALTITUDE # Meters.
		INITIAL_HORIZ_SPEED = INITIAL_HRZ_VEL # Meters per second.
		INITIAL_VERT_SPEED = INITIAL_VRT_VEL # Meters per second.

		FILE_ID = ""
		if self.FLAG == 0:
			FILE_ID = f"LINEAR_APPROX_1_{FIN_DEFLECTION_DEGREES}"
			print(FILE_ID)
		if self.FLAG == 1:
			FILE_ID = f"LINEAR_APPROX_2_{FIN_DEFLECTION_DEGREES}"
			print(FILE_ID)
		if self.FLAG == 2:
			FILE_ID = f"DIFFERENTIAL_APPROX_1_{FIN_DEFLECTION_DEGREES}"
			print(FILE_ID)
		if self.FLAG == 3:
			FILE_ID = f"DIFFERENTIAL_APPROX_2_{FIN_DEFLECTION_DEGREES}"
			print(FILE_ID)

		### MOCK HELLFIRE PARAMS ###
		self.REFERENCE_DIAMETER = 0.18 # Meters.
		self.REFERENCE_AREA = np.pi * (self.REFERENCE_DIAMETER ** 2) / 4 # Meters squared.
		# self.REFERENCE_LENGTH = 1.85026 # Meters.
		self.REFERENCE_LENGTH = 1.85026 - 0.249733 # Meters.
		self.WING_HALF_SPAN = 66.1175 * MM_TO_M / 2.0 # Meters.
		self.WING_TIP_CHORD = 91.047 * MM_TO_M # Meters.
		self.WING_ROOT_CHORD = 0.123564 # Meters.
		self.TAIL_HALF_SPAN = 71.3548 * MM_TO_M / 2.0 # Meters.
		self.TAIL_TIP_CHORD = 0.387894 # Meters.
		self.TAIL_ROOT_CHORD = 0.48084 # Meters.
		self.NOSE_LENGTH = 0.249733 # Meters.
		self.DISTANCE_FROM_BASE_OF_NOSE_TO_WING = 0.323925 # Meters.
		# self.CENTER_OF_DEFLECTION_FROM_NOSE = 1.8059 # Meters.
		self.CENTER_OF_DEFLECTION_FROM_NOSE = 1.8059 - self.NOSE_LENGTH # Meters.
		self.WING_AREA = 0.5 * self.WING_HALF_SPAN * (self.WING_TIP_CHORD + self.WING_ROOT_CHORD) # Meters squared.
		self.TAIL_AREA = 0.5 * self.TAIL_HALF_SPAN * (self.TAIL_TIP_CHORD + self.TAIL_ROOT_CHORD) # Meters squared.
		self.NOSE_AREA = self.NOSE_LENGTH * self.REFERENCE_DIAMETER # Meters squared.
		self.PLANFORM_AREA = (self.REFERENCE_LENGTH - self.NOSE_LENGTH) * self.REFERENCE_DIAMETER + 0.667 * self.NOSE_LENGTH * self.REFERENCE_DIAMETER # Meters squared.
		self.NOSE_CENTER_OF_PRESSURE = 0.67 * self.NOSE_LENGTH # Meters.
		self.WING_CENTER_OF_PRESSURE = self.NOSE_LENGTH + self.DISTANCE_FROM_BASE_OF_NOSE_TO_WING + 0.7 * self.WING_ROOT_CHORD - 0.2 * self.WING_TIP_CHORD # Meters.
		self.AN = 0.67 * self.NOSE_LENGTH * self.REFERENCE_DIAMETER # Meters squared.
		self.AB = (self.REFERENCE_LENGTH - self.NOSE_LENGTH) * self.REFERENCE_DIAMETER # Meters squared.
		self.BODY_CENTER_OF_PRESSURE = (0.67 * self.AN * self.NOSE_LENGTH + self.AB * (self.NOSE_LENGTH + 0.5 * (self.REFERENCE_LENGTH - self.NOSE_LENGTH))) / (self.AN + self.AB) # Meters.

		### STATE ###
		# TIME.
		self.TIME_OF_FLIGHT = 0.0 # Seconds.

		# POSITION.
		self.POSITION = npa([INITIAL_RANGE, INITIAL_ALTITUDE]) # Meters.

		# VELOCITY
		self.VELOCITY = npa([INITIAL_HORIZ_SPEED, INITIAL_VERT_SPEED]) # Meters per second.
		self.SPEED = la.norm(self.VELOCITY)

		# ACCELERATION.
		self.ACCELERATION = npa([0.0, 0.0]) # Meters per second squared.
		self.SPECIFIC_FORCE = npa([0.0, 0.0]) # Meters per second squared.
		self.NORMAL_ACCEL = 0.0 # Meters per second squared.

		# THIS IS THE ONLY TIME THETA IS ASSUMED TO BE ALIGNED WITH VELOCITY.
		self.THETA = returnEl(INITIAL_VERT_SPEED, INITIAL_HORIZ_SPEED) + 0.1 # Radians.
		self.THETA_DEG = np.degrees(self.THETA) # Degrees.
		self.THETA_DOT = 0.0 # Radians per second..
		self.THETA_DOT_DEG = np.degrees(self.THETA_DOT) # Degrees per second.

		# TRANSFORMATION MATRIX.
		self.BODY_TO_RANGE_AND_ALTITUDE = ct.BODY_TO_RANGE_AND_ALTITUDE(-self.THETA)

		# POSITIVE ALPHA MEANS NOSE BELOW FREE STREAM VELOCITY.
		self.BODY_VELOCITY = self.BODY_TO_RANGE_AND_ALTITUDE @ self.VELOCITY

		# VERTICAL ANGLE OF ATTACK.
		self.ALPHA = 0.0
		self.ALPHA_DEG = np.degrees(self.ALPHA)
		self.ALPHA_DOT = 0.0
		self.ALPHA_DOT_DEG = np.degrees(self.ALPHA_DOT)

		### COMPONENTS ###
		# ATMOSPHERE.
		self.ATMOSPHERE = Atmosphere()
		self.ATMOSPHERE.update(self.POSITION[1], self.SPEED)
		self.RHO = self.ATMOSPHERE.rho
		self.P = self.ATMOSPHERE.p
		self.A = self.ATMOSPHERE.a
		self.G = self.ATMOSPHERE.g
		self.Q = self.ATMOSPHERE.q
		self.MACH = self.ATMOSPHERE.mach
		self.BETA = None # Non dimensional >>> Zarchan - "Normalized Speed"
		if self.MACH > 1:
			self.BETA = np.sqrt(self.MACH ** 2 - 1)
		else:
			self.BETA = self.MACH

		# MASS AND MOTOR PROPERTIES.
		self.MASS_AND_MOTOR = MockHellfireMassAndMotor()
		self.MASS_AND_MOTOR.update(self.TIME_OF_FLIGHT, self.P)
		self.MASS = self.MASS_AND_MOTOR.MASS
		self.IYY = self.MASS_AND_MOTOR.TRANSVERSE_MOI
		self.CG = self.MASS_AND_MOTOR.XCG
		self.THRUST = self.MASS_AND_MOTOR.THRUST

		# DIFFERENTIAL DYNAMICS.
		self.E = 0.0
		self.EDOT = 0.0
		self.EDOTDOT = 0.0

		self.DYNAMICS_STATE = {

			# DYNAMICS
			"DT": self.TIME_STEP,
			"TOF": self.TIME_OF_FLIGHT,
			"MAX_TOF": self.MAX_TIME,
			"RNG": self.POSITION[0],
			"ALT": self.POSITION[1],
			"THT": self.THETA,
			"THT_DEG": self.THETA_DEG,
			"THT_DOT": self.THETA_DOT,
			"THT_DOT_DEG": self.THETA_DOT_DEG,
			"VEL_HRZ": self.VELOCITY[0],
			"VEL_VRT": self.VELOCITY[1],
			"SPD": self.SPEED,
			"U": self.BODY_VELOCITY[0],
			"W": self.BODY_VELOCITY[1],
			"ACC_HRZ": self.ACCELERATION[0],
			"ACC_VRT": self.ACCELERATION[1],
			"U_DOT": self.SPECIFIC_FORCE[0],
			"W_DOT": self.SPECIFIC_FORCE[1],
			"AOA": self.ALPHA,
			"AOA_DEG": self.ALPHA_DEG,
			"AOA_DOT": self.ALPHA_DOT,
			"AOA_DOT_DEG": self.ALPHA_DOT_DEG,

			# ATMOSPHERE
			"RHO": self.ATMOSPHERE.rho,
			"P": self.ATMOSPHERE.p,
			"A": self.ATMOSPHERE.a,
			"G": self.ATMOSPHERE.g,
			"Q": self.ATMOSPHERE.q,
			"MACH": self.ATMOSPHERE.mach,

			# MASS AND MOTOR
			"XCG": self.CG,
			"T_MOI": self.IYY,
			"MASS": self.MASS,
			"THR": self.THRUST,

		}

		### RK4 INTEGRATOR ###
		# STATES
		self.INTEGRATION_PASS = 0
		self.AOA_0 = self.ALPHA
		self.E_0 = self.E
		self.EDOT_0 = self.EDOT
		self.THT_0 = self.THETA
		self.THTDOT_0 = self.THETA_DOT
		self.POS_0 = self.POSITION
		self.VEL_0 = self.VELOCITY

		# DERIVATIVES
		self.AOADOT_1 = 0.0
		self.EDOTDOT_1 = 0.0
		self.EDOT_1 = 0.0
		self.THTDOTDOT_1 = 0.0
		self.THTDOT_1 = 0.0
		self.ACC_1 = np.zeros(2)
		self.VEL_1 = np.zeros(2)

		self.AOADOT_2 = 0.0
		self.EDOTDOT_2 = 0.0
		self.EDOT_2 = 0.0
		self.THTDOTDOT_2 = 0.0
		self.THTDOT_2 = 0.0
		self.ACC_2 = np.zeros(2)
		self.VEL_2 = np.zeros(2)

		self.AOADOT_3 = 0.0
		self.EDOTDOT_3 = 0.0
		self.EDOT_3 = 0.0
		self.THTDOTDOT_3 = 0.0
		self.THTDOT_3 = 0.0
		self.ACC_3 = np.zeros(2)
		self.VEL_3 = np.zeros(2)

		self.AOADOT_4 = 0.0
		self.EDOTDOT_4 = 0.0
		self.EDOT_4 = 0.0
		self.THTDOTDOT_4 = 0.0
		self.THTDOT_4 = 0.0
		self.ACC_4 = np.zeros(2)
		self.VEL_4 = np.zeros(2)

		### LOGGING ###
		self.LOG_FILE = open(f"MockHellfire_HighFidelityThreeDOF/output/{FILE_ID}.txt", "w")
		HEADER = ""
		COUNT = len(self.DYNAMICS_STATE.keys())
		for INDEX, KEY in enumerate(self.DYNAMICS_STATE.keys()):
			if INDEX + 1 == COUNT:
				HEADER += f"{KEY}\n"
			else:
				HEADER += f"{KEY} "
		self.LOG_FILE.write(HEADER)
		DATA = ""
		for INDEX, KEY in enumerate(self.DYNAMICS_STATE.keys()):
			VALUE = self.DYNAMICS_STATE[f"{KEY}"]
			if INDEX + 1 == COUNT:
				DATA += f"{VALUE}\n"
			else:
				DATA += f"{VALUE} "
		self.LOG_FILE.write(DATA)

	def populateDynamicsState(self):

		self.DYNAMICS_STATE = {

			# DYNAMICS
			"DT": self.TIME_STEP,
			"TOF": self.TIME_OF_FLIGHT,
			"MAX_TOF": self.MAX_TIME,
			"RNG": self.POSITION[0],
			"ALT": self.POSITION[1],
			"THT": self.THETA,
			"THT_DEG": self.THETA_DEG,
			"THT_DOT": self.THETA_DOT,
			"THT_DOT_DEG": self.THETA_DOT_DEG,
			"VEL_HRZ": self.VELOCITY[0],
			"VEL_VRT": self.VELOCITY[1],
			"SPD": self.SPEED,
			"U": self.BODY_VELOCITY[0],
			"W": self.BODY_VELOCITY[1],
			"ACC_HRZ": self.ACCELERATION[0],
			"ACC_VRT": self.ACCELERATION[1],
			"U_DOT": self.SPECIFIC_FORCE[0],
			"W_DOT": self.SPECIFIC_FORCE[1],
			"AOA": self.ALPHA,
			"AOA_DEG": self.ALPHA_DEG,
			"AOA_DOT": self.ALPHA_DOT,
			"AOA_DOT_DEG": self.ALPHA_DOT_DEG,

			# ATMOSPHERE
			"RHO": self.ATMOSPHERE.rho,
			"P": self.ATMOSPHERE.p,
			"A": self.ATMOSPHERE.a,
			"G": self.ATMOSPHERE.g,
			"Q": self.ATMOSPHERE.q,
			"MACH": self.ATMOSPHERE.mach,

			# MASS AND MOTOR
			"XCG": self.CG,
			"T_MOI": self.IYY,
			"MASS": self.MASS,
			"THR": self.THRUST,

		}

	def logDynamicsState(self):
		DATA = ""
		COUNT = len(self.DYNAMICS_STATE.keys())
		for INDEX, KEY in enumerate(self.DYNAMICS_STATE.keys()):
			VALUE = self.DYNAMICS_STATE[f"{KEY}"]
			if INDEX + 1 == COUNT:
				DATA += f"{VALUE}\n"
			else:
				DATA += f"{VALUE} "
		self.LOG_FILE.write(DATA)

	def getDynamicsState(self):
		return self.DYNAMICS_STATE

	def dynamics(self):

		GO = True
		while GO:

			# Update atmosphere.
			self.ATMOSPHERE.update(self.POSITION[1], self.SPEED)
			self.RHO = self.ATMOSPHERE.rho
			self.P = self.ATMOSPHERE.p
			self.A = self.ATMOSPHERE.a
			self.G = self.ATMOSPHERE.g
			self.Q = self.ATMOSPHERE.q
			self.MACH = self.ATMOSPHERE.mach
			self.BETA = None # Non dimensional >>> Zarchan - "Normalized Speed"
			if self.MACH > 1:
				self.BETA = np.sqrt(self.MACH ** 2 - 1)
			else:
				self.BETA = self.MACH

			# MASS AND MOTOR PROPERTIES.
			self.MASS_AND_MOTOR.update(self.TIME_OF_FLIGHT, self.P)
			self.MASS = self.MASS_AND_MOTOR.MASS
			self.IYY = self.MASS_AND_MOTOR.TRANSVERSE_MOI
			self.CG = self.MASS_AND_MOTOR.XCG
			self.THRUST = self.MASS_AND_MOTOR.THRUST

			# AERODYNAMICS AND DERIVATIVES.
			THETA_DOT_DOT = 0.0
			if self.FLAG == 0:
				
				# NORMAL COEFFICIENT AND PITCHING MOMENT COEFFICIENT CALCULATION, NON DIMENSIONAL.
				CN = 2 * self.ALPHA + (1.5 * self.PLANFORM_AREA * self.ALPHA * self.ALPHA) / self.REFERENCE_AREA + (8 * self.WING_AREA * self.ALPHA) / (self.BETA * self.REFERENCE_AREA) + (8 * self.TAIL_AREA * (self.ALPHA + self.FIN_DEFLECTION_RADIANS)) / (self.BETA * self.REFERENCE_AREA)
				CM = 2 * self.ALPHA * ((self.CG - self.NOSE_CENTER_OF_PRESSURE) / self.REFERENCE_DIAMETER) + ((1.5 * self.PLANFORM_AREA * self.ALPHA * self.ALPHA) / self.REFERENCE_AREA) * ((self.CG - self.BODY_CENTER_OF_PRESSURE) / self.REFERENCE_DIAMETER) + ((8 * self.WING_AREA * self.ALPHA) / (self.BETA * self.REFERENCE_AREA)) * ((self.CG - self.WING_CENTER_OF_PRESSURE) / self.REFERENCE_DIAMETER) + ((8 * self.TAIL_AREA * (self.ALPHA + self.FIN_DEFLECTION_RADIANS)) / (self.BETA * self.REFERENCE_AREA)) * ((self.CG - self.CENTER_OF_DEFLECTION_FROM_NOSE) / self.REFERENCE_DIAMETER)
				
				# DERIVATIVES.
				THETA_DOT_DOT = (self.Q * self.REFERENCE_AREA * self.REFERENCE_DIAMETER * CM) / self.IYY # Radians per second squared.

				self.NORMAL_ACCEL = (self.Q * self.REFERENCE_AREA * CN) / (self.MASS) # Meters per second squared.

				CA = 0.35
				AXIAL_FORCE = self.Q * CA * self.REFERENCE_AREA
				AXIAL_ACCEL = AXIAL_FORCE / self.MASS
				
				self.SPECIFIC_FORCE = npa([(self.THRUST / self.MASS) + AXIAL_ACCEL, self.NORMAL_ACCEL])
				LOCAL_G = npa([0.0, -1.0 * self.G])
				BODY_G = self.BODY_TO_RANGE_AND_ALTITUDE @ LOCAL_G
				self.SPECIFIC_FORCE += BODY_G
				self.ACCELERATION = self.SPECIFIC_FORCE @ self.BODY_TO_RANGE_AND_ALTITUDE # DERIVATIVE

			elif self.FLAG == 1:

				# NORMAL COEFFICIENT AND PITCHING MOMENT COEFFICIENT CALCULATION, NON DIMENSIONAL.
				# CMA = -5.0 # PER RAD
				# CMD = -20.0 # PER RAD
				CMA = -15.0 # PER RAD
				CMD = -5.0 # PER RAD

				CNA = 20.0 # PER RAD
				CND = 4.0 # PER RAD

				CN = CNA * self.ALPHA + CND * self.FIN_DEFLECTION_RADIANS
				CM = CMA * self.ALPHA + CMD * self.FIN_DEFLECTION_RADIANS

				# DERIVATIVES.
				THETA_DOT_DOT = (self.Q * self.REFERENCE_AREA * self.REFERENCE_DIAMETER * CM) / self.IYY # Radians per second squared.

				self.NORMAL_ACCEL = ((self.Q * self.REFERENCE_AREA * CN) / self.MASS) # Meters per second squared.
				
				CA = 0.35
				AXIAL_FORCE = self.Q * CA * self.REFERENCE_AREA
				AXIAL_ACCEL = AXIAL_FORCE / self.MASS
				
				self.SPECIFIC_FORCE = npa([(self.THRUST / self.MASS) + AXIAL_ACCEL, self.NORMAL_ACCEL])
				LOCAL_G = npa([0.0, -1.0 * self.G])
				BODY_G = self.BODY_TO_RANGE_AND_ALTITUDE @ LOCAL_G
				self.SPECIFIC_FORCE += BODY_G
				self.ACCELERATION = self.SPECIFIC_FORCE @ self.BODY_TO_RANGE_AND_ALTITUDE # DERIVATIVE

			elif self.FLAG == 2:

				# AERODYNAMICS
				TEMP1 = (self.CG - self.WING_CENTER_OF_PRESSURE) / self.REFERENCE_DIAMETER 
				TEMP2 = (self.CG - self.CENTER_OF_DEFLECTION_FROM_NOSE) / self.REFERENCE_DIAMETER
				TEMP3 = (self.CG - self.BODY_CENTER_OF_PRESSURE) / self.REFERENCE_DIAMETER
				TEMP4 = (self.CG - self.NOSE_CENTER_OF_PRESSURE) / self.REFERENCE_DIAMETER
				Y1 = 2 * TEMP4 + 8 * self.WING_AREA * TEMP1 / (self.BETA * self.REFERENCE_AREA) + 8 * self.TAIL_AREA * TEMP2 / (self.BETA * self.REFERENCE_AREA)
				Y2 = 1.5 * self.PLANFORM_AREA * TEMP3 / self.REFERENCE_AREA
				Y3 = 8 * self.TAIL_AREA * TEMP2 * self.FIN_DEFLECTION_RADIANS / (self.BETA * self.REFERENCE_AREA)
				ALPHA_TRIM = (-Y1 - np.sqrt((Y1 ** 2) - 4 * Y2 * Y3)) / (2 * Y2)
				CNA = 2 + 1.5 * self.PLANFORM_AREA * ALPHA_TRIM / self.REFERENCE_AREA + 8 * self.WING_AREA / (self.BETA * self.REFERENCE_AREA) + 8 * self.TAIL_AREA / (self.BETA * self.REFERENCE_AREA)
				CND = 8 * self.TAIL_AREA / (self.BETA * self.REFERENCE_AREA)
				ZA = -1 * self.Q * self.REFERENCE_AREA * CNA / (self.MASS * self.SPEED)
				ZD = -1 * self.Q * self.REFERENCE_AREA * CND / (self.MASS * self.SPEED)
				CMAP = 2 * TEMP4 + 1.5 * self.PLANFORM_AREA * ALPHA_TRIM * TEMP3 / self.REFERENCE_AREA + 8 * self.WING_AREA * TEMP1 / (self.BETA * self.REFERENCE_AREA)
				CMA = CMAP + 8 * self.TAIL_AREA * TEMP2 / (self.BETA * self.REFERENCE_AREA)
				CMD = 8 * self.TAIL_AREA * TEMP2 / (self.BETA * self.REFERENCE_AREA)
				MA = self.Q * self.REFERENCE_AREA * self.REFERENCE_DIAMETER * CMA / self.IYY
				MD = self.Q * self.REFERENCE_AREA * self.REFERENCE_DIAMETER * CMD / self.IYY

				OMEGA_Z = np.sqrt((MA * ZD - MD * ZA) / ZD)
				OMEGA_AF = np.sqrt(-1 * MA)
				ZETA_AF = ZA * OMEGA_AF / (2 * MA)
				K1 = -1 * self.SPEED * ((MA * ZD - ZA * MD) / (1845 * MA))
				K3 = 1845 * K1 / self.SPEED
				TA = MD / (MA * ZD - ZA * MD)

				self.EDOTDOT = (OMEGA_AF ** 2) * (self.FIN_DEFLECTION_DEGREES - self.E - 2 * ZETA_AF * self.EDOT / OMEGA_AF) # DERIVATIVE

				ETERM = K3 * (self.E + TA * self.EDOT)
				self.THETA_DOT_DEG = -ETERM# STATE AND DERIVATIVE
				self.THETA_DOT = np.radians(self.THETA_DOT_DEG)

				self.NORMAL_ACCEL = K1 * (self.E - (self.EDOTDOT / (OMEGA_Z ** 2))) * self.G
				
				CA = 0.35
				AXIAL_FORCE = self.Q * CA * self.REFERENCE_AREA
				AXIAL_ACCEL = AXIAL_FORCE / self.MASS
				
				self.SPECIFIC_FORCE = npa([(self.THRUST / self.MASS) + AXIAL_ACCEL, self.NORMAL_ACCEL])
				LOCAL_G = npa([0.0, -1.0 * self.G])
				BODY_G = self.BODY_TO_RANGE_AND_ALTITUDE @ LOCAL_G
				self.SPECIFIC_FORCE += BODY_G
				self.ACCELERATION = self.SPECIFIC_FORCE @ self.BODY_TO_RANGE_AND_ALTITUDE # DERIVATIVE

			elif self.FLAG == 3:

				VELMAG = la.norm(self.VELOCITY)
				TEMP1 = (self.CG - self.WING_CENTER_OF_PRESSURE) / self.REFERENCE_DIAMETER 
				TEMP2 = (self.CG - self.CENTER_OF_DEFLECTION_FROM_NOSE) / self.REFERENCE_DIAMETER
				TEMP3 = (self.CG - self.BODY_CENTER_OF_PRESSURE) / self.REFERENCE_DIAMETER
				TEMP4 = (self.CG - self.NOSE_CENTER_OF_PRESSURE) / self.REFERENCE_DIAMETER
				BETA = None
				if self.MACH > 1:
					BETA = np.sqrt(self.MACH ** 2 - 1) # Non dimensional.
				else:
					BETA = self.MACH # Non dimensional.
				# CNTRIM = self.MASS * COMMAND / (self.Q * self.REFERENCE_AREA)
				CNTRIM = 8 * self.TAIL_AREA * TEMP2 * self.FIN_DEFLECTION_RADIANS / (self.BETA * self.REFERENCE_AREA)
				Y1 = 2 + 8 * self.WING_AREA / (BETA * self.REFERENCE_AREA) + 8 * self.TAIL_AREA / (BETA * self.REFERENCE_AREA)
				Y2 = 1.5 * self.PLANFORM_AREA / self.REFERENCE_AREA
				Y3 = 8 * self.TAIL_AREA / (BETA * self.REFERENCE_AREA)
				Y4 = 2 * TEMP4 + 8 * self.WING_AREA * TEMP1 / (BETA * self.REFERENCE_AREA) + 8 * self.TAIL_AREA * TEMP2 / (BETA * self.REFERENCE_AREA)
				Y5 = 1.5 * self.PLANFORM_AREA * TEMP3 / self.REFERENCE_AREA
				Y6 = 8 * self.TAIL_AREA * TEMP2 / (BETA * self.REFERENCE_AREA)
				P2 = Y2 - (Y3 * Y5) / Y6
				P3 = Y1 - (Y3 * Y4) / Y6
				ALPHA_TRIM = (-P3 + np.sqrt(P3 * P3 + 4 * P2 * CNTRIM)) / (2 * P2)
				DELTA_TRIM = (-1 * Y4 * ALPHA_TRIM - Y5 * ALPHA_TRIM * ALPHA_TRIM) / Y6
				CNA = 2 + 1.5 * self.PLANFORM_AREA * ALPHA_TRIM / self.REFERENCE_AREA + 8 * self.WING_AREA / (BETA * self.REFERENCE_AREA) + 8 * self.TAIL_AREA / (BETA * self.REFERENCE_AREA)
				CND = 8 * self.TAIL_AREA / (BETA * self.REFERENCE_AREA)
				ZA = -1 * self.Q * self.REFERENCE_AREA * CNA / (self.MASS * VELMAG)
				ZD = -1 * self.Q * self.REFERENCE_AREA * CND / (self.MASS * VELMAG)
				CMAP = 2 * TEMP4 + 1.5 * self.PLANFORM_AREA * ALPHA_TRIM * TEMP3 / self.REFERENCE_AREA + 8 * self.WING_AREA * TEMP1 / (BETA * self.REFERENCE_AREA)
				CMA = CMAP + 8 * self.TAIL_AREA * TEMP2 / (BETA * self.REFERENCE_AREA)
				CMD = 8 * self.TAIL_AREA * TEMP2 / (BETA * self.REFERENCE_AREA)
				MA = self.Q * self.REFERENCE_AREA * self.REFERENCE_DIAMETER * CMA / self.IYY
				MD = self.Q * self.REFERENCE_AREA * self.REFERENCE_DIAMETER * CMD / self.IYY

				OMEGA_Z = np.sqrt((MA * ZD - MD * ZA) / ZD) # Per second.
				OMEGA_AF = np.sqrt(-1 * MA) # Per second.
				ZETA_AF = ZA * OMEGA_AF / (2 * MA) # Non dimensional damping.
				KR = 0.1 # Gain.
				K1 = -1 * VELMAG * ((MA * ZD - ZA * MD) / (1845 * MA)) # Gain.
				TA = MD / (MA * ZD - MD * ZA) # Time constant. Second.
				K3 = 1845 * K1 / VELMAG

				# (1/S * 1/S) * (DEGREES - E * EDOT / OMEGA_AF)
				self.EDOTDOT = (OMEGA_AF ** 2) * (self.FIN_DEFLECTION_DEGREES - self.E - 2 * ZETA_AF * self.EDOT / OMEGA_AF) # DERIVATIVE

				ETERM = K3 * (self.E + TA * self.EDOT)
				self.THETA_DOT_DEG = -ETERM# STATE AND DERIVATIVE
				self.THETA_DOT = np.radians(self.THETA_DOT_DEG)

				self.NORMAL_ACCEL = K1 * (self.E - (self.EDOTDOT / (OMEGA_Z ** 2))) * self.G

				CA = 0.35
				AXIAL_FORCE = self.Q * CA * self.REFERENCE_AREA
				AXIAL_ACCEL = AXIAL_FORCE / self.MASS
				
				self.SPECIFIC_FORCE = npa([(self.THRUST / self.MASS) + AXIAL_ACCEL, self.NORMAL_ACCEL])
				LOCAL_G = npa([0.0, -1.0 * self.G])
				BODY_G = self.BODY_TO_RANGE_AND_ALTITUDE @ LOCAL_G
				self.SPECIFIC_FORCE += BODY_G
				self.ACCELERATION = self.SPECIFIC_FORCE @ self.BODY_TO_RANGE_AND_ALTITUDE # DERIVATIVE

			"""

			STATE:
			EDOT
			E
			THETADOT
			THETA
			VELOCITY
			POSITION

			DERIVATIVES:
			EDOTDOT
			EDOT
			THETADOTDOT
			THETADOT
			ACCELERATION
			VELOCITY

			"""

			# NEW INTEGRATION.
			if self.INTEGRATION_PASS == 0:

				# STATES.
				self.AOA_0 = self.ALPHA
				self.E_0 = self.E
				self.EDOT_0 = self.EDOT
				self.THT_0 = self.THETA
				self.THTDOT_0 = self.THETA_DOT
				self.POS_0 = self.POSITION
				self.VEL_0 = self.VELOCITY

				# DERIVATIVES.
				GAMMA_DOT =  ((self.SPECIFIC_FORCE[1]) / self.SPEED)
				self.AOADOT_1 = self.THETA_DOT - GAMMA_DOT
				self.ALPHA_DOT = self.AOADOT_1
				self.ALPHA_DOT_DEG = np.degrees(self.ALPHA_DOT)
				self.EDOT_1 = self.EDOT
				self.EDOTDOT_1 = self.EDOTDOT
				self.THTDOT_1 = self.THETA_DOT
				self.THTDOTDOT_1 = THETA_DOT_DOT
				self.VEL_1 = self.VELOCITY
				self.ACC_1 = self.ACCELERATION

				# INTEGRATION.
				self.ALPHA = self.AOA_0 + self.AOADOT_1 * (self.TIME_STEP / 2.0)
				self.ALPHA_DEG = np.degrees(self.ALPHA)
				self.E = self.E_0 + self.EDOT_1 * (self.TIME_STEP / 2.0)
				self.EDOT = self.EDOT_0 + self.EDOTDOT_1 * (self.TIME_STEP / 2.0)
				self.THETA = self.THT_0 + self.THTDOT_1 * (self.TIME_STEP / 2.0)
				self.THETA_DOT = self.THTDOT_0 + self.THTDOTDOT_1 * (self.TIME_STEP / 2.0)
				self.THETA_DEG = np.degrees(self.THETA)
				self.THETA_DOT_DEG = np.degrees(self.THETA_DOT)
				self.POSITION = self.POS_0 + self.VEL_1 * (self.TIME_STEP / 2.0)
				self.VELOCITY = self.VEL_0 + self.ACC_1 * (self.TIME_STEP / 2.0)
				self.BODY_VELOCITY = self.BODY_TO_RANGE_AND_ALTITUDE @ self.VELOCITY
				self.SPEED = la.norm(self.VELOCITY)

				# ITERATE INTEGRATION.
				self.INTEGRATION_PASS += 1

				# UPDATE TIME OF FLIGHT.
				self.TIME_OF_FLIGHT += (self.TIME_STEP / 2.0)

			elif self.INTEGRATION_PASS == 1:

				# DERIVATIVES.
				GAMMA_DOT =  ((self.SPECIFIC_FORCE[1]) / self.SPEED)
				self.AOADOT_2 = self.THETA_DOT - GAMMA_DOT
				self.ALPHA_DOT = self.AOADOT_2
				self.ALPHA_DOT_DEG = np.degrees(self.ALPHA_DOT)
				self.EDOT_2 = self.EDOT
				self.EDOTDOT_2 = self.EDOTDOT
				self.THTDOT_2 = self.THETA_DOT
				self.THTDOTDOT_2 = THETA_DOT_DOT
				self.VEL_2 = self.VELOCITY
				self.ACC_2 = self.ACCELERATION

				# INTEGRATION.
				self.ALPHA = self.AOA_0 + self.AOADOT_2 * (self.TIME_STEP / 2.0)
				self.ALPHA_DEG = np.degrees(self.ALPHA)
				self.E = self.E_0 + self.EDOT_2 * (self.TIME_STEP / 2.0)
				self.EDOT = self.EDOT_0 + self.EDOTDOT_2 * (self.TIME_STEP / 2.0)
				self.THETA = self.THT_0 + self.THTDOT_2 * (self.TIME_STEP / 2.0)
				self.THETA_DOT = self.THTDOT_0 + self.THTDOTDOT_2 * (self.TIME_STEP / 2.0)
				self.THETA_DEG = np.degrees(self.THETA)
				self.THETA_DOT_DEG = np.degrees(self.THETA_DOT)
				self.POSITION = self.POS_0 + self.VEL_2 * (self.TIME_STEP / 2.0)
				self.VELOCITY = self.VEL_0 + self.ACC_2 * (self.TIME_STEP / 2.0)
				self.BODY_VELOCITY = self.BODY_TO_RANGE_AND_ALTITUDE @ self.VELOCITY
				self.SPEED = la.norm(self.VELOCITY)

				# ITERATE INTEGRATION.
				self.INTEGRATION_PASS += 1

			elif self.INTEGRATION_PASS == 2:

				# DERIVATIVES.
				GAMMA_DOT =  ((self.SPECIFIC_FORCE[1]) / self.SPEED)
				self.AOADOT_3 = self.THETA_DOT - GAMMA_DOT
				self.ALPHA_DOT = self.AOADOT_3
				self.ALPHA_DOT_DEG = np.degrees(self.ALPHA_DOT)
				self.EDOT_3 = self.EDOT
				self.EDOTDOT_3 = self.EDOTDOT
				self.THTDOT_3 = self.THETA_DOT
				self.THTDOTDOT_3 = THETA_DOT_DOT
				self.VEL_3 = self.VELOCITY
				self.ACC_3 = self.ACCELERATION

				# INTEGRATION.
				self.ALPHA = self.AOA_0 + self.AOADOT_3 * (self.TIME_STEP)
				self.ALPHA_DEG = np.degrees(self.ALPHA)
				self.E = self.E_0 + self.EDOT_3 * (self.TIME_STEP)
				self.EDOT = self.EDOT_0 + self.EDOTDOT_3 * (self.TIME_STEP)
				self.THETA = self.THT_0 + self.THTDOT_3 * (self.TIME_STEP)
				self.THETA_DOT = self.THTDOT_0 + self.THTDOTDOT_3 * (self.TIME_STEP)
				self.THETA_DEG = np.degrees(self.THETA)
				self.THETA_DOT_DEG = np.degrees(self.THETA_DOT)
				self.POSITION = self.POS_0 + self.VEL_3 * (self.TIME_STEP)
				self.VELOCITY = self.VEL_0 + self.ACC_3 * (self.TIME_STEP)
				self.BODY_VELOCITY = self.BODY_TO_RANGE_AND_ALTITUDE @ self.VELOCITY
				self.SPEED = la.norm(self.VELOCITY)

				# ITERATE INTEGRATION.
				self.INTEGRATION_PASS += 1

				# UPDATE TIME OF FLIGHT.
				self.TIME_OF_FLIGHT += (self.TIME_STEP / 2.0)

			else:

				# DERIVATIVES.
				GAMMA_DOT =  ((self.SPECIFIC_FORCE[1]) / self.SPEED)
				self.AOADOT_4 = self.THETA_DOT - GAMMA_DOT
				self.ALPHA_DOT = self.AOADOT_4
				self.ALPHA_DOT_DEG = np.degrees(self.ALPHA_DOT)
				self.EDOT_4 = self.EDOT
				self.EDOTDOT_4 = self.EDOTDOT
				self.THTDOT_4 = self.THETA_DOT
				self.THTDOTDOT_4 = THETA_DOT_DOT
				self.VEL_4 = self.VELOCITY
				self.ACC_4 = self.ACCELERATION

				# INTEGRATION.
				self.ALPHA = self.AOA_0 + (self.TIME_STEP / 6.0) * (self.AOADOT_1 + 2 * self.AOADOT_2 + 2 * self.AOADOT_3 + self.AOADOT_4)
				self.ALPHA_DEG = np.degrees(self.ALPHA)
				self.E = self.E_0 + (self.TIME_STEP / 6.0) * (self.EDOT_1 + 2 * self.EDOT_2 + 2 * self.EDOT_3 + self.EDOT_4)
				self.EDOT = self.EDOT_0 + (self.TIME_STEP / 6.0) * (self.EDOTDOT_1 + 2 * self.EDOTDOT_2 + 2 * self.EDOTDOT_3 + self.EDOTDOT_4)
				self.THETA = self.THT_0 + (self.TIME_STEP / 6.0) * (self.THTDOT_1 + 2 * self.THTDOT_2 + 2 * self.THTDOT_3 + self.THTDOT_4)
				self.THETA_DOT = self.THTDOT_0 + (self.TIME_STEP / 6.0) * (self.THTDOTDOT_1 + 2 * self.THTDOTDOT_2 + 2 * self.THTDOTDOT_3 + self.THTDOTDOT_4)
				self.THETA_DEG = np.degrees(self.THETA)
				self.THETA_DOT_DEG = np.degrees(self.THETA_DOT)
				self.POSITION = self.POS_0 + (self.TIME_STEP / 6.0) * (self.VEL_1 + 2 * self.VEL_2 + 2 * self.VEL_3 + self.VEL_4)
				self.VELOCITY = self.VEL_0 + (self.TIME_STEP / 6.0) * (self.ACC_1 + 2 * self.ACC_2 + 2 * self.ACC_3 + self.ACC_4)
				self.BODY_VELOCITY = self.BODY_TO_RANGE_AND_ALTITUDE @ self.VELOCITY
				self.SPEED = la.norm(self.VELOCITY)

				# ITERATE INTEGRATION.
				self.INTEGRATION_PASS = 0

			# RE ORIENT.
			self.BODY_TO_RANGE_AND_ALTITUDE = ct.BODY_TO_RANGE_AND_ALTITUDE(-self.THETA)

			# STORE DATA.
			self.populateDynamicsState()

			# LOG DATA.
			if self.INTEGRATION_PASS == 0:
				self.logDynamicsState()

			# Console report.
			if round(self.TIME_OF_FLIGHT, 3).is_integer():
				if self.INTEGRATION_PASS == 0:
					print(f"TOF {self.TIME_OF_FLIGHT:.2f} POS {self.POSITION} MACH {self.MACH:.2f}")

			# Performance and termination check.
			if self.INTEGRATION_PASS == 0:
				if self.TIME_OF_FLIGHT > self.MAX_TIME:
					GO = False
					print(f"MAX TIME - {self.TIME_OF_FLIGHT:.2f} POS {self.POSITION} MACH {self.MACH:.2f}\n")
				if self.POSITION[1] < 0:
					GO = False
					print(f"GROUND COLLISION - {self.TIME_OF_FLIGHT:.2f} POS {self.POSITION} MACH {self.MACH:.2f}\n")
				if np.isnan(np.sum(self.POSITION)):
					GO = False
					print(f"NAN - {self.TIME_OF_FLIGHT:.2f} POS {self.POSITION} MACH {self.MACH:.2f}\n")



if __name__ == "__main__":

	"""
	
	FLAG = 0 >>> SIMPLE LINEAR APPROXIMATIONS
	FLAG = 1 >>> FIXED COEFFICIENT APPROXIMATIONS
	FLAG = 2 >>> DIFFERENTIAL EQNS OF MOTION #1
	FLAG = 3 >>> DIFFERENTIAL EQNS OF MOTION #2
	
	"""

	# x = AirframeSimulation(
	# 	FLAG=0,
	# 	FIN_DEFLECTION_DEGREES=-1,
	# 	MAX_TIME=120,
	# 	TIME_STEP=(1.0 / 150.0),
	# 	INITIAL_MISSILE_RANGE=0.0,
	# 	INITIAL_MISSILE_ALTITUDE=0.0,
	# 	INITIAL_HRZ_VEL=10.0,
	# 	INITIAL_VRT_VEL=10.0
	# )
	# x.dynamics()
	# x = AirframeSimulation(
	# 	FLAG=1,
	# 	FIN_DEFLECTION_DEGREES=-1,
	# 	MAX_TIME=120,
	# 	TIME_STEP=(1.0 / 150.0),
	# 	INITIAL_MISSILE_RANGE=0.0,
	# 	INITIAL_MISSILE_ALTITUDE=0.0,
	# 	INITIAL_HRZ_VEL=10.0,
	# 	INITIAL_VRT_VEL=10.0
	# )
	# x.dynamics()
	x = AirframeSimulation(
		FLAG=2,
		FIN_DEFLECTION_DEGREES=-1,
		MAX_TIME=120,
		TIME_STEP=(1.0 / 50.0),
		INITIAL_MISSILE_RANGE=0.0,
		INITIAL_MISSILE_ALTITUDE=0.0,
		INITIAL_HRZ_VEL=10.0,
		INITIAL_VRT_VEL=10.0
	)
	x.dynamics()
	x = AirframeSimulation(
		FLAG=3,
		FIN_DEFLECTION_DEGREES=-1,
		MAX_TIME=120,
		TIME_STEP=(1.0 / 50.0),
		INITIAL_MISSILE_RANGE=0.0,
		INITIAL_MISSILE_ALTITUDE=0.0,
		INITIAL_HRZ_VEL=10.0,
		INITIAL_VRT_VEL=10.0
	)
	x.dynamics()
	

