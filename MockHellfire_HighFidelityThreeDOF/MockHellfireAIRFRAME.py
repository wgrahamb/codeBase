"""


TO DO:

1) GO THROUGH THE ENTIRE CODE AND CHECK UNITS. ***VERY IMPORTANT***
	A) MOVE AERODYNAMICS FROM CLASS TO AIRFRAME SIMULATION TO COMPARE WITH THE FIRST TWO.
4) ADD THRUST TO MOTION MODULE.
5) ADD DRAG PROFILE TO AERODYNAMICS.
7) VOTING SYSTEM FOR DYNAMICS UPDATE.
8) BASE CLASS - EACH MODULE NEEDS A "NEXT UPDATE TIME."
8) GUIDANCE AND CONTROL MODULES.
9) TARGET AND SEEKER MODELS
9) ADD AN INS.


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


COEFFICIENT GUESSES:

CMA = -5.0 PER RAD
CMD = -20.0 PER RAD

CNA = 15.0 PER RAD
CND = 4.0 PER RAD


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

	def __init__(self, FLAG):

		self.FLAG = FLAG

		### MOCK HELLFIRE PARAMS ###
		self.REFERENCE_DIAMETER = 0.18 # Meters.
		self.REFERENCE_AREA = np.pi * (self.REFERENCE_DIAMETER ** 2) / 4 # Meters squared.
		self.REFERENCE_LENGTH = 1.85026 # Meters.
		self.WING_HALF_SPAN = 66.1175 * MM_TO_M / 2.0 # Meters.
		self.WING_TIP_CHORD = 91.047 * MM_TO_M # Meters.
		self.WING_ROOT_CHORD = 0.123564 # Meters.
		self.TAIL_HALF_SPAN = 71.3548 * MM_TO_M / 2.0 # Meters.
		self.TAIL_TIP_CHORD = 0.387894 # Meters.
		self.TAIL_ROOT_CHORD = 0.48084 # Meters.
		self.NOSE_LENGTH = 0.249733 # Meters.
		self.DISTANCE_FROM_BASE_OF_NOSE_TO_WING = 0.323925 # Meters.
		self.CENTER_OF_DEFLECTION_FROM_NOSE = 1.8059 # Meters.
		self.WING_AREA = 0.5 * self.WING_HALF_SPAN * (self.WING_TIP_CHORD + self.WING_ROOT_CHORD) # Meters squared.
		self.TAIL_AREA = 0.5 * self.TAIL_HALF_SPAN * (self.TAIL_TIP_CHORD + self.TAIL_ROOT_CHORD) # Meters squared.
		self.NOSE_AREA = self.NOSE_LENGTH * self.REFERENCE_DIAMETER # Meters squared.
		self.PLANFORM_AREA = (self.REFERENCE_LENGTH - self.NOSE_LENGTH) * self.REFERENCE_DIAMETER + 0.667 * self.NOSE_LENGTH * self.REFERENCE_DIAMETER # Meters squared.
		self.NOSE_CENTER_OF_PRESSURE = 0.67 * self.NOSE_LENGTH # Meters.
		self.WING_CENTER_OF_PRESSURE = self.NOSE_LENGTH + self.DISTANCE_FROM_BASE_OF_NOSE_TO_WING + 0.7 * self.WING_ROOT_CHORD - 0.2 * self.WING_TIP_CHORD # Meters.
		self.AN = 0.67 * self.NOSE_LENGTH * self.REFERENCE_DIAMETER # Meters squared.
		self.AB = (self.REFERENCE_LENGTH - self.NOSE_LENGTH) * self.REFERENCE_DIAMETER # Meters squared.
		self.BODY_CENTER_OF_PRESSURE = (0.67 * self.AN * self.NOSE_LENGTH + self.AB * (self.NOSE_LENGTH + 0.5 * (self.REFERENCE_LENGTH - self.NOSE_LENGTH))) / (self.AN + self.AB) # Meters.

		# WILL BE AN INPUT.
		self.FIN_DEFLECTION_DEGREES = 0 # Degrees.
		self.FIN_DEFLECTION_RADIANS = np.radians(self.FIN_DEFLECTION_DEGREES) # Radians.

		### STATE ###
		# TIME.
		self.TIME_OF_FLIGHT = 0.0 # Seconds.
		self.TIME_STEP = 1.0 / 250.0 # Seconds. Variable.
		self.MAX_TIME = 40 # Seconds.

		# POSITION.
		INITIAL_RANGE = 0.0 # Meters.
		INITIAL_ALTITUDE = 1000.0 # Meters.
		self.POSITION = npa([INITIAL_RANGE, INITIAL_ALTITUDE]) # Meters.

		# INITIAL VELOCITY.
		INITIAL_HORIZ_SPEED = 130.0 # Meters per second.
		INITIAL_VERT_SPEED = 130.0 # Meters per second.

		# THIS IS THE ONLY TIME THETA IS ASSUMED TO BE ALIGNED WITH VELOCITY.
		self.THETA = returnEl(INITIAL_VERT_SPEED, INITIAL_HORIZ_SPEED) + 0.1 # Radians.
		self.THETA_DEG = np.degrees(self.THETA) # Degrees.
		self.THETA_DOT = 0.0 # Radians per second..
		self.THETA_DOT_DEG = np.degrees(self.THETA_DOT) # Degrees per second.

		# TRANSFORMATION MATRIX.
		self.BODY_TO_RANGE_AND_ALTITUDE = ct.BODY_TO_RANGE_AND_ALTITUDE(-self.THETA)

		# VELOCITY
		self.VELOCITY = npa([INITIAL_HORIZ_SPEED, INITIAL_VERT_SPEED]) # Meters per second.
		self.SPEED = la.norm(self.VELOCITY)

		# POSITIVE ALPHA MEANS NOSE BELOW FREE STREAM VELOCITY.
		self.BODY_VELOCITY = self.BODY_TO_RANGE_AND_ALTITUDE @ self.VELOCITY

		# ACCELERATION.
		self.ACCELERATION = npa([0.0, 0.0]) # Meters per second squared.
		self.SPECIFIC_FORCE = npa([0.0, 0.0]) # Meters per second squared.

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
		self.MASS = 45
		self.IYY = (self.MASS * (3 * ((0.5 * self.REFERENCE_DIAMETER) ** 2) + self.REFERENCE_LENGTH ** 2)) / (12) # Kilograms times meters squared.
		self.CG = self.NOSE_LENGTH # Meters.

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
		self.OMEGA_Z = np.sqrt((MA * ZD - MD * ZA) / ZD)
		self.OMEGA_AF = np.sqrt(-1 * MA)
		self.ZETA_AF = ZA * self.OMEGA_AF / (2 * MA)
		self.K1 = -1 * self.SPEED * ((MA * ZD - ZA * MD) / (1845 * MA))
		self.K2 = self.K1
		self.K3 = 1845 * self.K1 / self.SPEED
		self.TA = MD / (MA * ZD - ZA * MD)

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
			"T_MOI": self.IYY,
			"MASS": self.MASS,

			# AERODYNAMICS
			"W_Z": self.OMEGA_Z,
			"W_AF": self.OMEGA_AF,
			"Z_AF": self.ZETA_AF,
			"K1": self.K1,
			"K2": self.K2,
			"K3": self.K3,
			"TA": self.TA

		}

		### LOGGING ###
		self.LOG_FILE = open(f"MockHellfire_HighFidelityThreeDOF/output/data_{self.FLAG}.txt", "w")
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
			"T_MOI": self.IYY,
			"MASS": self.MASS,

			# AERODYNAMICS
			"W_Z": self.OMEGA_Z,
			"W_AF": self.OMEGA_AF,
			"Z_AF": self.ZETA_AF,
			"K1": self.K1,
			"K2": self.K2,
			"K3": self.K3,
			"TA": self.TA

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

			# Update dynamics.
			if self.FLAG == 0:
				
				# NORMAL COEFFICIENT AND PITCHING MOMENT COEFFICIENT CALCULATION, NON DIMENSIONAL.
				CN = 2 * self.ALPHA + (1.5 * self.PLANFORM_AREA * self.ALPHA * self.ALPHA) / self.REFERENCE_AREA + (8 * self.WING_AREA * self.ALPHA) / (self.BETA * self.REFERENCE_AREA) + (8 * self.TAIL_AREA * (self.ALPHA + self.FIN_DEFLECTION_RADIANS)) / (self.BETA * self.REFERENCE_AREA)
				CM = 2 * self.ALPHA * ((self.CG - self.NOSE_CENTER_OF_PRESSURE) / self.REFERENCE_DIAMETER) + ((1.5 * self.PLANFORM_AREA * self.ALPHA * self.ALPHA) / self.REFERENCE_AREA) * ((self.CG - self.BODY_CENTER_OF_PRESSURE) / self.REFERENCE_DIAMETER) + ((8 * self.WING_AREA * self.ALPHA) / (self.BETA * self.REFERENCE_AREA)) * ((self.CG - self.WING_CENTER_OF_PRESSURE) / self.REFERENCE_DIAMETER) + ((8 * self.TAIL_AREA * (self.ALPHA + self.FIN_DEFLECTION_RADIANS)) / (self.BETA * self.REFERENCE_AREA)) * ((self.CG - self.CENTER_OF_DEFLECTION_FROM_NOSE) / self.REFERENCE_DIAMETER)
				
				# DERIVATIVES.
				THETA_DOT_DOT = (self.Q * self.REFERENCE_AREA * self.REFERENCE_DIAMETER * CM) / self.IYY # Radians per second squared.

				NORMAL_ACCEL = (self.Q * self.REFERENCE_AREA * CN) / self.MASS # Meters per second squared.
				self.SPECIFIC_FORCE = npa([0.0, NORMAL_ACCEL])
				LOCAL_G = npa([0.0, -1.0 * self.G])
				BODY_G = self.BODY_TO_RANGE_AND_ALTITUDE @ LOCAL_G
				self.SPECIFIC_FORCE += BODY_G
				self.ACCELERATION = self.SPECIFIC_FORCE @ self.BODY_TO_RANGE_AND_ALTITUDE

				# INTEGRATION.
				DELTA_POS = self.VELOCITY * self.TIME_STEP
				self.POSITION += DELTA_POS

				DELTA_VEL = self.ACCELERATION * self.TIME_STEP
				self.VELOCITY += DELTA_VEL
				self.BODY_VELOCITY = self.BODY_TO_RANGE_AND_ALTITUDE @ self.VELOCITY

				self.THETA_DOT += THETA_DOT_DOT * self.TIME_STEP # Radians per second.
				self.THETA += self.THETA_DOT * self.TIME_STEP # Radians.
				self.THETA_DOT_DEG = np.degrees(self.THETA_DOT)
				self.THETA_DEG = np.degrees(self.THETA)

				self.ALPHA_DOT = self.THETA_DOT - (self.SPECIFIC_FORCE[1] / self.SPEED) # Radians per second.
				self.ALPHA += self.ALPHA_DOT * self.TIME_STEP # Radians.
				self.ALPHA_DOT_DEG = np.degrees(self.ALPHA_DOT)
				self.ALPHA_DEG = np.degrees(self.ALPHA)

				# RE ORIENT.
				self.BODY_TO_RANGE_AND_ALTITUDE = ct.BODY_TO_RANGE_AND_ALTITUDE(-self.THETA)

				# STORE DATA.
				self.populateDynamicsState()

				# LOG DATA.
				self.logDynamicsState()

				# UPDATE TIME OF FLIGHT.
				self.TIME_OF_FLIGHT += self.TIME_STEP

			# Console report.
			if round(self.TIME_OF_FLIGHT, 3).is_integer():
				print(f"{self.TIME_OF_FLIGHT:.2f} {self.POSITION}")

			# Performance and termination check.
			if self.TIME_OF_FLIGHT > self.MAX_TIME:
				GO = False
				print(f"MAX TIME - {self.TIME_OF_FLIGHT:.2f} {self.POSITION}")
			if self.POSITION[1] < 0:
				GO = False
				print(f"GROUND COLLISION - {self.TIME_OF_FLIGHT:.2f} {self.POSITION}")
			if np.isnan(np.sum(self.POSITION)):
				GO = False
				print(f"NAN - {self.TIME_OF_FLIGHT:.2f} {self.POSITION}")



if __name__ == "__main__":

	"""
	
	FLAG = 0 >>> SIMPLE LINEAR APPROXIMATIONS
	FLAG = 1 >>> FIXED COEFFICIENT APPROXIMATIONS
	FLAG = 2 >>> DIFFERENTIAL EQNS OF MOTION #1
	FLAG = 3 >>> DIFFERENTIAL EQNS OF MOTION #2
	
	"""

	x = AirframeSimulation(FLAG=0)
	x.dynamics()