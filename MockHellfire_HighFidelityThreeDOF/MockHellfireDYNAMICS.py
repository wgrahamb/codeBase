
# MATH CONSTANTS
MM_TO_M = 1.0 / 1000.0
RAD_TO_DEG = 57.2957795130823
DEG_TO_RAD = 1.0 / RAD_TO_DEG
EPSILON = 0.00000001

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
		INITIAL_MISSILE_RANGE,
		INITIAL_MISSILE_ALTITUDE,
		INITIAL_HRZ_VEL,
		INITIAL_VRT_VEL
	):

		self.LETHALITY = "FLYING"

		# THE INIT FOR THIS SIMULATION IS THE INPUT FOR THE UPDATE IN THE ABSTRACTED VERSION.
		self.FIN_DEFLECTION_DEGREES = None # Degrees.
		self.FIN_DEFLECTION_RADIANS = None # Radians.
		self.MAX_TIME = None # Seconds.
		self.TIME_STEP = None # Seconds. Variable.
		INITIAL_RANGE = INITIAL_MISSILE_RANGE # Meters.
		INITIAL_ALTITUDE = INITIAL_MISSILE_ALTITUDE # Meters.
		INITIAL_HORIZ_SPEED = INITIAL_HRZ_VEL # Meters per second.
		INITIAL_VERT_SPEED = INITIAL_VRT_VEL # Meters per second.

		### MOCK HELLFIRE PARAMS ###
		self.REFERENCE_DIAMETER = 0.18 # Meters.
		self.REFERENCE_AREA = np.pi * (self.REFERENCE_DIAMETER ** 2) / 4 # Meters squared.
		self.REFERENCE_LENGTH = 1.85026 # Meters.
		# self.REFERENCE_LENGTH = 1.85026 - 0.249733 # Meters.
		self.WING_HALF_SPAN = 66.1175 * MM_TO_M / 2.0 # Meters.
		self.WING_TIP_CHORD = 91.047 * MM_TO_M # Meters.
		self.WING_ROOT_CHORD = 0.123564 # Meters.
		self.TAIL_HALF_SPAN = 71.3548 * MM_TO_M / 2.0 # Meters.
		self.TAIL_TIP_CHORD = 0.387894 # Meters.
		self.TAIL_ROOT_CHORD = 0.48084 # Meters.
		self.NOSE_LENGTH = 0.249733 # Meters.
		self.DISTANCE_FROM_BASE_OF_NOSE_TO_WING = 0.323925 # Meters.
		self.CENTER_OF_DEFLECTION_FROM_NOSE = 1.8059 # Meters.
		# self.CENTER_OF_DEFLECTION_FROM_NOSE = 1.8059 - self.NOSE_LENGTH # Meters.
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

		# THIS IS THE ONLY TIME THETA IS ASSUMED TO BE ALIGNED WITH VELOCITY.
		self.THETA = returnEl(INITIAL_VERT_SPEED, INITIAL_HORIZ_SPEED) # Radians.
		self.THETA_DEG = np.degrees(self.THETA) # Degrees.
		self.THETA_DOT = 0.0 # Radians per second..
		self.THETA_DOT_DEG = np.degrees(self.THETA_DOT) # Degrees per second.

		# VERTICAL ANGLE OF ATTACK.
		self.ALPHA = 0.0
		self.ALPHA_DEG = np.degrees(self.ALPHA)

		# DIFFERENTIAL DYNAMICS.
		self.E = 0.0
		self.EDOT = 0.0
		self.EDOTDOT = 0.0

		# TRANSFORMATION MATRIX.
		self.BODY_TO_RANGE_AND_ALTITUDE = ct.BODY_TO_RANGE_AND_ALTITUDE(-self.THETA)

		# POSITIVE ALPHA MEANS NOSE BELOW FREE STREAM VELOCITY.
		self.BODY_VELOCITY = self.BODY_TO_RANGE_AND_ALTITUDE @ self.VELOCITY

		# CONTROL OUTPUT.
		self.KDC = 0.0

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
		self.CG = self.MASS_AND_MOTOR.XCG
		self.IYY = self.MASS_AND_MOTOR.TRANSVERSE_MOI
		self.MASS = self.MASS_AND_MOTOR.MASS
		self.THRUST = self.MASS_AND_MOTOR.THRUST

		self.DYNAMICS_STATE = {

			"LETHALITY": self.LETHALITY,

			# DYNAMICS
			"TOF": self.TIME_OF_FLIGHT,
			"RNG": self.POSITION[0],
			"ALT": self.POSITION[1],
			"RNG_VEL": self.VELOCITY[0],
			"ALT_VEL": self.VELOCITY[1],
			"U": self.BODY_VELOCITY[0],
			"W": self.BODY_VELOCITY[1],
			"SPD": self.SPEED,
			"RNG_ACC": self.ACCELERATION[0],
			"ALT_ACC": self.ACCELERATION[1],
			"U_DOT": self.SPECIFIC_FORCE[0],
			"W_DOT": self.SPECIFIC_FORCE[1],
			"THT": self.THETA,
			"THT_DEG": self.THETA_DEG,
			"THT_DOT": self.THETA_DOT,
			"THT_DOT_DEG": self.THETA_DOT_DEG,
			"AOA": self.ALPHA,
			"AOA_DEG": self.ALPHA_DEG,
			"E": self.E,
			"E_DOT": self.EDOT,
			"E_DOT_DOT": self.EDOTDOT,
			"KDC": self.KDC,

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
		self.E_0 = self.E
		self.EDOT_0 = self.EDOT
		self.THT_0 = self.THETA
		self.POS_0 = self.POSITION
		self.VEL_0 = self.VELOCITY

		# DERIVATIVES
		self.EDOTDOT_1 = 0.0
		self.EDOT_1 = 0.0
		self.THTDOT_1 = 0.0
		self.ACC_1 = np.zeros(2)
		self.VEL_1 = np.zeros(2)

		self.EDOTDOT_2 = 0.0
		self.EDOT_2 = 0.0
		self.THTDOT_2 = 0.0
		self.ACC_2 = np.zeros(2)
		self.VEL_2 = np.zeros(2)

		self.EDOTDOT_3 = 0.0
		self.EDOT_3 = 0.0
		self.THTDOT_3 = 0.0
		self.ACC_3 = np.zeros(2)
		self.VEL_3 = np.zeros(2)

		self.EDOTDOT_4 = 0.0
		self.EDOT_4 = 0.0
		self.THTDOT_4 = 0.0
		self.ACC_4 = np.zeros(2)
		self.VEL_4 = np.zeros(2)

		### LOGGING ###
		FILE_ID = f"MockHellfireDynamics"
		print(FILE_ID)
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

			"LETHALITY": self.LETHALITY,

			# DYNAMICS
			"TOF": self.TIME_OF_FLIGHT,
			"RNG": self.POSITION[0],
			"ALT": self.POSITION[1],
			"RNG_VEL": self.VELOCITY[0],
			"ALT_VEL": self.VELOCITY[1],
			"U": self.BODY_VELOCITY[0],
			"W": self.BODY_VELOCITY[1],
			"SPD": self.SPEED,
			"RNG_ACC": self.ACCELERATION[0],
			"ALT_ACC": self.ACCELERATION[1],
			"U_DOT": self.SPECIFIC_FORCE[0],
			"W_DOT": self.SPECIFIC_FORCE[1],
			"THT": self.THETA,
			"THT_DEG": self.THETA_DEG,
			"THT_DOT": self.THETA_DOT,
			"THT_DOT_DEG": self.THETA_DOT_DEG,
			"AOA": self.ALPHA,
			"AOA_DEG": self.ALPHA_DEG,
			"E": self.E,
			"E_DOT": self.EDOT,
			"E_DOT_DOT": self.EDOTDOT,
			"KDC": self.KDC,

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

	def dynamics(self, FLY_FOR_THIS_LONG, FIN_DEFL_DEG):

		self.FIN_DEFLECTION_DEGREES = FIN_DEFL_DEG
		self.FIN_DEFLECTION_RADIANS = np.radians(FIN_DEFL_DEG)
		self.MAX_TIME = self.TIME_OF_FLIGHT + FLY_FOR_THIS_LONG
		self.TIME_STEP = FLY_FOR_THIS_LONG
		DT_LIM = (1.0 / 100.0)
		if self.TIME_STEP > DT_LIM:
			self.TIME_STEP = DT_LIM

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
			self.CG = self.MASS_AND_MOTOR.XCG
			self.IYY = self.MASS_AND_MOTOR.TRANSVERSE_MOI
			self.MASS = self.MASS_AND_MOTOR.MASS
			self.THRUST = self.MASS_AND_MOTOR.THRUST

			# AERODYNAMICS.
			TEMP1 = (self.CG - self.WING_CENTER_OF_PRESSURE) / self.REFERENCE_DIAMETER 
			TEMP2 = (self.CG - self.CENTER_OF_DEFLECTION_FROM_NOSE) / self.REFERENCE_DIAMETER
			TEMP3 = (self.CG - self.BODY_CENTER_OF_PRESSURE) / self.REFERENCE_DIAMETER
			TEMP4 = (self.CG - self.NOSE_CENTER_OF_PRESSURE) / self.REFERENCE_DIAMETER

			Y1 = 2 * TEMP4 + 8 * self.WING_AREA * TEMP1 / (self.BETA * self.REFERENCE_AREA) + 8 * self.TAIL_AREA * TEMP2 / (self.BETA * self.REFERENCE_AREA)
			Y2 = 1.5 * self.PLANFORM_AREA * TEMP3 / self.REFERENCE_AREA
			Y3 = 8 * self.TAIL_AREA * TEMP2 * self.FIN_DEFLECTION_RADIANS / (self.BETA * self.REFERENCE_AREA)
			ALPHA_TRIM = (-Y1 - np.sqrt((Y1 ** 2) - 4 * Y2 * Y3)) / (2 * Y2)

			# DELTA_TRIM = (-1 * Y4 * ALPHA_TRIM - Y5 * ALPHA_TRIM * ALPHA_TRIM) / Y6
			CNA = 2 + 1.5 * self.PLANFORM_AREA * ALPHA_TRIM / self.REFERENCE_AREA + 8 * self.WING_AREA / (self.BETA * self.REFERENCE_AREA) + 8 * self.TAIL_AREA / (self.BETA * self.REFERENCE_AREA)
			CND = 8 * self.TAIL_AREA / (self.BETA * self.REFERENCE_AREA)
			ZA = -1 * self.Q * self.REFERENCE_AREA * CNA / (self.MASS * self.SPEED)
			ZD = -1 * self.Q * self.REFERENCE_AREA * CND / (self.MASS * self.SPEED)
			CMAP = 2 * TEMP4 + 1.5 * self.PLANFORM_AREA * ALPHA_TRIM * TEMP3 / self.REFERENCE_AREA + 8 * self.WING_AREA * TEMP1 / (self.BETA * self.REFERENCE_AREA)
			CMA = CMAP + 8 * self.TAIL_AREA * TEMP2 / (self.BETA * self.REFERENCE_AREA)
			CMD = 8 * self.TAIL_AREA * TEMP2 / (self.BETA * self.REFERENCE_AREA)
			MA = self.Q * self.REFERENCE_AREA * self.REFERENCE_DIAMETER * CMA / self.IYY
			MD = self.Q * self.REFERENCE_AREA * self.REFERENCE_DIAMETER * CMD / self.IYY

			OMEGA_Z = np.sqrt((MA * ZD - MD * ZA) / ZD) # Per second.
			OMEGA_AF = np.sqrt(-1 * MA) # Per second.
			ZETA_AF = ZA * OMEGA_AF / (2 * MA) # Non dimensional damping.
			# KR = 0.1 # Gain.
			K1 = -1 * self.SPEED * ((MA * ZD - ZA * MD) / (1845 * MA)) # Gain.
			TA = MD / (MA * ZD - MD * ZA) # Time constant. Second.
			K3 = 1845 * K1 / self.SPEED
			self.KDC = (1 - 0.1 * K3) / (K1 * 0.1)

			# DERIVATIVES.
			self.EDOTDOT = (OMEGA_AF ** 2) * (self.FIN_DEFLECTION_DEGREES - self.E - 2 * ZETA_AF * self.EDOT / OMEGA_AF) # DERIVATIVE

			# THETA DOT.
			ETERM = K3 * (self.E + TA * self.EDOT)
			self.THETA_DOT_DEG = -ETERM# DERIVATIVE
			self.THETA_DOT = np.radians(self.THETA_DOT_DEG)

			# KLUDGE DRAG.
			CD = 0.1
			D_FORCE = CD * self.Q * self.REFERENCE_AREA
			DX = D_FORCE * np.cos(self.ALPHA)

			# NORMAL ACCELERATION.
			NORMAL_ACCEL = K1 * (self.E - (self.EDOTDOT / (OMEGA_Z ** 2))) * self.G # NEEDS TO BE M/S^2
			self.SPECIFIC_FORCE = npa([
				(self.THRUST / self.MASS) - (DX / self.MASS),
				(NORMAL_ACCEL)
			]) # NEEDS TO BE M/S^2
			LOCAL_G = npa([0.0, -1.0 * self.G])
			BODY_G = self.BODY_TO_RANGE_AND_ALTITUDE @ LOCAL_G
			self.SPECIFIC_FORCE += BODY_G
			self.ACCELERATION = self.SPECIFIC_FORCE @ self.BODY_TO_RANGE_AND_ALTITUDE # DERIVATIVE

			# INTEGRATION.
			if self.INTEGRATION_PASS == 0:

				# STATES.
				self.E_0 = self.E
				self.EDOT_0 = self.EDOT
				self.THT_0 = self.THETA
				self.POS_0 = self.POSITION
				self.VEL_0 = self.VELOCITY

				# DERIVATIVES.
				self.EDOT_1 = self.EDOT
				self.EDOTDOT_1 = self.EDOTDOT
				self.THTDOT_1 = self.THETA_DOT
				self.VEL_1 = self.VELOCITY
				self.ACC_1 = self.ACCELERATION

				# INTEGRATION.
				self.E = self.E_0 + self.EDOT_1 * (self.TIME_STEP / 2.0)
				self.EDOT = self.EDOT_0 + self.EDOTDOT_1 * (self.TIME_STEP / 2.0)

				self.THETA = self.THT_0 + self.THTDOT_1 * (self.TIME_STEP / 2.0)
				self.THETA_DEG = np.degrees(self.THETA)

				self.POSITION = self.POS_0 + self.VEL_1 * (self.TIME_STEP / 2.0)

				self.VELOCITY = self.VEL_0 + self.ACC_1 * (self.TIME_STEP / 2.0)

				# ITERATE INTEGRATION.
				self.INTEGRATION_PASS += 1

				# UPDATE TIME OF FLIGHT.
				self.TIME_OF_FLIGHT += (self.TIME_STEP / 2.0)

			elif self.INTEGRATION_PASS == 1:

				# DERIVATIVES.
				self.EDOT_2 = self.EDOT
				self.EDOTDOT_2 = self.EDOTDOT
				self.THTDOT_2 = self.THETA_DOT
				self.VEL_2 = self.VELOCITY
				self.ACC_2 = self.ACCELERATION

				# INTEGRATION.
				self.E = self.E_0 + self.EDOT_2 * (self.TIME_STEP / 2.0)
				self.EDOT = self.EDOT_0 + self.EDOTDOT_2 * (self.TIME_STEP / 2.0)

				self.THETA = self.THT_0 + self.THTDOT_2 * (self.TIME_STEP / 2.0)
				self.THETA_DEG = np.degrees(self.THETA)

				self.POSITION = self.POS_0 + self.VEL_2 * (self.TIME_STEP / 2.0)

				self.VELOCITY = self.VEL_0 + self.ACC_2 * (self.TIME_STEP / 2.0)

				# ITERATE INTEGRATION.
				self.INTEGRATION_PASS += 1

			elif self.INTEGRATION_PASS == 2:

				# DERIVATIVES.
				self.EDOT_3 = self.EDOT
				self.EDOTDOT_3 = self.EDOTDOT
				self.THTDOT_3 = self.THETA_DOT
				self.VEL_3 = self.VELOCITY
				self.ACC_3 = self.ACCELERATION

				# INTEGRATION.
				self.E = self.E_0 + self.EDOT_3 * (self.TIME_STEP)
				self.EDOT = self.EDOT_0 + self.EDOTDOT_3 * (self.TIME_STEP)

				self.THETA = self.THT_0 + self.THTDOT_3 * (self.TIME_STEP)
				self.THETA_DEG = np.degrees(self.THETA)

				self.POSITION = self.POS_0 + self.VEL_3 * (self.TIME_STEP)

				self.VELOCITY = self.VEL_0 + self.ACC_3 * (self.TIME_STEP)

				# ITERATE INTEGRATION.
				self.INTEGRATION_PASS += 1

				# UPDATE TIME OF FLIGHT.
				self.TIME_OF_FLIGHT += (self.TIME_STEP / 2.0)

			else:

				# DERIVATIVES.
				self.EDOT_4 = self.EDOT
				self.EDOTDOT_4 = self.EDOTDOT
				self.THTDOT_4 = self.THETA_DOT
				self.VEL_4 = self.VELOCITY
				self.ACC_4 = self.ACCELERATION

				# INTEGRATION.
				self.E = self.E_0 + (self.TIME_STEP / 6.0) * (self.EDOT_1 + 2 * self.EDOT_2 + 2 * self.EDOT_3 + self.EDOT_4)
				self.EDOT = self.EDOT_0 + (self.TIME_STEP / 6.0) * (self.EDOTDOT_1 + 2 * self.EDOTDOT_2 + 2 * self.EDOTDOT_3 + self.EDOTDOT_4)

				self.THETA = self.THT_0 + (self.TIME_STEP / 6.0) * (self.THTDOT_1 + 2 * self.THTDOT_2 + 2 * self.THTDOT_3 + self.THTDOT_4)
				self.THETA_DEG = np.degrees(self.THETA)

				self.POSITION = self.POS_0 + (self.TIME_STEP / 6.0) * (self.VEL_1 + 2 * self.VEL_2 + 2 * self.VEL_3 + self.VEL_4)

				self.VELOCITY = self.VEL_0 + (self.TIME_STEP / 6.0) * (self.ACC_1 + 2 * self.ACC_2 + 2 * self.ACC_3 + self.ACC_4)

				# ITERATE INTEGRATION.
				self.INTEGRATION_PASS = 0

			# RE ORIENT.
			self.BODY_TO_RANGE_AND_ALTITUDE = ct.BODY_TO_RANGE_AND_ALTITUDE(-self.THETA)
			self.BODY_VELOCITY = self.BODY_TO_RANGE_AND_ALTITUDE @ self.VELOCITY
			self.SPEED = la.norm(self.BODY_VELOCITY)
			self.ALPHA = -1.0 * returnEl(self.BODY_VELOCITY[1], self.BODY_VELOCITY[0])
			self.ALPHA_DEG = np.degrees(self.ALPHA)

			# LOG DATA.
			if self.INTEGRATION_PASS == 0:
				self.logDynamicsState()

			# Console report.
			if round(self.TIME_OF_FLIGHT, 4).is_integer():
				if self.INTEGRATION_PASS == 0:
					print(f"TOF {self.TIME_OF_FLIGHT:.2f} POS {self.POSITION} MACH {self.MACH:.2f}")

			# Performance and termination check.
			if self.INTEGRATION_PASS == 0:
				if self.TIME_OF_FLIGHT >= self.MAX_TIME:
					GO = False
					self.LETHALITY = "MAX_TIME"
					# print(f"MAX TIME - {self.TIME_OF_FLIGHT:.8f} POS {self.POSITION} MACH {self.MACH:.2f}\n")
				if self.POSITION[1] < 0:
					GO = False
					self.LETHALITY = "GROUND_COLLISION"
					# print(f"GROUND COLLISION - {self.TIME_OF_FLIGHT:.8f} POS {self.POSITION} MACH {self.MACH:.2f}\n")
				if np.isnan(np.sum(self.POSITION)):
					GO = False
					self.LETHALITY = "NAN"
					# print(f"NAN - {self.TIME_OF_FLIGHT:.8f} POS {self.POSITION} MACH {self.MACH:.2f}\n")
			
			# STORE DATA.
			self.populateDynamicsState()



