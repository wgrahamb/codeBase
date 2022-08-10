import numpy as np
from numpy import linalg as la

class MockHellfireAerodynamics:

	def __init__(self):

		MM_TO_M = 1.0 / 1000.0
		self.STANDARD_GRAVITY = 9.81 # Meters per second squared.
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

		self.OMEGA_Z = None
		self.OMEGA_AF = None
		self.ZETA_AF = None
		self.KR = None
		self.K1 = None
		self.TA = None
		self.K3 = None

		print("MOCK HELLFIRE AERODYNAMICS LOADED")

	def update(self, VELOCITY, XCG, MACHSPEED, MASS, COMMAND, DYNAMIC_PRESSURE, TRANSVERSE_MOMENT_OF_INERTIA):

		VELMAG = la.norm(VELOCITY)
		TEMP1 = (XCG - self.WING_CENTER_OF_PRESSURE) / self.REFERENCE_DIAMETER 
		TEMP2 = (XCG - self.CENTER_OF_DEFLECTION_FROM_NOSE) / self.REFERENCE_DIAMETER
		TEMP3 = (XCG - self.BODY_CENTER_OF_PRESSURE) / self.REFERENCE_DIAMETER
		TEMP4 = (XCG - self.NOSE_CENTER_OF_PRESSURE) / self.REFERENCE_DIAMETER
		BETA = None
		if MACHSPEED > 1:
			BETA = np.sqrt(MACHSPEED ** 2 - 1) # Non dimensional.
		else:
			BETA = MACHSPEED / 10 # Non dimensional.
		CNTRIM = MASS * COMMAND / (DYNAMIC_PRESSURE * self.REFERENCE_AREA)
		Y1 = 2 + 8 * self.WING_AREA / (BETA * self.REFERENCE_AREA) + 8 * self.TAIL_AREA / (BETA * self.REFERENCE_AREA)
		Y2 = 1.5 * self.PLANFORM_AREA / self.REFERENCE_AREA
		Y3 = 8 * self.TAIL_AREA / (BETA * self.REFERENCE_AREA)
		Y4 = 2 * TEMP4 + 8 * self.WING_AREA * TEMP1 / (BETA * self.REFERENCE_AREA) + 8 * self.TAIL_AREA * TEMP2 / (BETA * self.REFERENCE_AREA)
		Y5 = 1.5 * self.PLANFORM_AREA * TEMP3 / self.REFERENCE_AREA
		Y6 = 8 * self.TAIL_AREA * TEMP2 / (BETA * self.REFERENCE_AREA)
		P2 = Y2 - (Y3 * Y5) / Y6
		P3 = Y1 - (Y3 * Y4) / Y6
		ALPHA_TRIM = (-1 * P3 + np.sqrt(P3 * P3 + 4 * P2 * CNTRIM)) / (2 * P2)
		DELTA_TRIM = (-1 * Y4 * ALPHA_TRIM - Y5 * ALPHA_TRIM * ALPHA_TRIM) / Y6
		CNA = 2 + 1.5 * self.PLANFORM_AREA * ALPHA_TRIM / self.REFERENCE_AREA + 8 * self.WING_AREA / (BETA * self.REFERENCE_AREA) + 8 * self.TAIL_AREA / (BETA * self.REFERENCE_AREA)
		CND = 8 * self.TAIL_AREA / (BETA * self.REFERENCE_AREA)
		
		
		# ZA = -1 * self.STANDARD_GRAVITY * DYNAMIC_PRESSURE * self.REFERENCE_AREA * CNA / (MASS * VELMAG)
		ZA = -1 * DYNAMIC_PRESSURE * self.REFERENCE_AREA * CNA / (MASS * VELMAG)
		
		"""
		
		ZA & ZD

		# With gravity.
		# ((m/s^2) * (Pascals) * (m^2)) / (kg * m/s^2)
		# ((m/s^2) * (kg/(m*s^2)) * (m^2)) / (kg * m/s^2)

		# Without gravity.
		# ((Pascals) * (m^2)) / (kg * m/s)
		# ((kg/(m*s^2)) * (m^2)) / (kg * m/s)
		# kg           m^2           1                       kg * m * m * s                        1
		# m*s^2     1                kg * m/s            m * s * s * kg * m                   s

		"""

		# ZD = -1 * self.STANDARD_GRAVITY * DYNAMIC_PRESSURE * self.REFERENCE_AREA * CND / (MASS * VELMAG)
		ZD = -1 * DYNAMIC_PRESSURE * self.REFERENCE_AREA * CND / (MASS * VELMAG)


		CMAP = 2 * TEMP4 + 1.5 * self.PLANFORM_AREA * ALPHA_TRIM * TEMP3 / self.REFERENCE_AREA + 8 * self.WING_AREA * TEMP1 / (BETA * self.REFERENCE_AREA)
		CMA = CMAP + 8 * self.TAIL_AREA * TEMP2 / (BETA * self.REFERENCE_AREA)
		CMD = 8 * self.TAIL_AREA * TEMP2 / (BETA * self.REFERENCE_AREA)


		MA = DYNAMIC_PRESSURE * self.REFERENCE_AREA * self.REFERENCE_DIAMETER * CMA / TRANSVERSE_MOMENT_OF_INERTIA
		
		"""
		
		MA & MD

		Pascals * m^2 * m
		kg * m2

		kg * m^2 * m                            1
		m * s^2 * kg * m^2                    s^2

		"""
		
		MD = DYNAMIC_PRESSURE * self.REFERENCE_AREA * self.REFERENCE_DIAMETER * CMD / TRANSVERSE_MOMENT_OF_INERTIA


		self.OMEGA_Z = np.sqrt((MA * ZD - MD * ZA) / ZD)
		self.OMEGA_AF = np.sqrt(-1 * MA)
		self.ZETA_AF = ZA * self.OMEGA_AF / (2 * MA)
		self.KR = 0.1
		self.K1 = -1 * VELMAG * ((MA * ZD - ZA * MD) / (1845 * MA))
		self.TA = MD / (MA * ZD - MD * ZA)
		self.K3 = 1845 * self.K1 / VELMAG