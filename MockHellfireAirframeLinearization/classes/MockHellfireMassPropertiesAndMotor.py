import numpy as np
from utility.interpolationGivenTwoVectors import linearInterpolation

class MockHellfireMassPropertiesAndMotor:

	def __init__(self):

		self.STANDARD_GRAVITY = 9.81 # Meters per second squared.
		self.REFERENCE_DIAMETER = 0.1778 # Meters. Hellfire.
		self.REFERENCE_LENGTH = 1.85026 # Meters.
		self.SEA_LEVEL_PRESSURE = 101325 # Pascals.
		self.NOZZLE_EXIT_AREA = 0.004 # Meters squared. Hellfire.
		STARTING_CG_FROM_NOSE =  0.644605 # Meters.
		LAST_CG_FROM_NOSE = 0.249733 # Meters.

		self.BURN_TIME = 3.1 # Seconds.
		self.ISP = 325 # Seconds. Adjusted to represent a hellfire.
		self.INITIAL_TOTAL_MASS = 45 # Kilograms.
		self.FINAL_TOTAL_MASS = 20 # Kilograms.
		self.MASS = self.INITIAL_TOTAL_MASS # Kilograms.
		self.EXIT_VEL = self.ISP * self.STANDARD_GRAVITY # Meters per second.
		self.DELTA_V = np.log(self.INITIAL_TOTAL_MASS / self.FINAL_TOTAL_MASS) * self.EXIT_VEL # Meters per second.
		self.MDOT = (self.INITIAL_TOTAL_MASS - self.FINAL_TOTAL_MASS) / self.BURN_TIME # Kilograms per second.
		self.THRUST = 0.0
		self.XCG = STARTING_CG_FROM_NOSE
		self.FLAG = 0
		self.TRANSVERSE_MOI = (self.MASS * (3 * ((0.5 * self.REFERENCE_DIAMETER) ** 2) + self.REFERENCE_LENGTH ** 2)) / (12) # Kilograms times meters squared.
		self.THRUST_TIME_VALUES = np.linspace(0.0, self.BURN_TIME, 100)
		self.THRUST_VALUES = np.linspace(4000.0, 6000.0, 100)
		self.CG_TIME_VALUES = np.linspace(0.0, self.BURN_TIME, 100)
		self.CG_VALUES = np.linspace(STARTING_CG_FROM_NOSE, LAST_CG_FROM_NOSE, 100)

		print("MOCK HELLFIRE MASS AND MOTOR PROPERTIES LOADED")

	def update(self, timeOfFlight, pressure):
		
		if self.FLAG == 0:
			fuelUsed = self.MDOT * timeOfFlight
			self.MASS = (self.INITIAL_TOTAL_MASS - fuelUsed)
		if self.MASS > self.FINAL_TOTAL_MASS:
			thrust = linearInterpolation(timeOfFlight, self.THRUST_TIME_VALUES, self.THRUST_VALUES)
			self.THRUST = thrust + (pressure - self.SEA_LEVEL_PRESSURE) * self.NOZZLE_EXIT_AREA
			self.XCG = linearInterpolation(timeOfFlight, self.CG_TIME_VALUES, self.CG_VALUES)
			self.TRANSVERSE_MOI = (self.MASS * (3 * ((0.5 * self.REFERENCE_DIAMETER) ** 2) + self.REFERENCE_LENGTH ** 2)) / (12) # Kilograms times meters squared.
		else:
			if self.FLAG == 0:
				self.FLAG = 1
				self.THRUST = 0.0
				self.XCG = self.CG_VALUES[-1]
				self.TRANSVERSE_MOI = (self.MASS * (3 * ((0.5 * self.REFERENCE_DIAMETER) ** 2) + self.REFERENCE_LENGTH ** 2)) / (12) # Kilograms times meters squared.