import numpy as np
from utility.trapezoidIntegrate import integrate

class SecondOrderActuator:

	def __init__(self):

		### ACTUATOR ###
		self.TIME = 0.0 # Seconds.
		self.TIME_STEP = 1 / 1200.0 # Seconds.
		self.NEXT_UPDATE_TIME = self.TIME + self.TIME_STEP # Seconds.
		self.DEFL_LIMIT = 10 # Degrees.
		self.DEFL_RATE_LIMIT = 25 # Degrees per second.
		self.WNACT = np.degrees(10.0) # Degrees per second.
		self.ZETACT = 0.7 # Non dimensional.
		self.DEFLECTION = 0.0 # Degrees.
		self.DEFLECTION_DOT_DER = 0.0 # Degrees per second.
		self.DEFLECTION_DOT = 0.0 # Degrees per second.
		self.DEFLECTION_DOT_DOT = 0.0 #  # Degrees per second.squared.
		self.FLAG = 0

		print("SECOND ORDER ACTUATOR LOADED")

	def update(self, DEFLECTION_COMMAND):

		### ACTUATOR ###
		self.FLAG = 0
		if np.abs(self.DEFLECTION) > self.DEFL_LIMIT:
			self.DEFLECTION = self.DEFL_LIMIT * np.sign(self.DEFLECTION)
			if (self.DEFLECTION * self.DEFLECTION_DOT) > 0.0:
				self.DEFLECTION_DOT = 0.0
		if np.abs(self.DEFLECTION_DOT > self.DEFL_RATE_LIMIT):
			self.FLAG = 1
			self.DEFLECTION_DOT = self.DEFL_RATE_LIMIT * np.sign(self.DEFLECTION_DOT)
		DEFL_DOT_NEW = self.DEFLECTION_DOT
		self.DEFLECTION = integrate(DEFL_DOT_NEW, self.DEFLECTION_DOT_DER, self.DEFLECTION, self.TIME_STEP)
		self.DEFLECTION_DOT_DER = DEFL_DOT_NEW
		EDX = DEFLECTION_COMMAND - self.DEFLECTION
		DEFLDOTDOT_NEW = self.WNACT * self.WNACT * EDX - 2 * self.ZETACT * self.WNACT * self.DEFLECTION_DOT_DER
		self.DEFLECTION_DOT_DER = integrate(DEFLDOTDOT_NEW, self.DEFLECTION_DOT_DOT, self.DEFLECTION_DOT, self.TIME_STEP)
		self.DEFLECTION_DOT_DOT = DEFLDOTDOT_NEW
		if self.FLAG == 1 and (self.DEFLECTION_DOT_DER * self.DEFLECTION_DOT_DOT) > 0:
			self.DEFLECTION_DOT_DOT= 0
		self.TIME += self.TIME_STEP
		self.NEXT_UPDATE_TIME = self.TIME + self.TIME_STEP # Seconds.