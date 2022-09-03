import numpy as np
from utility.trapezoidIntegrate import integrate
import utility.loggingFxns as lf

class SecondOrderActuator:

	def __init__(self, ID):

		### ACTUATOR ###
		self.TIME = 0.0 # Seconds.
		self.TIME_STEP = 1.0 / 300.0 # Seconds.
		self.NEXT_UPDATE_TIME = self.TIME + self.TIME_STEP # Seconds.
		self.DEFL_RATE_LIMIT = 25 # Degrees per second.
		self.DEFL_LIMIT = 10 # Degrees.
		self.WNACT = np.degrees(1.0) # Degrees per second.
		self.ZETACT = 0.8 # Non dimensional.
		self.DEFLECTION = 0.0 # Degrees.
		self.DEFLECTION_DER = 0.0 # Degrees per second.
		self.DEFLECTION_DOT = 0.0 # Degrees per second.
		self.DEFLECTION_DOT_DER = 0.0 #  # Degrees per second.squared.

		self.LOGFILE = open(f"PY_5DOF_HIGH_FIDELITY_MOCK_HELLFIRE/output/{ID}.txt", "w")

		self.STATE = {
			"TIME": self.TIME,
			"COMMAND": 0.0,
			"DEFL": self.DEFLECTION
		}

		lf.writeHeader(self.STATE, self.LOGFILE)
		lf.writeData(self.STATE, self.LOGFILE)

		print("SECOND ORDER ACTUATOR LOADED")

	def update(self, DEFLECTION_COMMAND):

		### ACTUATOR ###

		DEFLECTION_DER_NEW = self.DEFLECTION_DOT
		if DEFLECTION_DER_NEW > self.DEFL_RATE_LIMIT:
			DEFLECTION_DER_NEW = self.DEFL_RATE_LIMIT

		if self.DEFLECTION > self.DEFL_LIMIT:
			self.DEFLECTION = self.DEFL_LIMIT
		else:
			self.DEFLECTION = integrate(DEFLECTION_DER_NEW, self.DEFLECTION_DER, self.DEFLECTION, self.TIME_STEP)

		self.DEFLECTION_DER = DEFLECTION_DER_NEW
		EDX = DEFLECTION_COMMAND - self.DEFLECTION
		DEFLECTION_DOT_DER_NEW = self.WNACT * self.WNACT * EDX - 2 * self.ZETACT * self.WNACT * self.DEFLECTION_DER
		self.DEFLECTION_DOT = integrate(DEFLECTION_DOT_DER_NEW, self.DEFLECTION_DOT_DER, self.DEFLECTION_DOT, self.TIME_STEP)
		self.DEFLECTION_DOT_DER = DEFLECTION_DOT_DER_NEW

		self.TIME += self.TIME_STEP
		self.NEXT_UPDATE_TIME = self.TIME + self.TIME_STEP # Seconds.

		self.STATE = {
			"TIME": self.TIME,
			"COMMAND": DEFLECTION_COMMAND,
			"DEFL": self.DEFLECTION
		}

		lf.writeData(self.STATE, self.LOGFILE)