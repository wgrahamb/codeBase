
from utility.returnAzAndElevation import returnEl

class MockHellfireNavGuidanceCtrl:

	def __init__(self):
		self.TIME = 0.0
		self.TIME_STEP = (1.0 /600.0)
		self.NEXT_UPDATE_TIME = self.TIME + self.TIME_STEP
		self.ONBOARD_STATE = None
		self.PROPORTIONAL_GAIN = 2.5
		self.DEFLECTION_COMMAND = 0.0

	def update(
		self,
		DYNAMICS_TRUTH_STATE,
		LINE_OF_SIGHT_RATE,
		CLOSING_SPEED
	):

		# Navigation.
		# This is the only time DYNAMICS_TRUTH_STATE is used in this module.
		self.ONBOARD_STATE = DYNAMICS_TRUTH_STATE

		# Guidance.
		COMMAND = LINE_OF_SIGHT_RATE * self.PROPORTIONAL_GAIN * CLOSING_SPEED

		# Control.
		# deflection = KR * (KDC * accelCommand + THD)
		self.DEFLECTION_COMMAND = 0.1 * (self.ONBOARD_STATE["KDC"] * (COMMAND / 9.81) + self.ONBOARD_STATE["THT_DOT_DEG"])

		self.TIME += self.TIME_STEP
		self.NEXT_UPDATE_TIME = self.TIME + self.TIME_STEP


