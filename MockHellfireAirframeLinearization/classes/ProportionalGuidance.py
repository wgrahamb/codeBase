import numpy as np
from numpy import linalg as la

class ProportionalGuidance:

	def __init__(self):
		self.PROPORTIONAL_GAIN = 4
		self.GLIMIT = 12
		self.COMMAND = 0.0
		print("PROPORTIONAL GUIDANCE LOADED")

	def update(self, CLOSING_SPEED, MSL_TO_TARGET_RELPOS, LINE_OF_SIGHT_RATE):

		LINE_OF_SIGHT_ANGLE = np.arctan2(
			MSL_TO_TARGET_RELPOS[2],
			np.sqrt(MSL_TO_TARGET_RELPOS[1] * MSL_TO_TARGET_RELPOS[1] + MSL_TO_TARGET_RELPOS[0] * MSL_TO_TARGET_RELPOS[0])
		)
		self.COMMAND = -1.0 * self.PROPORTIONAL_GAIN * CLOSING_SPEED * LINE_OF_SIGHT_RATE[1]

		if np.abs(self.COMMAND) > self.GLIMIT:
			NEW_COMMAND = np.sign(self.COMMAND) * self.GLIMIT
			self.COMMAND = NEW_COMMAND