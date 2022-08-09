import numpy as np
from numpy import linalg as la

class ProportionalGuidance:

	def __init__(self):
		self.PROPORTIONAL_GAIN = 4
		self.GLIMIT = 5
		self.COMMAND = 0.0
		print("PROPORTIONAL GUIDANCE LOADED")

	def update(self, CLOSING_SPEED, MSL_TO_TARGET_RELPOS, LINE_OF_SIGHT_RATE):

		LINE_OF_SIGHT_ANGLE = np.arctan2(la.norm(MSL_TO_TARGET_RELPOS), MSL_TO_TARGET_RELPOS[2])
		self.COMMAND = -1.0 * self.PROPORTIONAL_GAIN * CLOSING_SPEED * LINE_OF_SIGHT_ANGLE * LINE_OF_SIGHT_RATE[1]

		if np.abs(self.COMMAND) > self.GLIMIT:
			NEW_COMMAND = np.sign(self.COMMAND) * self.GLIMIT
			self.COMMAND = NEW_COMMAND