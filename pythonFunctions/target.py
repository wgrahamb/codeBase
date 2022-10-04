import numpy as np

class Target:

	def __init__(self, enuPos0, enuVel0):
		self.t_tof = 0.0
		self.TIME_STEP = None
		self.enuPos = enuPos0
		self.enuVel = enuVel0

		print("TARGET CONSTRUCTED")

	def update(self, DT):
		self.TIME_STEP = DT
		deltaPos = self.TIME_STEP * self.enuVel
		self.enuPos += deltaPos
		self.t_tof += self.TIME_STEP