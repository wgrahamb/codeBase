import numpy as np

class Target:

	def __init__(self, INITIAL_TARGET_POSITION, INITIAL_TARGET_VELOCITY):
		self.targetTimeOfFlight = 0.0
		self.TIME_STEP =(1 / 2400.0)
		self.NEXT_UPDATE_TIME = self.targetTimeOfFlight + self.TIME_STEP
		self.targetRightUpPosition = INITIAL_TARGET_POSITION
		self.targetRightUpVelocity = INITIAL_TARGET_VELOCITY

		print("TARGET LOADED")

	def update(self):
		deltaPos = self.TIME_STEP * self.targetRightUpVelocity
		self.targetRightUpPosition += deltaPos
		self.targetTimeOfFlight += self.TIME_STEP
		self.NEXT_UPDATE_TIME = self.targetTimeOfFlight + self.TIME_STEP