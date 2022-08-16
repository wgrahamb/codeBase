import numpy as np

class Target:

	def __init__(self, initialTargetRightUpPosition, initialTargetRightUpVelocity, TIME_STEP):
		self.targetTimeOfFlight = 0.0
		self.TIME_STEP = TIME_STEP
		self.targetRightUpPosition = initialTargetRightUpPosition
		self.targetRightUpVelocity = initialTargetRightUpVelocity

		print("TARGET CONSTRUCTED")

	def update(self):
		deltaPos = self.TIME_STEP * self.targetRightUpVelocity
		self.targetRightUpPosition += deltaPos
		self.targetTimeOfFlight += self.TIME_STEP