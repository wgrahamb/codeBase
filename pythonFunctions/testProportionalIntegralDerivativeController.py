import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

class pidTest:

	def __init__(self):
		self.ku = 1.1
		self.tu = 22 - 7
		# NO OVERSHOOT METHOD
		self.kp = 0.2 * self.ku
		self.ki = 0.4 * self.ku / self.tu
		self.kd = 0.06666 * self.ku * self.tu
		
		self.error = 0.0
		self.intError = 0.0
		self.derError = 0.0
		self.lastError = 0.0
		self.maxThrust = 20
		self.minThrust = -20
		self.timeStep = 1
		self.go = True
		self.timeOfFlight = 0.0
		self.alt = 0.0
		self.targetAlt = 50.0
		self.vel = 0.0
		self.gravity = -9.81
		self.mass = 1
		self.data = []

	def physics(self, thrust):
		acc = self.gravity + thrust / self.mass
		self.vel += (acc * self.timeStep)
		self.alt += (self.vel * self.timeStep)
		self.timeOfFlight += self.timeStep

	def control(self):
		self.error = self.targetAlt - self.alt
		self.intError += (self.error * self.timeStep)
		self.derError = (self.error - self.lastError) / self.timeStep
		self.lastError = self.error
		controlledThrust = self.kp * self.error + self.ki * self.intError + self.kd * self.derError
		if controlledThrust > self.maxThrust:
			controlledThrust = self.maxThrust
		elif controlledThrust < self.minThrust:
			controlledThrust = self.minThrust
		return controlledThrust
	
	def fly(self):
		while self.timeOfFlight < 100:
			thrust = self.control()
			self.physics(thrust)
			dataPoint = np.array([self.timeOfFlight, self.alt, self.vel, thrust])
			self.data.append(dataPoint)
		dataFrame = pd.DataFrame(self.data, columns=["TOF", "ALT", "VEL", "THRUST"])
		plt.plot(dataFrame.iloc[:, 0], dataFrame.iloc[:, 1])
		plt.show()

def main():
	x = pidTest()
	x.fly()



if __name__ == "__main__":
	main()