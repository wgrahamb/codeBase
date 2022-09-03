from utility.interpolationGivenTwoVectors import linearInterpolation

class SphericalNoseDrag:

	def __init__(self):
		
		self.machValues = [
			0.0,
			0.8,
			0.9,
			1.0,
			1.1,
			1.2,
			1.3,
			1.4,
			1.5,
			1.6,
			1.65,
			5
		]
		self.dragCoefficientValues = [
			0.1,
			0.27,
			0.275,
			0.4,
			0.42,
			0.475,
			0.5,
			0.525,
			0.55,
			0.56,
			0.575,
			0.75
		]

	def getMachValues(self):
		return self.machValues

	def getCDValues(self):
		return self.dragCoefficientValues

	def update(self, mach):
		return linearInterpolation(mach, self.machValues, self.dragCoefficientValues)