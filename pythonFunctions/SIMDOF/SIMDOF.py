"""

SIMDOF

"""

class assetBaseClass:

	def __init__(self, identity, classType, frequency):
		self.id = identity
		self.classType = classType
		self.timer = 0.0
		self.timeStep = 1 / frequency
		self.startTime = None
		self.endTime = None
		self.loopCount = 0
		self.lastFunctionOutput = 0.0
		self.currentFunctionOutput = 0.0
		self.classDataStorage = open(f"pythonFunctions/SIMDOF/output/{self.id}.txt", "w")
		self.classDataStorage.write(f"TIMER START_TIME END_TIME VALUE\n")
		print(self.id, f"CONSTRUCTED")

	def update(self):
		self.startTime = self.timer
		self.loopCount += 1
		self.timer += self.timeStep
		newValue = self.rk4Integration(self.timer, self.currentFunctionOutput, self.timeStep)
		self.lastFunctionOutput = self.currentFunctionOutput
		self.currentFunctionOutput = newValue
		self.endTime = self.timer

	def report(self):
		self.classDataStorage.write(f"{self.timer} {self.startTime} {self.endTime} {self.currentFunctionOutput}\n")

	@staticmethod # AVERAGE INTEGRATION
	def averageIntegration(self, newDerivative, oldDerivative, value, step):
		return value + (newDerivative + oldDerivative) * step / 2

	@staticmethod # RK4 INTEGRATION METHOD
	def rk4Integration(currentTime, currentValue, timeStep):
		K1 = timeStep * assetBaseClass.function(
			currentTime,
			currentValue
		)
		K2 = timeStep * assetBaseClass.function(
			currentTime + (timeStep / 2),
			currentValue + (K1 / 2)
		)
		K3 = timeStep * assetBaseClass.function(
			currentTime + (timeStep / 2),
			currentValue + (K2 / 2)
		)
		K4 = timeStep * assetBaseClass.function(
			currentTime + timeStep,
			currentValue + K3
		)
		newValue = currentValue + ((K1 + (2 * K2) + (2 * K3) + K4) / 6)
		return newValue

	@staticmethod # BASIC MATHEMATICAL FUNCTION FOR MODELING
	def function(x, y):
		return x + y**2

class watchTowerBaseClass:

	def __init__(self, identity, classType, frequency, assets):
		self.id = identity
		self.type = classType
		self.timer = 0.0
		self.timeStep = 1 / frequency
		self.startTime = None
		self.endTime = None
		self.loopCount = 0
		self.assets = assets
		
		self.classDataStorage = open(f"pythonFunctions/SIMDOF/output/{self.id}.txt", "w")
		self.classDataStorage.write(f"TIMER START_TIME END_TIME\n")
		print(self.id, f"CONSTRUCTED")

	def update(self):
		self.startTime = self.timer
		self.loopCount += 1
		self.timer += self.timeStep

		for index, asset in enumerate(self.assets):
			while asset.timer < self.timer:
				asset.update()
				asset.report()

		self.endTime = self.timer

	def report(self):
		self.classDataStorage.write(f"{self.timer} {self.startTime} {self.endTime}\n")

if __name__ == "__main__":
	
	assetOne = assetBaseClass("ASSET_ONE", "ASSET", 100.0)
	assetTwo = assetBaseClass("ASSET_TWO", "ASSET", 200.0)
	
	nestedAssetOne = assetBaseClass("NESTED_ASSET_ONE", "ASSET", 300.0)
	nestedAssetTwo = assetBaseClass("NESTED_ASSET_TWO", "ASSET", 400.0)
	nestedAssetThree = assetBaseClass("NESTED_ASSET_THREE", "ASSET", 500.0)
	nestedAssets = [nestedAssetOne, nestedAssetTwo, nestedAssetThree]
	nestedWatchTower = watchTowerBaseClass("NESTED_WATCH_TOWER", "WATCH_TOWER", 100.0, nestedAssets)

	assetThree = assetBaseClass("ASSET_THREE", "ASSET", 600.0)
	assetFour = assetBaseClass("ASSET_FOUR", "ASSET", 700.0)

	mainAssets = [
		assetOne,
		assetTwo,
		nestedWatchTower,
		assetThree,
		assetFour
	]

	mainWatchTower = watchTowerBaseClass("MAIN_WATCH_TOWER", "WATCH_TOWER", 100.0, mainAssets)

	for i in range(100):
		mainWatchTower.update()
		mainWatchTower.report()

	print("THIS IS A BREAK POINT")
