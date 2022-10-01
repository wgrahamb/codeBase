"""

SIMDOF :

EACH OBJECT IS EITHER AN "ASSET" OR A "WATCHTOWER"
EACH OBJECT CONSISTS OF A CONSTRUCTOR AND AN UPDATE FUNCTION
THE OBJECTS ARE INDEXED BY AN ID AND TYPE
SIMDOF REQUIRES A CERTAIN FLOW OF OBJECTS AS LAID OUT BELOW
THIS IS FAST FOR SMALL INPUTS AND OUTPUTS
THE I/O IS DOCUMENTED
THE OUTPUT IS A FUNCTION USING VARIOUS AERODYNAMIC METHODS
UTILITY IS ONE HEADER FILE FOR NOW
IT BUILDS WITH THE PROJECT
IT IS INCLUDED AS A HEADER IN EVERY OBJECT
TABLES ARE HANDLED WITHIN EACH OBJECT TO PREVENT LARGE SINGULAR AMOUNTS OF MEMORY FROM BEING ALLOCATED
CURRENTLY LOGGING DATA WITH EACH INDIVIDUAL OBJECT

THIS FALLS UNDER THE FOLLOWING PSEUDO CODE :

	SIM_CONTROL - OVERHEAD CONTROL, INCLUDES END CHECK
		INPUT - NEW NAVIGATION STATE
		OUTPUT - END CHECK

	TARGET STATE - PREDICTED INTERCEPT POINT, CONSTANT, ACCELERATING
		INPUT - NA, IF ACCELERATING ASSUMED TO TARGET THE MISSILE LAUNCH SITE
		OUPUT - TARGET STATE

	MISSILE STATE - 
	### THIS IS A LARGE DATAPACKET THAT INCLUDES THE MISSILE TRANSLATIONAL STATE AND ROTATIONAL STATE IN MULTIPLE FRAMES ###
	### THIS IS BECAUSES THE NAVIGATION MODEL REQUIRES A FAIR AMOUNT OF DATA ###
	### THIS UPDATE DOES NOT CHANGE THE MISSILE STATE, IT ONLY TRANSLATES IT TO NEW FRAMES ###
		INPUT - NEW NAVIGATION STATE
		OUPUT - MISSILE STATE

	# SINGLE MISSILE ENGINE - S.M.E. CAN BE SWAPPED AS WHOLE OR INDIVIDUAL PIECES ( INDIVIDUAL SWAPS SHOULD HANDLED BY A MISSILE ENGINEER )

		NAVIGATION - LONGBOW, LOWERAD, MMT ( ALL SOURCE CODE )
		### COULD POSSIBLY BE OUTSIDE THE S.M.E. SINCE THE MISSILE STATE IS LARGE ###
			INPUT - MISSILE STATE
			OUTPUT - NAVIGATION STATE

		###
		NOW THE NAVIGATION STATE IS USED BY ALL OBJECTS INSIDE THE S.M.E.
		NAVIGATION STATE

			# TIMER
			MISSILE TIME OF FLIGHT - DETERMINED BY THE FREQUENCY OF THE DYNAMICS ENGINE

			# TRANSLATIONAL STATE
			LOCAL POSITION [3]
			LOCAL VELOCITY [3]
			LOCAL ACCELERATION [3]

			# ROTATIONAL STATE
			LOCAL EULER ANGLES [3]
			LOCAL RATE [3]
			LOCAL RATE DOT [3]

		###

		ATMOSPHERE - 1962 ATMOSPHERIC STANDARD, 1984 ATMOSPHERIC STANDARD, CAN ADD MORE
			INPUT - CURRENT ALTITUDE
			OUTPUT - AIR DENSITY, SPEED OF SOUND, PRESSURE

		SEEKER - HIGH FIDELITY, DYNAMIC STATE INTEGRATION, TRUTH
			INPUT - NAVIGATION STATE, TARGET STATE
			OUTPUT - MISSILE TO INTERCEPT RELATIVE POSITION IN BODY FRAME, MISSILE TO INTERCEPT RELATIVE VELOCITY IN BODY FRAME

		GUIDANCE - LINE OF ATTACK, PROPORTIONAL GUIDANCE, BLENDED GUIDANCE
			INPUT - NAVIGATION STATE, MISSILE TO INTERCEPT RELATIVE POSITION IN BODY FRAME, MISSILE TO INTERCEPT RELATIVE VELOCITY IN BODY FRAME
			OUTPUT - PITCH GUIDANCE COMMAND, YAW GUIDANCE COMMAND

		CONTROL - LONGBOW, LOWERAD, MMT ( ALL SOURCE CODE )
			INPUT - NAVIGATION STATE, PITCH GUIDANCE COMMAND, YAW GUIDANCE COMMAND, AIR DENSITY, SPEED OF SOUND, PRESSURE
			OUTPUT - FIN COMMANDS

		ACTUATORS - MMT HIGH FIDELITY MODEL, DYNAMIC STATE INTEGRATION
			INPUT - FIN COMMANDS
			OUTPUT - FIN DEFLECTIONS

		DYNAMICS - LONGBOW, LOWERAD, MMT ( ALL SOURCE CODE )
			INPUT - NAVIGATION STATE, FIN DEFLECTIONS, AIR DENSITY, SPEED OF SOUND, PRESSURE
			OUTPUT - NEW NAVIGATION STATE

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
