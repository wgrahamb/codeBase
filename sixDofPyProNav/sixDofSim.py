# INCLUDED WITH PYTHON
import time
from enum import Enum

# PIP INSTALLED LIBRARIES
from ambiance import Atmosphere as atm
from ambiance import CONST as seaLevelAtm
import numpy as np
from numpy import array as npa
from numpy import linalg as la

# GRAHAM'S FUNCTIONS
from coordinateTransformations import FLIGHTPATH_TO_LOCAL_TM
from coordinateTransformations import ORIENTATION_TO_LOCAL_TM
from loadPickle import loadpickle as lp
from unitVector import unitvector
from returnAzAndElevation import returnAzAndElevation
from projection import projection

class endChecks(Enum):
	intercept = 1
	flying = 0
	groundCollision = -1
	pointOfClosestApproachPassed = -2
	notANumber = -3
	maxTimeExceeded = -4
	forcedSimTermination = -5

class sixDofSim:

	def __init__(self):

		########################################################################################################################
		#
		# AUTHOR - WILSON GRAHAM BEECH
		# REFERENCE - MODELING AND SIMULATION OF AEROSPACE VEHICLE DYNAMICS SECOND EDITON - PETER H. ZIPFEL
		#
		# EAST, NORTH, UP COORDINATE SYSTEM
		#
		# INTERCEPTOR LOCAL ORIENTATION
		# ARRAY 0 >>> LOOKING DOWN THE NOZZLE OF THE INTERCEPTOR
		# ARRAY 1 >>> LOOKING DOWN THE NOZZLE OF THE INTERCEPTOR, THIS POINTS OUT THE LEFT HAND SIDE
		# ARRAY 2 >>> LOOKING DOWN THE NOZZLE OF THE INTERCEPTOR, THIS POINTS OUT THE TOP SIDE OF THE INTERCEPTOR
		#                           POSITIVE NORMAL
		#                                         |
		#                                         |
		#                                         |
		#  POSITIVE SIDE -----------O----------- NEGATIVE SIDE
		#                                         |
		#                                         |
		#                                         |
		#                          NEGATIVE NORMAL
		#
		# NEGATIVE AXIS IS COMING OUT OF THE SCREEN STRAIGHT AT YOU
		# POSITIVE AXIS IS POINTING INTO THE SCREEN DIRECTLY AWAY FROM YOU
		#
		# POSITIVE ALPHA INDICATES NOSE BELOW FREE STREAM VELOCITY
		# POSITIVE BETA INDICATES NOSE LEFT FREE STREAM VELOCITY
		# POSITIVE ROLL INDICATES NORMAL AXIS CLOCKWISELY ROTATED FROM TWELVE O'CLOCK
		#
		# FIN ORIENTATION
		# LOOKING DOWN THE NOZZLE OF THE MISSILE
		#
		#                              FIN 4       FIN 1
		#                                         X
		#                              FIN 3       FIN 2
		#
		########################################################################################################################

		# SIM CONTROL
		self.wallClockStart = time.time()
		self.go = True
		self.timeStep = 0.001 # SECONDS
		self.integrationStep = 0.001 # SECONDS
		self.maxTime = 400 # SECONDS

		# TARGETED INTERCEPT POINT
		self.tgtPos = npa([7000.0, 7000.0, 7000.0])
		self.tgtVel = npa([-400.0, -400.0, -300.0])

		### MSL PACKET ###
		self.mslTof = 0.0 # SECONDS
		self.mslAz = np.radians(45) # RADIANS >>> ONLY POSITIVE NUMBERS 0-360, MEASURED COUNTER CLOCKWISE FROM TRUE EAST
		self.mslEl = np.radians(60) # RADIANS
		self.mslLocalOrient = FLIGHTPATH_TO_LOCAL_TM(self.mslAz, -self.mslEl) # ND
		self.mslPos = np.zeros(3)# METERS
		self.mslVel = self.mslLocalOrient[0] # METERS PER SECOND
		self.mslBodyVel = self.mslLocalOrient @ self.mslVel # METERS PER SECOND
		self.mslAcc = np.zeros(3) # METERS PER SECOND^2
		self.mslBodyAcc = self.mslLocalOrient @ self.mslAcc # METERS PER SECOND^2
		self.mslSpeed = la.norm(self.mslBodyVel) # METERS PER SECOND
		self.mslMach = 0.0 # ND
		self.mslRange = 0.0 # METERS
		self.mslAlpha = 0.0 # RADIANS
		self.mslBeta = 0.0 # RADIANS
		self.mslEuler = npa([0.0, self.mslEl, self.mslAz]) # RADIANS
		self.mslEulerDot = np.zeros(3) # RADIANS PER SECOND
		self.mslRate = np.zeros(3) # RADIANS PER SECOND
		self.mslRateDot = np.zeros(3) # RADIANS PER SECOND^2
		self.mslRefArea = 0.01824 # METERS^2
		self.mslRefDiam = 0.1524 # METERS
		self.mslExitArea = 0.0125 # METERS^2
		self.mslBurnOut = 2.421 # SECONDS

		###### FUNCTION VARIABLES ######

		# TIME OF FLIGHT
		### VARIABLES INCLUDED IN MSL PACKET ###

		# ENVIRONMENT
		self.grav = 0.0 # METERS PER SECOND^2
		self.gravBodyVec = np.zeros(3) # METERS PER SECOND^2
		self.press = 0.0 # PASCALS
		self.q = 0.0 # PASCALS

		# SEEKER >>> INITIALIZE BY POINTING THE SEEKER DIRECTLY AT THE TARGET
		relPosU = unitvector(self.tgtPos - self.mslPos) # ND
		mslToInterceptU = self.mslLocalOrient @ relPosU # ND
		mslToInterceptAz, mslToInterceptEl = returnAzAndElevation(mslToInterceptU) # RADIANS
		self.seekerPitch = mslToInterceptEl # RADIANS
		self.seekerYaw = mslToInterceptAz # RADIANS
		self.seekerLocalOrient = np.zeros((3, 3)) # ND
		self.seekPitchErr = 0.0 # ND
		self.seekYawErr = 0.0 # ND
		self.gk = 10 # KALMAN FILTER GAIN >>> ONE PER SECOND
		self.zetak = 0.9 # KALMAN FILTER DAMPING
		self.wnk = 60 # KALMAN FILTER NATURAL FREQUENCY >>> RADIANS PER SECOND
		self.wlr = self.seekerYaw # RADIANS PER SECOND >>> POINTING YAW RATE
		self.wlrd = 0.0 # RADIANS PER SECOND^2 >>> DERIVATIVE OF POINTING YAW RATE
		self.wlr1 = 0.0 # RADIANS PER SECOND >>> YAW SIGHT LINE SPIN RATE
		self.wlr1d = 0.0 # RADIANS PER SECOND^2 >>> DERIVATIVE OF YAW SIGHT LINE SPIN RATE
		self.wlr2 = 0.0 # RADIANS PER SECOND^2 >>> SECOND STATE VARIABLE IN KALMAN FILTER, YAW
		self.wlr2d = 0.0 # RADIANS PER SECOND^3 >>> DERIVATIVE OF SECOND STATE VARIABLE IN KALMAN FILTER, YAW
		self.wlq = self.seekerPitch # RADIANS PER SECOND >>> POINTING PITCH RATE
		self.wlqd = 0.0 # RADIANS PER SECOND^2 >>> DERIVATIVE OF POINTING PITCH RATE
		self.wlq1 = 0.0 # RADIANS PER SECOND >>> PITCH SIGHT LINE SPIN RATE
		self.wlq1d = 0.0 # RADIANS PER SECOND^2 >>> DERIVATIVE OF PITCH SIGHT LINE SPIN RATE
		self.wlq2 = 0.0 # RADIANS PER SECOND^2 >>> SECOND STATE VARIABLE IN KALMAN FILTER, PITCH
		self.wlq2d = 0.0 # RADIANS PER SECOND^3 >>> DERIVATIVE OF SECOND STATE VARIABLE IN KALMAN FILTER, PITCH

		# GUIDANCE
		self.forwardLeftUpMslToInterceptRelPos = np.zeros(3) # METERS
		self.loft = 1.0 # ND >>> THIS IS THE ELEVATION PARAMETER OF THE LINE OF ATTACK, ASSUMING LINE OF SIGHT FOR THE AZIMUTH PARAMETER
		self.K = 1 # ND
		self.proNavGain = 4 # ND
		self.normCommand = 0.0 # METERS PER SECOND^2
		self.sideCommand = 0.0 # METERS PER SECOND^2
		self.maxAccelAllow = 500.0 # METERS PER SECOND^2 >>> ROUGHLY FIFTY Gs
		self.maxAccel = self.maxAccelAllow

		# CONTROL
		self.zetlagr = 0.6 # ND >>> DAMPING OF CLOSED RATE LOOP
		self.wrcl = 20 # RADIANS PER SECOND >>> FREQUENCY OF ROLL CLOSED LOOP COMPLEX POLE
		self.zrcl = 0.9 # ND >> DAMPING OF ROLL CLOSED LOOP POLE
		self.yy = 0.0 # METERS PER SECOND >>> YAW FEED FORWARD INTEGRATION
		self.yyd = 0.0 # METERS PER SECOND >>> YAW FEED FORWARD DERIVATIVE
		self.zz = 0.0 # METERS PER SECOND >>> PITCH FEED FORWARD INTEGRATION
		self.zzd = 0.0 # METER PER SECOND >>> PITCH FEED FORWARD DERIVATIVE
		self.maxDefl = 28.0 # DEGREES 
		self.pitchFinComm = 0.0 # RADIANS
		self.yawFinComm = 0.0 # RADIANS
		self.rollFinComm = 0.0 # RADIANS
		self.rollComm = 0.0 # RADIANS

		# ACTUATORS
		self.pitchFinDefl = 0.0 # RADIANS
		self.yawFinDefl = 0.0 # RADIANS
		self.rollFinDefl = 0.0 # RADIANS
		self.DEL1 = 0.0 # RADIANS >>> FIN POSITION
		self.DEL1D = 0.0 # RADIANS PER SECOND >>> FIN POSITON DERIVED
		self.DEL1DOT = 0.0 # RADIANS PER SECOND >>> FIN RATE
		self.DEL1DOTDOT = 0.0 # RADIANS PER S^2 >>> FIN RATE DERIVED
		self.DEL2 = 0.0 # RADIANS
		self.DEL2D = 0.0 # RADIANS PER SECOND
		self.DEL2DOT = 0.0 # RADIANS PER SECOND >>> FIN RATE
		self.DEL2DOTDOT = 0.0 # RADIANS PER S^2 >>> FIN RATE DERIVED
		self.DEL3 = 0.0 # RADIANS
		self.DEL3D = 0.0 # RADIANS PER SECOND
		self.DEL3DOT = 0.0 # RADIANS PER SECOND >>> FIN RATE
		self.DEL3DOTDOT = 0.0 # RADIANS PER S^2 >>> FIN RATE DERIVED
		self.DEL4 = 0.0 # RADIANS
		self.DEL4D = 0.0 # RADIANS SECOND
		self.DEL4DOT = 0.0 # RADIANS PER SECOND >>> FIN RATE
		self.DEL4DOTDOT = 0.0 # RADIANS PER S^2 >>> FIN RATE DERIVED
		self.finRadianLimit = 0.4887 # RADIANS
		self.finRateLimit = 10.472 # RADIANS PER SECOND
		self.WNACT = 100 # NATURAL FREQUENCY OF ACTUATOR >>> RADIANS PER SECOND
		self.ZETACT = 0.7 # DAMPING OF ACTUATOR

		# AERO BALLISTIC ANGLES
		self.alphaPrimeDeg = 0.0 # DEGREES
		self.sinPhiPrime = 0.0 # ND
		self.cosPhiPrime = 0.0 # ND
		self.pitchDeflAeroDeg = 0.0 # DEGREES
		self.yawDeflAeroDeg = 0.0 # DEGREES
		self.rollDeflDeg = 0.0 # DEGREES
		self.totalFinDeflDeg = 0.0 # DEGREES
		self.pitchRateAeroDeg = 0.0 # DEGREES PER SECOND
		self.yawRateAeroDeg = 0.0 # DEGREES PER SECOND
		self.rollRateDeg = 0.0 # DEGREES PER SECOND
		self.sinOfFourTimesPhiPrime = 0.0 # ND
		self.squaredSinOfTwoTimesPhiPrime = 0.0 # ND

		# DATA LOOK UP
		self.lookUpValues = lp("lookUpTables.pickle")
		self.CA0 = 0.0 # ND
		self.CAA = 0.0 # PER DEGREE
		self.CAD = 0.0 # PER DEGREE^2
		self.CAOFF = 0.0 # ND
		self.CYP = 0.0 # ND
		self.CYDR = 0.0 # PER DEGREE
		self.CN0 = 0.0 # ND
		self.CNP = 0.0 # ND
		self.CNDQ = 0.0 # PER DEGREE
		self.CLLAP = 0.0 # PER DEGREE^2
		self.CLLP = 0.0 # PER DEGREE
		self.CLLDP = 0.0 # PER DEGREE
		self.CLM0 = 0.0 # ND
		self.CLMP = 0.0 # ND
		self.CLMQ = 0.0 # PER DEGREE
		self.CLMDQ = 0.0 # PER DEGREE
		self.CLNP = 0.0 # ND
		self.mass = 0.0 # KILOGRAMS
		self.unAdjThrust = 0.0 # NEWTONS
		self.transverseMomentOfInertia = 0.0 # KILOGRAMS * METERS^2
		self.axialMomentOfInertia = 0.0 # KILOGRAMS * METERS^2
		self.cgFromNose = 0.0 # METERS
		self.alphaPrimeMax = 40.0 # DEGREES

		# PROPULSION
		self.seaLevelPress = seaLevelAtm.P_0
		self.thrust = 0.0

		# AERO DYNAMIC COEFFICIENTS
		self.launchCg = self.lookUpValues["CENTER OF GRAVITY"](0.0) # METERS
		self.CX = 0 # AXIAL FORCE COEFFICIENT
		self.CY = 0 # SIDE FORCE COEFFICIENT
		self.CZ = 0 # NORMAL FORCE COEFFICIENT
		self.CL = 0 # ROLL MOMENT COEFFICIENT
		self.CM = 0 # PITCHING MOMENT COEFFICIENT
		self.CN = 0 # YAWING MOMENT COEFFICIENT

		# AERO DYNAMIC REFERENCE DATA
		self.refAeroData = {}
		self.staticMargin = 0

		# ACCELERATE
		### VARIABLES INCLUDED IN MSL PACKET ###

		# ROTATE
		### VARIABLES INCLUDED IN MSL PACKET ###

		# EULER ANGLES
		### VARIABLES INCLUDED IN MSL PACKET ###

		# EULER SEMI IMPLICIT INTEGRATION
		### VARIABLES INCLUDED IN MSL PACKET ###

		# INTERCEPT
		self.missDistance = 0.0 # METERS

		# END CHECK
		self.lethality = endChecks.flying # ND

		# LOG DATA
		self.logFile = open("log.txt", "w")
		self.logFile.write(f"tof posE posN posU tgtE tgtN tgtU normComm normAch pitchRate thetaRate pitchDefl alpha theta sideComm sideAch yawRate psiRate yawDefl beta psi rollComm phiRate rollRate rollDefl roll seekPitchErr seekYawErr staticMargin mach\n")

		# FLY
		### NO NEW VARIABLES ###

		# MAIN
		### NO NEW VARIABLES ###

	def integrate(self, dy_new, dy, y, step):
		return y + (dy_new + dy) * step / 2

	def target(self):
		target = npa([10.0, 10.0, 0.0])
		relPos = target - self.tgtPos
		relPosU = unitvector(relPos)
		closingVel = -1 * self.tgtVel # METERS PER SECOND
		closingVelMag = la.norm(closingVel) # METERS PER SECOND
		TEMP1 = np.cross(relPos, closingVel)
		TEMP2 = np.dot(relPos, relPos)
		lineOfSightRate = TEMP1 / TEMP2 # RADIANS PER SECOND
		command = np.cross(-1 * self.proNavGain * closingVelMag * relPosU, lineOfSightRate) # METERS PER SECOND^2
		deltaVel = command * self.timeStep
		self.tgtVel += deltaVel
		deltaPos = self.tgtVel * self.timeStep
		self.tgtPos += deltaPos

	def timeOfFlight(self):
		self.mslTof += self.timeStep # SECONDS

	def environment(self):
		# PASCALS = NEWTONS PER METER^2 = KILOGRAMS PER (METER * SECOND^2)
		# NEWTONS = (KILOGRAMS * METER) PER SECOND^2
		alt = self.mslPos[2] # METERS
		atmos = atm(alt) # STANDARD ATMOSPHERIC DATA
		rho = atmos.density[0] # KILOGRAMS PER METER^3
		self.grav = atmos.grav_accel[0] # METERS PER SECOND^2
		gravLocalVec = npa([0.0, 0.0, -self.grav]) # METERS PER SECOND^2
		self.gravBodyVec = self.mslLocalOrient @ gravLocalVec # METERS PER SECOND^2
		self.press = atmos.pressure[0] # PASCALS
		a = atmos.speed_of_sound[0] # METERS PER SECOND
		self.mslSpeed = la.norm(self.mslBodyVel) # METERS PER SECOND
		self.mslMach = self.mslSpeed / a # ND
		self.q = 0.5 * rho * self.mslSpeed * self.mslSpeed # PASCALS
	
	def seeker(self):

		wsq = self.wnk ** 2 # ND
		gg = self.gk * wsq # ND

		# YAW CHANNEL
		wlr1d_new = self.wlr2 # RADIANS PER SECOND^2
		self.wlr1 = self.integrate(wlr1d_new, self.wlr1d, self.wlr1, self.integrationStep) # RADIANS PER SECOND
		self.wlr1d = wlr1d_new # RADIANS PER SECOND^2
		wlr2d_new = gg * self.seekYawErr - 2 * self.zetak * self.wnk * self.wlr1d - wsq * self.wlr1 # RADIANS PER SECOND^3
		self.wlr2 = self.integrate(wlr2d_new, self.wlr2d, self.wlr2, self.integrationStep) # RADIANS PER SECOND^2
		self.wlr2d = wlr2d_new # RADIANS PER SECOND^3

		# PITCH CHANNEL
		wlq1d_new = self.wlq2 # RADIANS PER SECOND^2
		self.wlq1 = self.integrate(wlq1d_new, self.wlq1d, self.wlq1, self.integrationStep) # RADIANS PER SECOND
		self.wlq1d = wlq1d_new # RADIANS PER SECOND^2
		wlq2d_new = gg * self.seekPitchErr - 2 * self.zetak * self.wnk * self.wlq1d - wsq * self.wlq1 # RADIANS PER SECOND^3
		self.wlq2 = self.integrate(wlq2d_new, self.wlq2d, self.wlq2, self.integrationStep) # RADIANS PER SECOND^2
		self.wlq2d = wlq2d_new # RADIANS PER SECOND^3
		
		# YAW CONTROL
		wlrd_new = self.wlr1 - self.mslEulerDot[2] # RADIANS PER SECOND^2
		self.wlr = self.integrate(wlrd_new, self.wlrd, self.wlr, self.integrationStep) # RADIANS PER SECOND
		self.wlrd = wlrd_new # RADIANS PER SECOND^2
		self.seekerYaw = self.wlr # RADIANS

		# PITCH CONTROL
		wlqd_new = self.wlq1 - self.mslEulerDot[1] # RADIANS PER SECOND^2
		self.wlq = self.integrate(wlqd_new, self.wlqd, self.wlq, self.integrationStep) # RADIANS PER SECOND
		self.wlqd = wlqd_new # RADIANS PER SECOND^2
		self.seekerPitch = self.wlq # RADIANS

		localRelPos = self.tgtPos - self.mslPos # METERS
		seekerAttitudeToLocalTM = ORIENTATION_TO_LOCAL_TM(0, -self.seekerPitch, self.seekerYaw) # ND
		self.seekerLocalOrient = seekerAttitudeToLocalTM @ self.mslLocalOrient # ND
		seekerToInterceptRelPos = (self.seekerLocalOrient @ localRelPos) * npa([1.0, 0.5, 0.2]) # METERS >>> ARRAY AT THE END SIMULATES ERROR
		self.seekYawErr, self.seekPitchErr = returnAzAndElevation(seekerToInterceptRelPos) # RADIANS
		self.forwardLeftUpMslToInterceptRelPos = (seekerToInterceptRelPos @ seekerAttitudeToLocalTM) # METERS

	def guidance(self):
		# PROPORTIONAL NAVIGATION
		relPos = self.tgtPos - self.mslPos # METERS
		self.forwardLeftUpMslToInterceptRelPos = self.mslLocalOrient @ relPos # METERS
		forwardLeftUpMslToInterceptRelPosU = unitvector(self.forwardLeftUpMslToInterceptRelPos) # ND
		forwardLeftUpMslToInterceptLineOfSightVel = projection(forwardLeftUpMslToInterceptRelPosU, self.mslBodyVel) # METERS PER SECOND
		timeToGo = la.norm(self.forwardLeftUpMslToInterceptRelPos) / la.norm(forwardLeftUpMslToInterceptLineOfSightVel) # SECONDS
		if timeToGo < 3:
			closingVel = -1 * self.mslBodyVel # METERS PER SECOND
			closingVelMag = la.norm(closingVel) # METERS PER SECOND
			TEMP1 = np.cross(self.forwardLeftUpMslToInterceptRelPos, closingVel)
			TEMP2 = np.dot( self.forwardLeftUpMslToInterceptRelPos, self.forwardLeftUpMslToInterceptRelPos)
			lineOfSightRate = TEMP1 / TEMP2 # RADIANS PER SECOND
			command = np.cross(-1 * self.proNavGain * closingVelMag * forwardLeftUpMslToInterceptRelPosU, lineOfSightRate) # METERS PER SECOND^2
			self.normCommand = command[2] # METERS PER SECOND^2
			self.sideCommand = command[1] # METERS PER SECOND^2
		else:
			lineOfAttack = npa([forwardLeftUpMslToInterceptRelPosU[0], forwardLeftUpMslToInterceptRelPosU[1], self.loft]) # ND
			forwardLeftUpMslToInterceptLineOfAttackVel = projection(lineOfAttack, self.mslBodyVel) # METERS PER SECOND
			G = 1 - np.exp(-1 * la.norm(self.forwardLeftUpMslToInterceptRelPos) / 10000) # ND
			command = npa(
				[
					0.0,
					self.K * (forwardLeftUpMslToInterceptLineOfSightVel[1] + G * forwardLeftUpMslToInterceptLineOfAttackVel[1]),
					self.K * (forwardLeftUpMslToInterceptLineOfSightVel[2] + G * forwardLeftUpMslToInterceptLineOfAttackVel[2]),
				]
			) # METERS PER SECOND^2
			self.normCommand = command[2] # METERS PER SECOND^2
			self.sideCommand = command[1] # METERS PER SECOND^2
		
		accMag = la.norm(npa([self.sideCommand, self.normCommand])) # METER PER SECOND^2
		trigonometricRatio = np.arctan2(self.normCommand, self.sideCommand) # ND
		if accMag > self.maxAccel:
			accMag = self.maxAccel # METERS PER SECOND^2
		self.sideCommand = accMag * np.cos(trigonometricRatio) # METERS PER SECOND^2
		self.normCommand = accMag * np.sin(trigonometricRatio) # METERS PER SECOND^2

	def control(self):
		
		if len(self.refAeroData) > 0 and self.mslMach > 0.6:
			
			deg = 57.3
			CNA = self.refAeroData["CNA"] * deg # ND
			CMA = self.refAeroData["CMA"] * deg # ND
			CMD = self.refAeroData["CMD"] * deg # ND
			CMQ = self.refAeroData["CMQ"] * deg # ND
			CLP = self.refAeroData["CLP"] * deg # ND
			CLD = self.refAeroData["CLD"] * deg # ND
			mass = self.mass # KILOGRAMS
			tMoi = self.transverseMomentOfInertia # KILOGRAMS * METERS^2
			aMoi = self.axialMomentOfInertia # KILOGRAMS * METERS^2

			DNA = CNA * (self.q * self.mslRefArea / mass) # METERS PER SECOND^2
			DMA = CMA * (self.q * self.mslRefArea * self.mslRefDiam / tMoi) # PER SECOND^2
			DMD = CMD * (self.q * self.mslRefArea * self.mslRefDiam / tMoi) # PER SECOND^2
			DMQ = CMQ * (self.mslRefDiam / (2 * self.mslSpeed)) * (self.q * self.mslRefArea * self.mslRefDiam / tMoi) # PER SECOND
			DLP = CLP * (self.mslRefDiam / (2 * self.mslSpeed)) * (self.q * self.mslRefArea * self.mslRefDiam / aMoi) # PER SECOND
			DLD = CLD * (self.q * self.mslRefArea * self.mslRefDiam / aMoi) # PER SECOND^2

			WACL = 0.013 * np.sqrt(self.q) + 7.1 # PER SECOND
			ZACL = 0.000559 * np.sqrt(self.q) + 0.232 # ND
			PACL = 14 # ND

			# FEEDBACK GAINS
			GAINFB3 = WACL * WACL * PACL / (DNA * DMD) # ND
			GAINFB2 = (2 * ZACL * WACL + PACL + DMQ - DNA / self.mslSpeed) / DMD # ND
			GAINFB1 = (
				WACL ** 2 + 2 * ZACL * WACL * PACL + DMA + DMQ * DNA / self.mslSpeed - GAINFB2 * DMD * DNA / self.mslSpeed
			) / (DNA * DMD) # ND
			
			# ROLL
			GKP = (2 * self.wrcl * self.zrcl + DLP) / DLD # ND
			GKPHI = self.wrcl * self.wrcl / DLD # ND
			EPHI = GKPHI * (self.rollComm - self.mslEuler[0]) # RADIANS
			self.rollFinComm = EPHI - GKP * self.mslEulerDot[0] # RADIANS

			# PITCH
			zzdNew = self.normCommand - self.refAeroData["BODYACC"][2] # METERS PER SECOND
			self.zz = self.integrate(zzdNew, self.zzd, self.zz, self.integrationStep) # METERS PER SECOND
			self.zzd = zzdNew # METERS PER SECOND
			deflPitch = -1 * GAINFB1 * self.refAeroData["BODYACC"][2] - GAINFB2 * self.mslEulerDot[1] + GAINFB3 * self.zz # DEGREES
			if np.abs(deflPitch) > self.maxDefl:
				deflPitch = np.sign(deflPitch) * self.maxDefl # DEGREES
			self.pitchFinComm = np.radians(deflPitch) # RADIANS

			# YAW
			yydNew = self.refAeroData["BODYACC"][1] - self.sideCommand # METERS PER SECOND
			self.yy = self.integrate(yydNew, self.yyd, self.yy, self.integrationStep) # METERS PER SECOND
			self.yyd = yydNew # METERS PER SECOND
			deflYaw = -1 * GAINFB1 * -1 * self.refAeroData["BODYACC"][1] - GAINFB2 * self.mslEulerDot[2] + GAINFB3 * self.yy # DEGREES
			if np.abs(deflYaw) > self.maxDefl: 
				deflYaw = np.sign(deflYaw) * self.maxDefl # DEGREES
			self.yawFinComm = np.radians(deflYaw) # RADIANS

		# THIS KEEPS THE MISSILE ON LINE BEFORE IT GAINS ENOUGH SPEED TO PROPERLY MANEUVER
		elif len(self.refAeroData) > 0:

			deg = 57.3
			CNA = self.refAeroData["CNA"] * deg # ND
			CMA = self.refAeroData["CMA"] * deg # ND
			CMD = self.refAeroData["CMD"] * deg # ND
			CMQ = self.refAeroData["CMQ"] * deg # ND
			CLP = self.refAeroData["CLP"] * deg # ND
			CLD = self.refAeroData["CLD"] * deg # ND
			CND = self.refAeroData["CND"] * deg #ND
			mass = self.mass # KILOGRAMS
			tMoi = self.transverseMomentOfInertia # KILOGRAMS * METERS^2
			aMoi = self.axialMomentOfInertia # KILOGRAMS * METERS^2

			DNA = CNA * (self.q * self.mslRefArea / mass) # METERS PER SECOND^2
			DND = CND * (self.q * self.mslRefArea / mass) # METERS PER SECOND^2
			DMA = CMA * (self.q * self.mslRefArea * self.mslRefDiam / tMoi) # PER SECOND^2
			DMD = CMD * (self.q * self.mslRefArea * self.mslRefDiam / tMoi) # PER SECOND^2
			DMQ = CMQ * (self.mslRefDiam / (2 * self.mslSpeed)) * (self.q * self.mslRefArea * self.mslRefDiam / tMoi) # PER SECOND
			DLP = CLP * (self.mslRefDiam / (2 * self.mslSpeed)) * (self.q * self.mslRefArea * self.mslRefDiam / aMoi) # PER SECOND
			DLD = CLD * (self.q * self.mslRefArea * self.mslRefDiam / aMoi) # PER SECOND^2

			# ROLL
			GKP = (2 * self.wrcl * self.zrcl + DLP) / DLD # ND
			GKPHI = self.wrcl * self.wrcl / DLD # ND
			EPHI = GKPHI * (self.rollComm - self.mslEuler[0]) # RADIANS
			self.rollFinComm = EPHI - GKP * self.mslEulerDot[0] # RADIANS

			# RATE CONTROL
			ZRATE = DNA / self.mslSpeed - DMA * DND / (self.mslSpeed * DMD) # ND
			AA = DNA / self.mslSpeed - DMQ # ND
			BB = -1 * DMA - DMQ * DNA / self.mslSpeed # ND
			TEMP1 = AA - 2 * self.zetlagr * self.zetlagr * ZRATE # ND
			TEMP2 = AA * AA - 4 * self.zetlagr * self.zetlagr * BB # ND
			RADIX = TEMP1 ** 2 - TEMP2 # ND
			GRATE = (-1 * TEMP1 + np.sqrt(RADIX)) / (-1 * DMD) # ND

			# PITCH
			self.pitchFinComm = GRATE * self.mslEulerDot[1] # RADIANS

			# YAW
			self.yawFinComm = GRATE * self.mslEulerDot[2] # RADIANS

	def actuators(self):
		
		DELC1 = -self.rollFinComm + self.pitchFinComm - self.yawFinComm
		DELC2 = -self.rollFinComm + self.pitchFinComm + self.yawFinComm
		DELC3 = self.rollFinComm + self.pitchFinComm - self.yawFinComm
		DELC4 = self.rollFinComm + self.pitchFinComm + self.yawFinComm

		# FIN ONE
		flag = 0
		if np.abs(self.DEL1 > self.finRadianLimit):
			self.DEL1 = self.finRadianLimit * np.sign(self.DEL1)
			if (self.DEL1 * self.DEL1DOT) > 0.0:
				self.DEL1DOT = 0.0
		if np.abs(self.DEL1DOT > self.finRateLimit):
			flag = 1
			self.DEL1DOT = self.finRateLimit * np.sign(self.DEL1DOT)
		DEL1D_NEW = self.DEL1DOT
		self.DEL1 = self.integrate(DEL1D_NEW, self.DEL1D, self.DEL1, self.integrationStep)
		self.DEL1D = DEL1D_NEW
		EDX1 = DELC1 - self.DEL1
		DEL1DOTDOT_NEW = self.WNACT * self.WNACT * EDX1 - 2 * self.ZETACT * self.WNACT * self.DEL1D
		self.DEL1DOT = self.integrate(DEL1DOTDOT_NEW, self.DEL1DOTDOT, self.DEL1DOT, self.integrationStep)
		self.DEL1DOTDOT = DEL1DOTDOT_NEW
		if flag == 1 and (self.DEL1DOT * self.DEL1DOTDOT) > 0:
			self.DEL1DOTDOT = 0

		# FIN TWO
		flag = 0
		if np.abs(self.DEL2 > self.finRadianLimit):
			self.DEL2 = self.finRadianLimit * np.sign(self.DEL2)
			if (self.DEL2 * self.DEL2DOT) > 0.0:
				self.DEL2DOT = 0.0
		if np.abs(self.DEL2DOT > self.finRateLimit):
			flag = 1
			self.DEL2DOT = self.finRateLimit * np.sign(self.DEL2DOT)
		DEL2D_NEW = self.DEL2DOT
		self.DEL2 = self.integrate(DEL2D_NEW, self.DEL2D, self.DEL2, self.integrationStep)
		self.DEL2D = DEL2D_NEW
		EDX2 = DELC2 - self.DEL2
		DEL2DOTDOT_NEW = self.WNACT * self.WNACT * EDX2 - 2 * self.ZETACT * self.WNACT * self.DEL2D
		self.DEL2DOT = self.integrate(DEL2DOTDOT_NEW, self.DEL2DOTDOT, self.DEL2DOT, self.integrationStep)
		self.DEL2DOTDOT = DEL2DOTDOT_NEW
		if flag == 1 and (self.DEL2DOT * self.DEL2DOTDOT) > 0:
			self.DEL2DOTDOT = 0

		# FIN THREE
		flag = 0
		if np.abs(self.DEL3 > self.finRadianLimit):
			self.DEL3 = self.finRadianLimit * np.sign(self.DEL3)
			if (self.DEL3 * self.DEL3DOT) > 0.0:
				self.DEL3DOT = 0.0
		if np.abs(self.DEL3DOT > self.finRateLimit):
			flag = 1
			self.DEL3DOT = self.finRateLimit * np.sign(self.DEL3DOT)
		DEL3D_NEW = self.DEL3DOT
		self.DEL3 = self.integrate(DEL3D_NEW, self.DEL3D, self.DEL3, self.integrationStep)
		self.DEL3D = DEL3D_NEW
		EDX3 = DELC3 - self.DEL3
		DEL3DOTDOT_NEW = self.WNACT * self.WNACT * EDX3 - 2 * self.ZETACT * self.WNACT * self.DEL3D
		self.DEL3DOT = self.integrate(DEL3DOTDOT_NEW, self.DEL3DOTDOT, self.DEL3DOT, self.integrationStep)
		self.DEL3DOTDOT = DEL3DOTDOT_NEW
		if flag == 1 and (self.DEL3DOT * self.DEL3DOTDOT) > 0:
			self.DEL3DOTDOT = 0

		# FIN FOUR
		flag = 0
		if np.abs(self.DEL4 > self.finRadianLimit):
			self.DEL4 = self.finRadianLimit * np.sign(self.DEL4)
			if (self.DEL4 * self.DEL4DOT) > 0.0:
				self.DEL4DOT = 0.0
		if np.abs(self.DEL4DOT > self.finRateLimit):
			flag = 1
			self.DEL4DOT = self.finRateLimit * np.sign(self.DEL4DOT)
		DEL4D_NEW = self.DEL4DOT
		self.DEL4 = self.integrate(DEL4D_NEW, self.DEL4D, self.DEL4, self.integrationStep)
		self.DEL4D = DEL4D_NEW
		EDX4 = DELC4 - self.DEL4
		DEL4DOTDOT_NEW = self.WNACT * self.WNACT * EDX4 - 2 * self.ZETACT * self.WNACT * self.DEL4D
		self.DEL4DOT = self.integrate(DEL4DOTDOT_NEW, self.DEL4DOTDOT, self.DEL4DOT, self.integrationStep)
		self.DEL4DOTDOT = DEL4DOTDOT_NEW
		if flag == 1 and (self.DEL4DOT * self.DEL4DOTDOT) > 0:
			self.DEL4DOTDOT = 0

		self.rollFinDefl = (-self.DEL1 - self.DEL2 + self.DEL3 + self.DEL4) / 4
		self.pitchFinDefl = (self.DEL1 + self.DEL2 + self.DEL3 + self.DEL4) / 4
		self.yawFinDefl = (-self.DEL1 + self.DEL2 - self.DEL3 + self.DEL4) / 4

	def aeroBallisticAngles(self):
		alphaPrime = np.arccos(np.cos(self.mslAlpha) * np.cos(self.mslBeta)) # RADIANS
		self.alphaPrimeDeg = np.degrees(alphaPrime) # DEGREES
		phiPrime = np.arctan2(
			np.tan(self.mslBeta),
			np.sin(self.mslAlpha)
		) # RADIANS
		self.sinPhiPrime = np.sin(phiPrime) # ND
		self.cosPhiPrime = np.cos(phiPrime) # ND
		pitchDeflAeroFrame = self.pitchFinDefl * self.cosPhiPrime - self.yawFinDefl * self.sinPhiPrime # RADIANS
		self.pitchDeflAeroDeg = np.degrees(pitchDeflAeroFrame) # DEGREES
		yawDeflAeroFrame = self.pitchFinDefl * self.sinPhiPrime + self.yawFinDefl * self.cosPhiPrime # RADIANS
		self.yawDeflAeroDeg = np.degrees(yawDeflAeroFrame) # DEGREES
		self.rollDeflDeg = np.degrees(self.rollFinDefl)
		self.totalFinDeflDeg = (np.abs(self.pitchDeflAeroDeg) + np.abs(self.yawDeflAeroDeg)) / 2 # DEGREES
		pitchRateAeroFrame = self.mslEulerDot[1] * self.cosPhiPrime - self.mslEulerDot[2] * self.sinPhiPrime # RADIANS PER SECOND
		self.pitchRateAeroDeg = np.degrees(pitchRateAeroFrame) # DEGREES PER SECOND
		yawRateAeroFrame = self.mslEulerDot[1] * self.sinPhiPrime + self.mslEulerDot[2] * self.cosPhiPrime # RADIANS PER SECOND
		self.yawRateAeroDeg = np.degrees(yawRateAeroFrame) # DEGREES PER SECOND
		self.rollRateDeg = np.degrees(self.mslEulerDot[0]) # DEGREES PER SECOND
		self.sinOfFourTimesPhiPrime = np.sin(4 * phiPrime) # DIMENSIONLESS
		self.squaredSinOfTwoTimesPhiPrime = (np.sin(2 * phiPrime)) ** 2 # DIMENSIONLESS

	def dataLookUp(self):
		self.CA0 = self.lookUpValues["CA0"](self.mslMach) # ND
		self.CAA = self.lookUpValues["CAA"](self.mslMach) # PER DEGREE
		self.CAD = self.lookUpValues["CAD"](self.mslMach) # PER DEGREE^2
		if self.mslTof <= self.mslBurnOut:
			self.CAOFF = 0.0 # ND
		else:
			self.CAOFF = self.lookUpValues["CAOFF"](self.mslMach) # ND
		self.CYP = self.lookUpValues["CYP"](self.mslMach, self.alphaPrimeDeg)[0] # ND
		self.CYDR = self.lookUpValues["CYDR"](self.mslMach, self.alphaPrimeDeg)[0] # PER DEGREE
		self.CN0 = self.lookUpValues["CN0"](self.mslMach, self.alphaPrimeDeg)[0] # ND
		self.CNP = self.lookUpValues["CNP"](self.mslMach, self.alphaPrimeDeg)[0] # ND
		self.CNDQ = self.lookUpValues["CNDQ"](self.mslMach, self.alphaPrimeDeg)[0] # PER DEGREE
		self.CLLAP = self.lookUpValues["CLLAP"](self.mslMach, self.alphaPrimeDeg)[0] # PER DEGREE^2
		self.CLLP = self.lookUpValues["CLLP"](self.mslMach, self.alphaPrimeDeg)[0] # PER DEGREE
		self.CLLDP = self.lookUpValues["CLLDP"](self.mslMach, self.alphaPrimeDeg)[0] # PER DEGREE
		self.CLM0 = self.lookUpValues["CLM0"](self.mslMach, self.alphaPrimeDeg)[0] # ND
		self.CLMP = self.lookUpValues["CLMP"](self.mslMach, self.alphaPrimeDeg)[0] # ND
		self.CLMQ = self.lookUpValues["CLMQ"](self.mslMach)  # PER DEGREE
		self.CLMDQ = self.lookUpValues["CLMDQ"](self.mslMach, self.alphaPrimeDeg)[0] # PER DEGREE
		self.CLNP = self.lookUpValues["CLNP"](self.mslMach, self.alphaPrimeDeg)[0] # ND
		self.mass = self.lookUpValues["MASS"](self.mslTof) # KILOGRAMS
		self.unAdjThrust = self.lookUpValues["THRUST"](self.mslTof) # NEWTONS
		self.transverseMomentOfInertia = self.lookUpValues["TRANSVERSE MOI"](self.mslTof) # KILOGRAMS * METERS^2
		self.axialMomentOfInertia = self.lookUpValues["AXIAL MOI"](self.mslTof) # KILOGRAMS * METERS ^ 2
		self.cgFromNose = self.lookUpValues["CENTER OF GRAVITY"](self.mslTof) # METERS

		CN0MAX = self.lookUpValues["CN0"](self.mslMach, self.alphaPrimeMax)[0] # ND
		MAXACCEL = CN0MAX * self.q * self.mslRefArea / self.mass # METERS PER SECOND^2
		CURRENTACCEL = self.CN0 * self.q * self.mslRefArea / self.mass # METERS PER SECOND^2
		AVAILACCEL = MAXACCEL - CURRENTACCEL # METERS PER SECOND^2
		if AVAILACCEL < 0:
			self.maxAccel = 1 # METERS PER SECOND^2
		elif AVAILACCEL > self.maxAccelAllow:
			AVAILACCEL = self.maxAccelAllow # METERS PER SECOND^2
		else:
			self.maxAccel = AVAILACCEL # METERS PER SECOND^2

	def propulsion(self):
		if self.mslTof > self.mslBurnOut:
			self.thrust = 0
		else:
			self.thrust = self.unAdjThrust + (self.seaLevelPress - self.press) * self.mslExitArea # NEWTONS

	def aeroDynamicCoefficients(self):
		self.CX = self.CA0 + self.CAA * self.alphaPrimeDeg + self.CAD * (self.totalFinDeflDeg ** 2) + self.CAOFF # ND
		CYAERO = self.CYP * self.sinOfFourTimesPhiPrime + self.CYDR * self.yawDeflAeroDeg # ND
		CZAERO = self.CN0 + self.CNP * self.squaredSinOfTwoTimesPhiPrime + self.CNDQ * self.pitchDeflAeroDeg # ND
		self.CL = self.CLLAP * (self.alphaPrimeDeg ** 2) * self.sinOfFourTimesPhiPrime + self.CLLP * self.rollRateDeg * self.mslRefDiam / (2 * self.mslSpeed) + self.CLLDP * self.rollDeflDeg # ND
		CNAEROREF = self.CLNP * self.sinOfFourTimesPhiPrime + self.CLMQ * self.yawRateAeroDeg * self.mslRefDiam / (2 * self.mslSpeed) + self.CLMDQ * self.yawDeflAeroDeg # ND
		CNAERO = CNAEROREF - CYAERO * (self.launchCg - self.cgFromNose) / self.mslRefDiam # ND
		CMAEROREF = self.CLM0 + self.CLMP * self.squaredSinOfTwoTimesPhiPrime + self.CLMQ * self.pitchRateAeroDeg * self.mslRefDiam / (2 * self.mslSpeed) + self.CLMDQ * self.pitchDeflAeroDeg # ND
		CMAERO = CMAEROREF - CZAERO * (self.launchCg - self.cgFromNose) / self.mslRefDiam # ND
		self.CY = CYAERO * self.cosPhiPrime - CZAERO * self.sinPhiPrime # ND
		self.CZ = CYAERO * self.sinPhiPrime + CZAERO * self.cosPhiPrime # ND
		self.CN = CMAERO * self.sinPhiPrime + CNAERO * self.cosPhiPrime # ND
		self.CM = CMAERO * self.cosPhiPrime + CNAERO * self.sinPhiPrime # ND

	def aeroDynamicRefValues(self):
		alphaPrimeDegLookUp = None
		if self.alphaPrimeDeg > self.alphaPrimeMax - 3:
			alphaPrimeDegLookUp = self.alphaPrimeMax - 3
		else:
			alphaPrimeDegLookUp = self.alphaPrimeDeg
		alphaPrimeDegMinusThree = alphaPrimeDegLookUp - 3 # DEGREES
		alphaPrimeDegPlusThree = alphaPrimeDegLookUp + 3 # DEGREES
		CN0MIN = self.lookUpValues["CN0"](self.mslMach, alphaPrimeDegMinusThree)[0] # ND
		CN0MAX = self.lookUpValues["CN0"](self.mslMach, alphaPrimeDegPlusThree)[0] # ND
		self.refAeroData["CNA"] = (CN0MAX - CN0MIN) / (alphaPrimeDegPlusThree - alphaPrimeDegMinusThree) # PER DEGREE
		CLM0MIN = self.lookUpValues["CLM0"](self.mslMach, alphaPrimeDegMinusThree)[0] # ND
		CLM0MAX = self.lookUpValues["CLM0"](self.mslMach, alphaPrimeDegPlusThree)[0] # ND
		self.refAeroData["CMA"] = (CLM0MAX - CLM0MIN) / (alphaPrimeDegPlusThree - alphaPrimeDegMinusThree) - self.refAeroData["CNA"] * (self.launchCg - self.cgFromNose) / self.mslRefDiam # PER DEGREE
		self.refAeroData["CND"] = self.CNDQ # PER DEGREE
		self.refAeroData["CMD"] = self.CLMDQ # PER DEGREE
		self.refAeroData["CMQ"] = self.CLMQ # PER DEGREE
		self.refAeroData["CLP"] = self.CLLP # PER DEGREE
		self.refAeroData["CLD"] = self.CLLDP # PER DEGREE
		self.staticMargin = -1 * self.refAeroData["CMA"] / self.refAeroData["CNA"]

	def accelerate(self):
		axialForce = self.thrust - self.CX * self.q * self.mslRefArea + (self.gravBodyVec[0] * self.mass) # NEWTONS
		sideForce = self.CY * self.q * self.mslRefArea + (self.gravBodyVec[1] * self.mass) # NEWTONS
		normalForce = self.CZ * self.q * self.mslRefArea + (self.gravBodyVec[2] * self.mass) # NEWTONS
		self.mslBodyAcc[0] = axialForce / self.mass - (self.mslRate[1] * self.mslBodyVel[2] - self.mslRate[2] * self.mslBodyVel[1]) # METERS PER SECOND^2
		self.mslBodyAcc[1] = sideForce / self.mass - (self.mslRate[2] * self.mslBodyVel[0] - self.mslRate[0] * self.mslBodyVel[2]) # METERS PER SECOND^2
		self.mslBodyAcc[2] = normalForce / self.mass - (self.mslRate[0] * self.mslBodyVel[1] - self.mslRate[1] * self.mslBodyVel[0]) # METERS PER SECOND^2
		self.refAeroData["BODYACC"] = self.mslBodyAcc # METERS PER SECOND^2
		self.mslAcc = self.mslBodyAcc @ self.mslLocalOrient # METERS PER SECOND^2

	def rotate(self):

		rollMoment = self.CL * self.q * self.mslRefArea * self.mslRefDiam # NEWTON * METERS
		pitchMoment = self.CM * self.q * self.mslRefArea * self.mslRefDiam # NEWTON * METERS
		yawMoment = self.CN * self.q * self.mslRefArea * self.mslRefDiam # NEWTON * METERS

		newRollRateDot = rollMoment / self.axialMomentOfInertia # RADIANS PER SECOND^2
		newRollRate = self.integrate(newRollRateDot, self.mslRateDot[0], self.mslRate[0], self.integrationStep) # RADIANS PER SECOND
		self.mslRateDot[0] = newRollRateDot # RADIANS PER SECOND^2
		
		newPitchRateDot = (1 / self.transverseMomentOfInertia) * ((self.transverseMomentOfInertia - self.axialMomentOfInertia) * self.mslRate[0] * self.mslRate[2] + pitchMoment) # RADIAN PER SECOND^2
		newPitchRate = self.integrate(newPitchRateDot, self.mslRateDot[1], self.mslRate[1], self.integrationStep) # RADIANS PER SECOND
		self.mslRateDot[1] = newPitchRateDot # RADIANS PER SECOND^2
		
		newYawRateDot = (1 / self.transverseMomentOfInertia) * ((self.axialMomentOfInertia - self.transverseMomentOfInertia) * self.mslRate[0] * self.mslRate[1] + yawMoment) # RADIAN PER SECOND^2
		newYawRate = self.integrate(newYawRateDot, self.mslRateDot[2], self.mslRate[2], self.integrationStep) # RADIANS PER SECOND
		self.mslRateDot[2] = newYawRateDot # RADIANS PER SECOND^2
		
		self.mslRate[0] = newRollRate # RADIANS PER SECOND
		self.mslRate[1] = newPitchRate # RADIANS PER SECOND
		self.mslRate[2] = newYawRate # RADIANS PER SECOND

	def eulerAngles(self):

		newPhiDot = self.mslRate[0] + (self.mslRate[1] * np.sin(self.mslEuler[0]) + self.mslRate[2] * np.cos(self.mslEuler[0])) * np.tan(self.mslEuler[1])
		self.mslEuler[0] = self.integrate(newPhiDot, self.mslEulerDot[0], self.mslEuler[0], self.integrationStep)
		self.mslEulerDot[0] = newPhiDot

		newThetaDot = self.mslRate[1] * np.cos(self.mslEuler[0]) - self.mslRate[2] * np.sin(self.mslEuler[0])
		self.mslEuler[1] = self.integrate(newThetaDot, self.mslEulerDot[1], self.mslEuler[1], self.integrationStep)
		self.mslEulerDot[1] = newThetaDot

		newPsiDot = -1 * (self.mslRate[1] * np.sin(self.mslEuler[0]) + self.mslRate[2] * np.cos(self.mslEuler[0])) / np.cos(self.mslEuler[1])
		self.mslEuler[2] = self.integrate(newPsiDot, self.mslEulerDot[2], self.mslEuler[2], self.integrationStep)
		self.mslEulerDot[2] = newPsiDot

		self.mslLocalOrient = ORIENTATION_TO_LOCAL_TM(self.mslEuler[0], -self.mslEuler[1], self.mslEuler[2])

	def eulerSemiImplicitIntegration(self):
		deltaVel = self.timeStep * self.mslAcc # METERS PER SECOND
		self.mslVel += deltaVel # METERS PER SECOND
		deltaPos = self.timeStep * self.mslVel # METERS
		self.mslPos += deltaPos # METERS
		self.mslRange += la.norm(deltaPos) # METERS

	def aeroDynamicAngles(self):
		self.mslBodyVel = self.mslLocalOrient @ self.mslVel # METERS PER SECOND
		self.mslSpeed = la.norm(self.mslBodyVel) # METERS PER SECOND
		self.mslAlpha = -1 * np.arctan2(self.mslBodyVel[2], self.mslBodyVel[0]) # RADIANS
		self.mslBeta = np.arctan2(self.mslBodyVel[1], self.mslBodyVel[0])

	def intercept(self):
		self.missDistance = la.norm(self.forwardLeftUpMslToInterceptRelPos)

	def endCheck(self):
		if self.mslPos[2] < 0.0:
			self.lethality = endChecks.groundCollision
			self.go = False
		elif self.missDistance < 2.0:
			self.lethality = endChecks.intercept
			self.go = False
		elif self.forwardLeftUpMslToInterceptRelPos[0] < 0.0:
			self.lethality = endChecks.pointOfClosestApproachPassed
			self.go = False
		elif np.isnan(np.sum(self.mslPos)):
			self.lethality = endChecks.notANumber
			self.go = False
		elif self.mslTof > self.maxTime:
			self.lethality = endChecks.maxTimeExceeded
			self.go = False
		elif self.lethality == endChecks.forcedSimTermination:
			self.go = False

	def logData(self):
		self.logFile.write(f"{self.mslTof:.4f} {self.mslPos[0]} {self.mslPos[1]} {self.mslPos[2]} {self.tgtPos[0]} {self.tgtPos[1]} {self.tgtPos[2]} {self.normCommand / self.grav} {self.mslBodyAcc[2] / self.grav} {self.mslRate[1] * 57.3} {self.mslEulerDot[1] * 57.3} {self.pitchFinComm * 57.3} {self.mslAlpha * 57.3} {self.mslEuler[1] * 57.3} {self.sideCommand / self.grav} {self.mslBodyAcc[1] / self.grav} {-self.mslRate[2] * 57.3} {self.mslEulerDot[2] * 57.3} {self.yawFinComm * 57.3} {self.mslBeta * 57.3} {self.mslEuler[2] * 57.3} {self.rollComm / self.grav} {self.mslEulerDot[0] * 57.3} {self.mslRate[0] * 57.3} {self.rollDeflDeg} {self.mslEuler[0] * 57.3} {self.seekPitchErr} {self.seekYawErr} {self.staticMargin} {self.mslMach}\n")

	def fly(self):
		self.target()
		self.timeOfFlight()
		self.environment()
		self.seeker()
		self.guidance()
		self.control()
		self.actuators()
		self.aeroBallisticAngles()
		self.dataLookUp()
		self.propulsion()
		self.aeroDynamicCoefficients()
		self.aeroDynamicRefValues()
		self.accelerate()
		self.rotate()
		self.eulerAngles()
		self.eulerSemiImplicitIntegration()
		self.aeroDynamicAngles()
		self.intercept()
		self.endCheck()
		self.logData()

	def main(self):
		while self.go:
			self.fly()
			if round(self.mslTof, 3).is_integer():
				print(f"TIME {self.mslTof:.3f} : EAST {self.mslPos[0]:.2f}, NORTH {self.mslPos[1]:.2f}, UP {self.mslPos[2]:.2f}, MACH {self.mslMach:.2f}, RANGE {self.mslRange:.2f}, AOA {self.alphaPrimeDeg:.2f}")
		wallClockEnd = time.time()
		print(f"TIME {self.mslTof:.3f} : EAST {self.mslPos[0]:.2f}, NORTH {self.mslPos[1]:.2f}, UP {self.mslPos[2]:.2f}, MACH {self.mslMach:.2f}, RANGE {self.mslRange:.2f}, AOA {self.alphaPrimeDeg:.2f}")
		print(f"SIMULATION RESULT : {self.lethality.name}, MISS DISTANCE : {self.missDistance:.4f} {self.forwardLeftUpMslToInterceptRelPos} METERS")
		print(f"SIMULATION RUN TIME : {wallClockEnd - self.wallClockStart} SECONDS")



if __name__ == "__main__":
	np.set_printoptions(suppress=True, precision=4)
	x = sixDofSim()
	x.main()