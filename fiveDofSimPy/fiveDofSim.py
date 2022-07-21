import numpy as np
from numpy import array as npa
from numpy import linalg as la
from ambiance import Atmosphere as atm
from loadPickle import loadpickle as lp
from coordinateTransformations import FLIGHTPATH_TO_LOCAL_TM
from coordinateTransformations import ORIENTATION_TO_LOCAL_TM
from returnAzAndElevation import returnAzAndElevation
from projection import projection
from enum import Enum
import time

class endChecks(Enum):
	intercept = 1
	flying = 0
	groundCollision = -1
	pointOfClosestApproachPassed = -2
	notANumber = -3
	maxTimeExceeded = -4

class fiveDofInterceptor:

	def __init__(self, interceptorElevation, interceptorAzimuth, interceptorSpeed, interceptorAltitude):
		
		# STOPS NUMPY FROM PRINTING IN SCIENTIFIC NOTATION
		np.set_printoptions(suppress=True)
		
		### EAST, NORTH, UP COORDINATE SYSTEM ###

		# SIM CONTROL
		self.wallClockStart = time.time()
		self.timeStep = 0.001 # SECONDS
		self.go = True
		self.maxTime = 200 # SECONDS
		self.lethality = endChecks.flying

		# INTERCEPTOR STARTING CONDITIONS
		self.interceptorEl = np.radians(interceptorElevation) # CONVERT INPUT ELEVATION TO RADIANS
		self.interceptorAz = np.radians(interceptorAzimuth) # CONVERT INPUT AZIMUTH TO RADIANS
		self.interceptorTimeOfFlight = 0.0 # SECONDS
		self.interceptorPos = npa([0.0, 0.0, interceptorAltitude]) # METERS
		self.interceptorVel = npa(
			[
				np.cos(-self.interceptorEl) * np.cos(self.interceptorAz),
				np.cos(-self.interceptorEl) * np.sin(self.interceptorAz),
				-np.sin(-self.interceptorEl)
			]
		) * interceptorSpeed # METERS / SECOND
		self.interceptorAcc = np.zeros(3) # METERS / S^2
		self.interceptorBodyAcc =np.zeros(3) # METERS / S^2

		# TARGET DATA
		self.targetPos = npa([3000, 3000, 3000]) # METERS
		self.targetVel = npa([100, 100, 0])

		# INITIALIZE INTERCEPTOR ORIENTATION
		self.interceptorLocalOrientation = FLIGHTPATH_TO_LOCAL_TM(self.interceptorAz, -self.interceptorEl)

		# INTERCEPTOR REFERENCE VALUES
		self.interceptorRefArea = 0.01767 # M^2
		self.interceptorNozzleExitArea = 0.00948 # M^2
		self.interceptorMotorBurnOutTime = 3.02 # SECONDS
		self.aeroDynamicAngleMaxDegrees = 35.0 # DEGREES

		# LOOK UP DATA
		self.lookUps = lp("fiveDofSimPy/lookUps.pickle") # AERODYNAMIC AND PROPULSION LOOKUPS

		# INTERCEPTOR ATTITUDE >>> THIS IS A FIVE DEGREE OF FREEDOM SIMULATION. NO AERODYNAMIC BANK ANGLE.
		self.alpha = 0.0 # RADIANS >>> POSITIVE ALPHA INDICATES NOSE BELOW FREE STREAM VELOCITY
		self.beta = 0.0 # RADIANS >>> POSITIVE BETA INDICATES NOSE LEFT OF FREE STREAM VELOCITY

		# STORE SEA LEVEL ATMOSPHERE FOR REFERENCE
		self.seaLevelAtmosphere = atm(0.0)

		# SEEKER AND GUIDANCE
		self.guiding = True
		self.forwardLeftUpInterceptorToInterceptRelativeToLineOfSightVel = np.zeros(3)
		self.lineOfAttack = np.zeros(3)
		self.forwardLeftUpInterceptorToInterceptRelativeToLineOfAttackVel = np.zeros(3)
		self.timeToGo = 0.0
		self.K = 1
		self.proNavGain = 4
		self.normalAccelCommandBody = 0.0
		self.sideAccelCommandBody = 0.0

		# AUTOPILOT
		self.integrationStep = 0.002
		self.ta = 2
		self.tr = 0.1
		self.gacp = 40

		# PITCH AUTOPILOT
		self.xi = 0.0
		self.xid = 0.0
		self.ratep = 0.0
		self.ratepd = 0.0
		self.alpd = 0.0

		# YAW AUTOPILOT
		self.yi = 0.0
		self.yid = 0.0
		self.ratey = 0.0
		self.rateyd = 0.0
		self.betd = 0.0

		# INTERCEPT
		self.missDistance = 0.0

		# INITIALIZE LOG FILE AND WRITE HEADER
		self.logFile = open("log.txt", "w")
		self.logFile.write("timeOfFlight(s) posEast(m) posNorth(m) posUp(m) velEast(m/s) velNorth(m/s) velUp(m/s) accEast(m/s^2) accNorth(m/s^2) accUp(m/s^2) alpha(rads) beta(rads) normalAccCommand(m/s^2) normalAccAchieved(m/s^2) sideAccCommand(m/s^2) sideAccAchieved(m/s^2) targetEast(m) targetNorth(m) targetUp(m)\n")

	def fly(self):

		### LOOP FLOW ###
		# INTERCEPTOR TIME OF FLIGHT
		# FREE STREAM VELOCITY LOCAL ORIENTATION
		# INTERCEPTOR LOCAL ORIENTATION
		# ENVIRONMENT
		# MAX ACCELERATION ALLOWABLE
		# SEEKER AND GUIDANCE
		# NORMAL AND SIDE FORCE COEFFICIENT DERIVATIVES
		# INTEGRATION FUNCTION
		# PITCH AUTOPILOT
		# YAW AUTOPILOT
		# AERO BALLISTIC ANGLES
		# AERODYNAMIC AND PROPULSION LOOK UPS
		# AERODYNAMIC FORCE COEFFICIENT COMPUTATIONS
		# AERODYNAMIC FORCE COEFFICIENT CHECK AND BALANCE
		# INTERCEPTOR BODY ACCELERATION
		# INTERCEPTOR LOCAL ACCELERATION
		# EULER SEMI IMPLICIT INTEGRATION
		# LOG DATA

		# INTERCEPTOR TIME OF FLIGHT
		self.interceptorTimeOfFlight += self.timeStep

		# FREE STREAM VELOCITY LOCAL ORIENTATION
		freeStreamAz, freeStreamEl = returnAzAndElevation(self.interceptorVel)
		freeStreamLocalOrientation = FLIGHTPATH_TO_LOCAL_TM(freeStreamAz, -freeStreamEl)

		# INTERCEPTOR LOCAL ORIENTATION
		# ARRAY 0 >>> LOOKING DOWN THE NOZZLE OF THE INTERCEPTOR
		# ARRAY 1 >>> LOOKING DOWN THE NOZZLE OF THE INTERCEPTOR, THIS POINTS OUT THE LEFT HAND SIDE
		# ARRAY 2 >>> LOOKING DOWN THE NOZZLE OF THE INTERCEPTOR, THIS POINTS OUT THE TOP SIDE OF THE INTERCEPTOR
		####################################################################
		#                       POSITIVE NORMAL
		#                                      |
		#                                      |
		#                                      |
		#  POSITIVE SIDE ---------O--------- NEGATIVE SIDE
		#                                      |
		#                                      |
		#                                      |
		#                       NEGATIVE NORMAL
		#
		# NEGATIVE AXIS IS COMING OUT OF THE SCREEN STRAIGHT AT YOU
		# POSITIVE AXIS IS POINTING INTO THE SCREEN DIRECTLY AWAY FROM YOU
		#
		####################################################################
		attitudeToLocalTransformationMatrix = ORIENTATION_TO_LOCAL_TM(0.0, self.alpha, self.beta)
		self.interceptorLocalOrientation = attitudeToLocalTransformationMatrix @ freeStreamLocalOrientation

		# ENVIRONMENT
		standardAtmoshpere = atm(self.interceptorPos[2])
		density = standardAtmoshpere.density[0]
		gravity = standardAtmoshpere.grav_accel[0]
		gravityVec = npa([0, 0, -gravity])
		pressure = standardAtmoshpere.pressure[0]
		speedOfSound = standardAtmoshpere.speed_of_sound[0]
		freeStreamSpeed = la.norm(self.interceptorVel)
		interceptorMach = freeStreamSpeed / speedOfSound
		dynamicPressure = 0.5 * density * freeStreamSpeed * freeStreamSpeed

		# MAX ACCELERATION ALLOWABLE
		dragCoefficientMax = None
		if self.interceptorTimeOfFlight <= self.interceptorMotorBurnOutTime:
			dragCoefficientMax = self.lookUps["CD MOTOR ON"](interceptorMach, self.aeroDynamicAngleMaxDegrees)[0]
		else:
			dragCoefficientMax = self.lookUps["CD MOTOR OFF"](interceptorMach, self.aeroDynamicAngleMaxDegrees)[0]
		liftCoefficientMax = self.lookUps["CL"](interceptorMach, self.aeroDynamicAngleMaxDegrees)[0]
		mass = self.lookUps["MASS (KG)"](self.interceptorTimeOfFlight)
		maxTransverseForceCoefficient = dragCoefficientMax * np.sin(np.radians(self.aeroDynamicAngleMaxDegrees)) + liftCoefficientMax * np.cos(np.radians(self.aeroDynamicAngleMaxDegrees))
		maxNormalForce = maxTransverseForceCoefficient * dynamicPressure * self.interceptorRefArea
		maxAccelAllowable = np.abs(maxNormalForce / mass)

		# SEEKER AND GUIDANCE
		self.forwardLeftUpInterceptorToInterceptRelativePos = self.interceptorLocalOrientation @ (self.targetPos - self.interceptorPos)
		if self.guiding:
			interceptorBodyVel = self.interceptorLocalOrientation @ self.interceptorVel
			forwardLeftUpInterceptorToInterceptLineOfSight = self.forwardLeftUpInterceptorToInterceptRelativePos / la.norm(self.forwardLeftUpInterceptorToInterceptRelativePos)
			self.forwardLeftUpInterceptorToInterceptRelativeToLineOfSightVel = projection(forwardLeftUpInterceptorToInterceptLineOfSight, interceptorBodyVel)
			self.lineOfAttack = npa(
				[
					forwardLeftUpInterceptorToInterceptLineOfSight[0],
					forwardLeftUpInterceptorToInterceptLineOfSight[1],
					-0.3
				]
			)
			self.forwardLeftUpInterceptorToInterceptRelativeToLineOfAttackVel = projection(self.lineOfAttack, interceptorBodyVel)
			self.timeToGo = la.norm(self.forwardLeftUpInterceptorToInterceptRelativePos) / la.norm(self.forwardLeftUpInterceptorToInterceptRelativeToLineOfSightVel)
			if self.timeToGo > 3:
				G = 1 - np.exp(-1 * la.norm(self.forwardLeftUpInterceptorToInterceptRelativePos) / 10000)
				command = npa(
					[
						0.0,
						self.K * (self.forwardLeftUpInterceptorToInterceptRelativeToLineOfSightVel[1] + G * self.forwardLeftUpInterceptorToInterceptRelativeToLineOfAttackVel[1]),
						self.K * (self.forwardLeftUpInterceptorToInterceptRelativeToLineOfSightVel[2] + G * self.forwardLeftUpInterceptorToInterceptRelativeToLineOfAttackVel[2]),
					]
				)
				self.normalAccelCommandBody = command[2]
				self.sideAccelCommandBody = command[1]
			else:
				omega = np.cross(
					self.forwardLeftUpInterceptorToInterceptRelativePos,
					np.negative(interceptorBodyVel)
				) / np.dot(
					self.forwardLeftUpInterceptorToInterceptRelativePos,
					self.forwardLeftUpInterceptorToInterceptRelativePos
				)
				command = np.cross(
					np.negative(self.proNavGain) * la.norm(np.negative(interceptorBodyVel)) * (self.forwardLeftUpInterceptorToInterceptRelativePos / la.norm(self.forwardLeftUpInterceptorToInterceptRelativePos)),
					omega
				)
				self.normalAccelCommandBody = command[2]
				self.sideAccelCommandBody = command[1]
			accelCommandMag = la.norm(npa([self.normalAccelCommandBody, self.sideAccelCommandBody]))
			if accelCommandMag > maxAccelAllowable:
				trigonometricRatio = np.arctan2(self.normalAccelCommandBody, self.sideAccelCommandBody)
				self.normalAccelCommandBody = maxAccelAllowable * np.sin(trigonometricRatio)
				self.sideAccelCommandBody = maxAccelAllowable * np.cos(trigonometricRatio)

		# AEROBALLISTIC ANGLES
		alphaPrime = np.arccos(np.cos(self.alpha) * np.cos(self.beta))
		alphaPrimeDeg = np.degrees(alphaPrime)
		phiPrime = np.arctan2(np.tan(self.beta), np.sin(self.alpha))

		# AERODYNAMIC AND PROPULSION LOOK UPS
		dragCoefficient = None
		thrust = None
		if self.interceptorTimeOfFlight <= self.interceptorMotorBurnOutTime:
			dragCoefficient = self.lookUps["CD MOTOR ON"](interceptorMach, alphaPrimeDeg)[0]
			thrustSeaLevel = self.lookUps["THRUST (NEWTONS)"](self.interceptorTimeOfFlight)
			thrust = thrustSeaLevel + (self.seaLevelAtmosphere.pressure[0] - pressure) * self.interceptorNozzleExitArea
		else:
			dragCoefficient = self.lookUps["CD MOTOR OFF"](interceptorMach, alphaPrimeDeg)[0]
			thrust = 0.0
		liftCoefficient = self.lookUps["CL"](interceptorMach, alphaPrimeDeg)[0]
		
		# AERODYNAMIC FORCE COEFFICIENT COMPUTATIONS
		cosAlpha = np.cos(self.alpha)
		sinAlpha = np.sin(self.alpha)
		axialForceCoefficient = dragCoefficient * cosAlpha - liftCoefficient * sinAlpha
		transverseForceCoefficient = dragCoefficient * sinAlpha + liftCoefficient * cosAlpha
		normalForceCoefficient = np.abs(transverseForceCoefficient) * np.cos(phiPrime)
		sideForceCoefficient = np.negative(np.abs(transverseForceCoefficient)) * np.sin(phiPrime)

		# AERODYNAMIC FORCE COEFFICIENT CHECKS AND BALANCES
		if self.alpha >= 0.0: # NOSE BELOW FREE STREAM, NORMAL FORCE PUSHES INTERCEPTOR TOWARD THE GROUND
			normalForceCoefficient = np.negative(np.abs(normalForceCoefficient))
		elif self.alpha < 0.0: # NOSE ABOVE FREE STREAM, NORMAL FORCE PUSHES INTERCEPTOR TOWARDS SPACE
			normalForceCoefficient = np.positive(np.abs(normalForceCoefficient))
		if self.beta >= 0.0: # NOSE LEFT OF FREE STREAM, SIDE FORCE PUSHES INTERCEPTOR LEFT
			sideForceCoefficient = np.negative(np.abs(sideForceCoefficient))
		elif self.beta < 0.0: # NOSE RIGHT OF FREE STREAM, SIDE FORCE PUSHES INTERCEPTOR RIGHT
			sideForceCoefficient = np.positive(np.abs(sideForceCoefficient))
		axialForceCoefficient = np.negative(np.abs(axialForceCoefficient)) # AXIAL FORCE COEFFICIENT ALWAYS ACTS IN THE NEGATIVE AXIS

		# NORMAL AND SIDE FORCE COEFFICIENT DERIVATIVES
		absAlphaDeg = np.abs(np.degrees(self.alpha))
		absBetaDeg = np.abs(np.degrees(self.beta))
		normalForceCoefficientDerivative = None
		sideForceCoefficientDerivative = None
		if absAlphaDeg < 10:
			normalForceCoefficientDerivative = np.degrees(0.123 + 0.013 * absAlphaDeg)
		else:
			normalForceCoefficientDerivative = np.degrees(0.06 * (absAlphaDeg ** 0.625))
		if absBetaDeg < 10:
			sideForceCoefficientDerivative = np.degrees(0.123 + 0.013 * absBetaDeg)
		else:
			sideForceCoefficientDerivative = np.degrees(0.06 * (absBetaDeg ** 0.625))

		# INTEGRATION FUNCTION
		def integrate(dydx_new, dydx, y, intStep):
			return y + (dydx_new + dydx) * intStep / 2

		### TO DO : TRY TO UNDERSTAND ###
		# PITCH AUTOPILOT >>> FOR NOW ASSUME PERFECT COMMAND ACHIEVEMENT
		tip = freeStreamSpeed * mass / (thrust + dynamicPressure * self.interceptorRefArea * np.abs(normalForceCoefficientDerivative))
		fspz = dynamicPressure * self.interceptorRefArea * normalForceCoefficient / mass
		gr = self.gacp * tip * self.tr / freeStreamSpeed
		gi = gr / self.ta
		abez = self.normalAccelCommandBody
		ep = abez - fspz
		xid_new = gi * ep
		self.xi = integrate(xid_new, self.xid, self.xi, self.integrationStep)
		self.xid = xid_new
		ratepc = -1 * (ep * gr + self.xi)
		ratepd_new = (ratepc - self.ratep) / self.tr
		self.ratep = integrate(ratepd_new, self.ratepd, self.ratep, self.integrationStep)
		self.ratepd = ratepd_new
		alpd_new = (tip * self.ratep - self.alpha) / tip
		self.alpha = integrate(alpd_new, self.alpd, self.alpha, self.integrationStep)
		self.alpd = alpd_new

		### TO DO : TRY TO UNDERSTAND ###
		# YAW AUTOPILOT >>> FOR NOW ASSUME PERFECT COMMAND ACHIEVEMENT
		tiy = freeStreamSpeed * mass / (thrust + dynamicPressure * self.interceptorRefArea * np.abs(sideForceCoefficientDerivative))
		fspy = dynamicPressure * self.interceptorRefArea * sideForceCoefficient / mass
		gr = self.gacp * tiy * self.tr / freeStreamSpeed
		gi = gr / self.ta
		abey = self.sideAccelCommandBody
		ey = abey - fspy
		yid_new = gi * ey
		self.yi = integrate(yid_new, self.yid, self.yi, self.integrationStep)
		self.yid = yid_new
		rateyc = ey * gr + self.yi
		rateyd_new = (rateyc - self.ratey) / self.tr
		self.ratey = integrate(rateyd_new, self.rateyd, self.ratey, self.integrationStep)
		self.rateyd = rateyd_new
		betd_new = -1 * (tiy * self.ratey + self.beta) / tiy
		self.beta = integrate(betd_new, self.betd, self.beta, self.integrationStep)
		self.betd = betd_new

		# INTERCEPTOR BODY ACCELERATION
		axialAcc = (thrust + axialForceCoefficient * dynamicPressure * self.interceptorRefArea) / mass
		sideAcc= (sideForceCoefficient * dynamicPressure * self.interceptorRefArea) / mass
		normalAcc = (normalForceCoefficient * dynamicPressure * self.interceptorRefArea) / mass
		self.interceptorBodyAcc = npa([axialAcc, sideAcc, normalAcc])

		# INTERCEPTOR LOCAL ACCELERATION
		self.interceptorAcc = (self.interceptorBodyAcc @ self.interceptorLocalOrientation) + gravityVec

		# EULER SEMI IMPLICIT INTEGRATION
		deltaVel = self.timeStep * self.interceptorAcc
		self.interceptorVel += deltaVel
		deltaPos = self.timeStep * self.interceptorVel
		self.interceptorPos += deltaPos

		# INTERCEPT
		self.missDistance = la.norm(self.forwardLeftUpInterceptorToInterceptRelativePos)

		# END CHECKS
		if self.missDistance < 2.0:
			self.lethality = endChecks.intercept
			self.go = False
		elif self.interceptorPos[2] < 0.0:
			self.lethality = endChecks.groundCollision
			self.go = False
		elif self.forwardLeftUpInterceptorToInterceptRelativePos[0] < 0.0:
			self.lethality = endChecks.pointOfClosestApproachPassed
			self.go = False
		elif np.isnan(np.sum(self.interceptorPos)):
			self.lethality = endChecks.notANumber
			self.go = False
		elif self.interceptorTimeOfFlight > self.maxTime:
			self.lethality = endChecks.maxTimeExceeded
			self.go = False

		# LOG DATA
		dataLine = f"{self.interceptorTimeOfFlight} {self.interceptorPos[0]} {self.interceptorPos[1]} {self.interceptorPos[2]} {self.interceptorVel[0]} {self.interceptorVel[1]} {self.interceptorVel[2]} {self.interceptorAcc[0]} {self.interceptorAcc[1]} {self.interceptorAcc[2]} {self.alpha} {self.beta} {self.normalAccelCommandBody} {self.interceptorBodyAcc[2]} {self.sideAccelCommandBody} {self.interceptorBodyAcc[1]} {self.targetPos[0]} {self.targetPos[1]} {self.targetPos[2]}\n"
		self.logFile.write(dataLine)

	def main(self):
		while self.go:
			self.fly()
			if round(self.interceptorTimeOfFlight, 3).is_integer():
				print(f"TIME {self.interceptorTimeOfFlight:.3f} : EAST {self.interceptorPos[0]:.2f}, NORTH {self.interceptorPos[1]:.2f}, UP {self.interceptorPos[2]:.2f}")
		wallClockEnd = time.time()
		print(f"SIMULATION RUN TIME : {wallClockEnd - self.wallClockStart} SECONDS, RESULT : {self.lethality.name}, MISS DISTANCE : {self.missDistance} METERS")



if __name__ == "__main__":
	launchEl = 70.0 # MEASURED FROM THE HORIZON
	launchAz = 330.0 # MEASURED FROM TRUE NORTH
	launchSpeed = 1.0 # ALWAYS >= 1.0
	launchAltitude = 0.0 # ALWAYS >= 0.0
	x = fiveDofInterceptor(launchEl, launchAz, launchSpeed, launchAltitude)
	x.main()