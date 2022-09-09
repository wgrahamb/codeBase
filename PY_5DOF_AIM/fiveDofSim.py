
# INCLUDED WITH PYTHON.
from enum import Enum
import time

# PIP INSTALLED LIBRARIES.
import numpy as np
from numpy import array as npa
from numpy import linalg as la
from ambiance import Atmosphere as atm
np.set_printoptions(suppress=True, precision=2)

# UTILITY.
from utility.loadPickle import loadpickle as lp
from utility.coordinateTransformations import FLIGHTPATH_TO_LOCAL_TM
from utility.coordinateTransformations import ORIENTATION_TO_LOCAL_TM
from utility.returnAzAndElevation import returnAzAndElevation
from utility.projection import projection
from utility.ATM1976 import ATM1976

class endChecks(Enum):
	intercept = 1
	flying = 0
	groundCollision = -1
	pointOfClosestApproachPassed = -2
	notANumber = -3
	maxTimeExceeded = -4

class fiveDofInterceptor:

	def __init__(
		self,
		interceptorElevation,
		interceptorAzimuth,
		interceptorSpeed,
		interceptorAltitude
	):

		############################################################################
		#
		# AUTHOR - WILSON GRAHAM BEECH
		# REFERENCE - MODELING AND SIMULATION OF AEROSPACE
		# VEHICLE DYNAMICS SECOND EDITON - PETER H. ZIPFEL
		#
		# EAST, NORTH, UP COORDINATE SYSTEM
		#
		# INTERCEPTOR LOCAL ORIENTATION
		# ARRAY 0 >>> LOOKING DOWN THE NOZZLE OF THE INTERCEPTOR
		# ARRAY 1 >>> LOOKING DOWN THE NOZZLE OF THE INTERCEPTOR,
		# THIS POINTS OUT THE LEFT HAND SIDE
		# ARRAY 2 >>> LOOKING DOWN THE NOZZLE OF THE INTERCEPTOR,
		# THIS POINTS OUT THE TOP SIDE OF THE INTERCEPTOR
		#
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
		# POSITIVE ROLL INDICATES NORMAL AXIS CLOCKWISELY
		# ROTATED FROM TWELVE O'CLOCK
		#
		# FIN ORIENTATION
		# LOOKING DOWN THE NOZZLE OF THE MISSILE
		#
		#                              FIN 4       FIN 1
		#                                         X
		#                              FIN 3       FIN 2
		#
		############################################################################

		# SIM CONTROL
		self.wallClockStart = time.time()
		self.timeStep = 0.001 # SECONDS
		self.go = True
		self.maxTime = 200 # SECONDS
		self.lethality = endChecks.flying

		# INTERCEPTOR STARTING CONDITIONS
		el = np.radians(interceptorElevation) # CONVERT INPUT ELEVATION TO RADIANS
		az = np.radians(interceptorAzimuth) # CONVERT INPUT AZIMUTH TO RADIANS
		self.ENUtoFLU = FLIGHTPATH_TO_LOCAL_TM(az, -1.0 * el)
		self.tof = 0.0 # SECONDS
		self.posEnu = npa([0.0, 0.0, interceptorAltitude]) # METERS
		self.velEnu = self.ENUtoFLU[0] * interceptorSpeed
		self.accEnu = np.zeros(3) # METERS / S^2
		self.specificForce =np.zeros(3) # METERS / S^2

		# TARGET DATA
		self.targetPos = npa([3000, 3000, 3000]) # METERS
		self.targetVel = npa([100, 100, 0])

		# INTERCEPTOR CONSTANTS
		self.refArea = 0.01767 # M^2
		self.nozzleExitArea = 0.00948 # M^2
		self.burnOut = 3.02 # SECONDS

		# LOOK UP DATA
		self.lookUps = lp("PY_5DOF_AIM/lookUps.pickle") # AERODYNAMIC AND PROPULSION LOOKUPS

		# INTERCEPTOR ATTITUDE >>> THIS IS A FIVE DEGREE OF FREEDOM SIMULATION. NO AERODYNAMIC BANK ANGLE.
		self.alpha = 0.0 # RADIANS >>> POSITIVE ALPHA INDICATES NOSE BELOW FREE STREAM VELOCITY
		self.beta = 0.0 # RADIANS >>> POSITIVE BETA INDICATES NOSE LEFT OF FREE STREAM VELOCITY

		 # ATMOSPHERE.
		self.ATMOS = ATM1976()
		self.ATMOS.update(self.posEnu[2], interceptorSpeed)
		self.RHO = self.ATMOS.rho # Kilograms per meter cubed.
		self.Q = self.ATMOS.q # Pascals.
		self.P = self.ATMOS.p # Pascals.
		self.A = self.ATMOS.a # Meters per second.
		self.G = self.ATMOS.g # Meters per second squared.
		self.mslMach = self.ATMOS.mach # Non dimensional.
		self.gravBodyVec = np.zeros(3) # METERS PER SECOND^2

		# STORE SEA LEVEL ATMOSPHERE FOR REFERENCE
		self.seaLevelAtmosphere = atm(0.0)

		# SEEKER AND GUIDANCE
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
		self.logFile = open("PY_5DOF_AIM/log.txt", "w")
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
		self.tof += self.timeStep

		# FREE STREAM VELOCITY LOCAL ORIENTATION
		freeStreamAz, freeStreamEl = returnAzAndElevation(self.velEnu)
		freeStreamLocalOrientation = FLIGHTPATH_TO_LOCAL_TM(freeStreamAz, -freeStreamEl)
		attitudeToLocalTransformationMatrix = ORIENTATION_TO_LOCAL_TM(0.0, self.alpha, self.beta)
		self.ENUtoFLU = attitudeToLocalTransformationMatrix @ freeStreamLocalOrientation

		# ATMOSPHERE
		freeStreamSpeed = la.norm(self.velEnu)
		self.ATMOS.update(self.posEnu[2], freeStreamSpeed)
		self.RHO = self.ATMOS.rho # Kilograms per meter cubed.
		self.Q = self.ATMOS.q # Pascals.
		self.P = self.ATMOS.p # Pascals.
		self.A = self.ATMOS.a # Meters per second.
		self.G = self.ATMOS.g # Meters per second squared.
		self.mslMach = self.ATMOS.mach # Non dimensional.
		self.gravBodyVec = np.zeros(3) # METERS PER SECOND^2
		gravityVec = npa([0, 0, -1.0 * self.G])

		# SEEKER AND GUIDANCE
		maxAccelAllowable = 250
		self.forwardLeftUpInterceptorToInterceptRelativePos = self.ENUtoFLU @ (self.targetPos - self.posEnu)
		interceptorBodyVel = self.ENUtoFLU @ self.velEnu
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
			TEMP1 = np.cross(self.forwardLeftUpInterceptorToInterceptRelativePos, np.negative(interceptorBodyVel))
			TEMP2 = np.dot(self.forwardLeftUpInterceptorToInterceptRelativePos, self.forwardLeftUpInterceptorToInterceptRelativePos)
			omega = TEMP1 / TEMP2
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
		mass = None
		if self.tof <= self.burnOut:
			mass = self.lookUps["MASS (KG)"](self.tof)
			dragCoefficient = self.lookUps["CD MOTOR ON"](self.mslMach, alphaPrimeDeg)[0]
			thrustSeaLevel = self.lookUps["THRUST (NEWTONS)"](self.tof)
			thrust = thrustSeaLevel + (101325 - self.P) * self.nozzleExitArea
		else:
			mass = 38.5
			dragCoefficient = self.lookUps["CD MOTOR OFF"](self.mslMach, alphaPrimeDeg)[0]
			thrust = 0.0
		liftCoefficient = self.lookUps["CL"](self.mslMach, alphaPrimeDeg)[0]

		# AERODYNAMIC TRANSLATION COEFFICIENTS
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
		tip = freeStreamSpeed * mass / (thrust + self.Q * self.refArea * np.abs(normalForceCoefficientDerivative))
		fspz = self.Q * self.refArea * normalForceCoefficient / mass
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
		tiy = freeStreamSpeed * mass / (thrust + self.Q * self.refArea * np.abs(sideForceCoefficientDerivative))
		fspy = self.Q * self.refArea * sideForceCoefficient / mass
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
		axialAcc = (thrust + axialForceCoefficient * self.Q * self.refArea) / mass
		sideAcc= (sideForceCoefficient * self.Q * self.refArea) / mass
		normalAcc = (normalForceCoefficient * self.Q * self.refArea) / mass
		self.specificForce = npa([axialAcc, sideAcc, normalAcc])

		# INTERCEPTOR LOCAL ACCELERATION
		self.accEnu = (self.specificForce @ self.ENUtoFLU) + gravityVec

		# EULER SEMI IMPLICIT INTEGRATION
		deltaPos = self.timeStep * self.velEnu
		self.posEnu += deltaPos
		deltaVel = self.timeStep * self.accEnu
		self.velEnu += deltaVel

		# INTERCEPT
		self.missDistance = la.norm(self.forwardLeftUpInterceptorToInterceptRelativePos)

		# END CHECKS
		if self.missDistance < 2.0:
			self.lethality = endChecks.intercept
			self.go = False
		elif self.posEnu[2] < 0.0:
			self.lethality = endChecks.groundCollision
			self.go = False
		elif self.forwardLeftUpInterceptorToInterceptRelativePos[0] < 0.0:
			self.lethality = endChecks.pointOfClosestApproachPassed
			self.go = False
		elif np.isnan(np.sum(self.posEnu)):
			self.lethality = endChecks.notANumber
			self.go = False
		elif self.tof > self.maxTime:
			self.lethality = endChecks.maxTimeExceeded
			self.go = False

		# LOG DATA
		dataLine = f"{self.tof} {self.posEnu[0]} {self.posEnu[1]} {self.posEnu[2]} {self.velEnu[0]} {self.velEnu[1]} {self.velEnu[2]} {self.accEnu[0]} {self.accEnu[1]} {self.accEnu[2]} {self.alpha} {self.beta} {self.normalAccelCommandBody} {self.specificForce[2]} {self.sideAccelCommandBody} {self.specificForce[1]} {self.targetPos[0]} {self.targetPos[1]} {self.targetPos[2]}\n"
		self.logFile.write(dataLine)

	def main(self):
		while self.go:
			self.fly()
			if round(self.tof, 3).is_integer():
				print(f"TIME {self.tof:.3f} : EAST {self.posEnu[0]:.2f}, NORTH {self.posEnu[1]:.2f}, UP {self.posEnu[2]:.2f}")
		wallClockEnd = time.time()
		print(f"SIMULATION RUN TIME : {wallClockEnd - self.wallClockStart} SECONDS, RESULT : {self.lethality.name}, MISS DISTANCE : {self.missDistance} METERS")



if __name__ == "__main__":
	launchEl = 50.0 # MEASURED FROM THE HORIZON
	launchAz = 30.0 # MEASURED FROM TRUE NORTH
	launchSpeed = 1.0 # ALWAYS >= 1.0
	launchAltitude = 0.0 # ALWAYS >= 0.0
	x = fiveDofInterceptor(launchEl, launchAz, launchSpeed, launchAltitude)
	x.main()