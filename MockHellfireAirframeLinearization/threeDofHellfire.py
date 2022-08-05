# Python standard library.
import time
from enum import Enum

# Pip installed libraries.
import matplotlib.pyplot as plt
import numpy as np
from numpy import array as npa
from numpy import linalg as la
from ambiance import Atmosphere as atm

# Utility.
from coordinateTransformations import FLIGHTPATH_TO_LOCAL_TM
from unitVector import unitvector
from returnAzAndElevation import returnAzAndElevation

# Simulation constants.
WALL_CLOCK_START = time.time()
TIME_STEP = 1.0 / 500.0
HALF_TIME_STEP = TIME_STEP / 2.0
MAX_TIME = 80.0

# Math constants.
STANDARD_GRAVITY = 9.81 # Meters per second squared.
SEA_LEVEL_PRESSURE = 101325 # Pascals.

# Missile constant characteristics. Rough approximation of a Hellfire missile.
BURN_TIME = 2.95 # Seconds. Hellfire.
NOZZLE_EXIT_AREA = 0.004 # Meters squared. Hellfire.
REFERENCE_DIAMETER = 0.1778 # Meters. Hellfire.
REFERENCE_LENGTH = 1.6 # Meters. Hellfire.
REFERENCE_AREA = np.pi * (REFERENCE_DIAMETER ** 2) / 4 # Meters squared. Hellfire.

#
# Author - Wilson Graham Beech.
# Reference - Modeling and Simulation of Aerospace Vehicle Dynamics, Second Edition - Peter H. Zipfel.
#
# ENU = East, North, Up Coordinate System.
# FLU = Forward, Left, Up Coordinate System.
#
# Interceptor Orientation.
# Array 0, Axis - Looking down the nozzle of the interceptor.
# Array 1, Side - Looking down the nozzle of the interceptor, this points out the left hand side.
# Array 2, Normal - Looking down the nozzle of the interceptor, this points out the top side.
#
#			Positive normal.
#				|
#				|
#				|
#	Positive side. -------O------- Negative side.
#				|
#				|
#				|
#			Negative normal.
#
# Negative axis is pointing out of the screen directly at you.
# Positive axis is pointing into the screen directly away from you.
#
# Positive alpha indicates nose below free stream velocity.
# Positive beta indicates nose left of free stream velocity.
# Positive roll indicates normal axis clockwisely rotated from twelve o'clock.
#
# Fin orientation, looking down the nozzle of the missile.
#
#		Fin 4	Fin 1
#			X
#		Fin 3	Fin 2
#

# End checks.
class endChecks(Enum):

	intercept = 1
	flying = 0
	groundCollision = -1
	pointOfClosestApproachPassed = -2
	notANumber = -3
	maxTimeExceeded = -4
	forcedSimTermination = -5

# Three dof translational hellfire.
class threeDofHellfire:

	def __init__(self, ballistic, psiDegrees, thetaDegrees, speed, waypoint):

		# Simulation control.
		self.go = True
		
		# Missile waypoint.
		self.waypoint = waypoint # Meters.

		# Guidance.
		self.guidanceNormalAccelerationCommand = 0.0 # Meters per second squared.
		self.guidanceSideAccelerationCommand = 0.0 # Meters per second squared.
		self.maneuveringLimit = 25.0 # Meters per second squared.

		# Control.

		# Atmosphere.
		atmosphere = atm(0.0)
		self.rho = atmosphere.density[0] # Kilograms per meter cubed.
		self.p = atmosphere.pressure[0] # Pascals.
		self.a = atmosphere.speed_of_sound[0] # Meters per second.
		self.gravity = atmosphere.grav_accel[0] # Meters per second squared.
		self.q = self.rho * 0.5 * speed * speed # Pascals.
		self.mach = speed / self.a # Non dimensional.

		# Mass properties and rocket motor.
		self.ISP = 325 # Seconds. Adjusted to represent a hellfire.
		self.initialTotalMass = 45 # Kilograms.
		self.finalTotalMass = 20 # Kilograms.
		self.currentTotalMass = self.initialTotalMass # Kilograms.
		self.exitVelocity = self.ISP * STANDARD_GRAVITY # Meters per second.
		self.deltaV = np.log(self.initialTotalMass / self.finalTotalMass) * self.exitVelocity # Meters per second.
		self.massFlowRate = (self.initialTotalMass - self.finalTotalMass) / BURN_TIME # Kilograms per second.
		self.thrust = 0.0
		self.FLAG = 0
		self.transverseMomentOfInertia = (self.currentTotalMass * (3 * ((0.5 * REFERENCE_DIAMETER) ** 2) + REFERENCE_LENGTH ** 2)) / (12) # Kilograms times meters squared.

		# Missile motion.
		self.timeOfFlight = 0.0 # Seconds.
		self.ENUPosition = np.zeros(3) # Meters.
		self.psiRadians = np.radians(psiDegrees)
		self.thetaRadians = np.radians(thetaDegrees)
		self.ENUToFLUMatrix = FLIGHTPATH_TO_LOCAL_TM(self.psiRadians, -self.thetaRadians) # Non dimensional.
		self.ENUVelocity = self.ENUToFLUMatrix[0] * speed # Meters per second.
		self.FLUVelocity = self.ENUToFLUMatrix @ self.ENUVelocity # Meters per second.
		self.speed = la.norm(self.FLUVelocity)
		self.FLUMslToWaypointRelPos = self.ENUToFLUMatrix @ (self.waypoint - self.ENUPosition) # Meters.
		self.ENUAcceleration = np.zeros(3) # Meters per second squared.
		self.FLUAcceleration = np.zeros(3) # Meters per second squared.

		# Log data.
		self.logFile = open("HellfireAirframeLinearization/threeDofHellfire.txt", "w")
		self.logFile.write("tof posE posN posU psiRads thetaRads velE velN velU mach speed specificForceX specificForceY specificForceZ tgtE tgtN tgtU\n")

		# Performance and termination check.
		self.ballistic = ballistic
		self.lethality = endChecks.flying # Non dimensional.

	def guidance(self):

		# True kinematic seeker.
		self.FLUMslToWaypointRelPos = self.ENUToFLUMatrix @ (self.waypoint - self.ENUPosition) # Meters.
		forwardLeftUpMslToInterceptRelPosU = unitvector(self.FLUMslToWaypointRelPos) # Non dimensional.

		# Proportional guidance.
		closingVel = -1 * self.FLUVelocity # Meters per second.
		closingVelMag = la.norm(closingVel) # Meters per second.
		GUIDANCE_TEMP1 = np.cross(self.FLUMslToWaypointRelPos, closingVel)
		GUIDANCE_TEMP2 = np.dot( self.FLUMslToWaypointRelPos, self.FLUMslToWaypointRelPos)
		lineOfSightRate = GUIDANCE_TEMP1 / GUIDANCE_TEMP2 # Per second.
		command = np.cross(-1 * 4 * closingVelMag * forwardLeftUpMslToInterceptRelPosU, lineOfSightRate) # Meters per second squared.
		self.guidanceNormalAccelerationCommand = command[2] # Meters per second squared.
		self.guidanceSideAccelerationCommand = command[1] # Meters per second squared.

		# Limit maneuver and account for gravity.
		accMag = la.norm(npa([self.guidanceSideAccelerationCommand, self.guidanceNormalAccelerationCommand])) # Meters per second squared.
		trigonometricRatio = np.arctan2(self.guidanceNormalAccelerationCommand, self.guidanceSideAccelerationCommand) # Non dimensional.
		if accMag > self.maneuveringLimit:
			accMag = self.maneuveringLimit # Meters per second squared.
		self.guidanceSideAccelerationCommand = accMag * np.cos(trigonometricRatio) # Meters per second squared.
		self.guidanceNormalAccelerationCommand = (accMag * np.sin(trigonometricRatio) + STANDARD_GRAVITY) # Meters per second squared.

	def control(self):
		pass

	def atmosphere(self):

		atmosphere = atm(self.ENUPosition[2])
		self.rho = atmosphere.density[0] # Kilograms per meter cubed.
		self.p = atmosphere.pressure[0] # Pascals.
		self.a = atmosphere.speed_of_sound[0] # Meters per second.
		self.gravity = atmosphere.grav_accel[0] # Meters per second squared.
		self.q = self.rho * 0.5 * self.speed * self.speed # Pascals.
		self.mach = self.speed / self.a # Non dimensional.

	def massPropertiesAndRocketMotor(self):

		if self.FLAG == 0:
			fuelUsed = self.massFlowRate * self.timeOfFlight
			self.currentTotalMass = (self.initialTotalMass - fuelUsed)

		if self.currentTotalMass > self.finalTotalMass:
			self.thrust = (self.ISP * self.massFlowRate * (BURN_TIME - self.timeOfFlight)) + (self.p - SEA_LEVEL_PRESSURE) * NOZZLE_EXIT_AREA
			self.transverseMomentOfInertia = (self.currentTotalMass * (3 * ((0.5 * REFERENCE_DIAMETER) ** 2) + REFERENCE_LENGTH ** 2)) / (12) # Kilograms times meters squared.
		else:
			if self.FLAG == 0:
				self.thrust = 0.0
				self.transverseMomentOfInertia = (self.currentTotalMass * (3 * ((0.5 * REFERENCE_DIAMETER) ** 2) + REFERENCE_LENGTH ** 2)) / (12) # Kilograms times meters squared.
				self.FLAG = 1

	def missileMotion(self):

		# Axial acceleration.
		rocketAcceleration = self.thrust / self.currentTotalMass
		axialAcceleration = rocketAcceleration

		# Acceleration due to gravity.
		ENUGravity = npa([0.0, 0.0, -1.0 * self.gravity]) # Meters per second squared.
		FLUGravity = self.ENUToFLUMatrix @ ENUGravity # Meters per second squared.

		# Total acceleration.
		self.FLUAcceleration = npa([axialAcceleration, self.guidanceSideAccelerationCommand, self.guidanceNormalAccelerationCommand]) + FLUGravity # Meters per second squared.
		self.ENUAcceleration = self.FLUAcceleration @ self.ENUToFLUMatrix # Meters per second squared.

		# Euler semi implicit integration.
		# Change in velocity.
		deltaVel = self.ENUAcceleration * TIME_STEP # Meters per second.
		self.ENUVelocity += deltaVel # Meters per second.
		self.FLUVelocity = self.ENUToFLUMatrix @ self.ENUVelocity # Meters per second.
		self.speed = la.norm(self.FLUVelocity) # Meters per second.

		# Change in position.
		deltaPos = self.ENUVelocity * TIME_STEP # Meters.
		self.ENUPosition += deltaPos # Meters.

		# Adjust time of flight.
		self.timeOfFlight += TIME_STEP # Seconds.

		# Attitude.
		self.psiRadians, self.thetaRadians = returnAzAndElevation(self.ENUVelocity) # Radians.
		self.ENUToFLUMatrix = FLIGHTPATH_TO_LOCAL_TM(self.psiRadians, -self.thetaRadians) # Non dimensional.

	def logData(self):
		self.logFile.write(f"{self.timeOfFlight:.4f} {self.ENUPosition[0]:.4f} {self.ENUPosition[1]:.4f} {self.ENUPosition[2]:.4f} {self.psiRadians:.4f} {self.thetaRadians:.4f} {self.ENUVelocity[0]:.4f} {self.ENUVelocity[1]:.4f} {self.ENUVelocity[2]:.4f} {self.mach:.4f} {self.speed:.4f} {self.FLUAcceleration[0]:.4f} {self.FLUAcceleration[1]:.4f} {self.FLUAcceleration[2]:.4f} {self.waypoint[0]:.4f} {self.waypoint[1]:.4f} {self.waypoint[2]:.4f}\n")

	def performanceAndTerminationCheck(self):

		self.missDistance = la.norm(self.FLUMslToWaypointRelPos)

		if self.ENUPosition[2] < 0.0:
			self.lethality = endChecks.groundCollision
			self.go = False
		elif np.isnan(np.sum(self.ENUPosition)):
			self.lethality = endChecks.notANumber
			self.go = False
		elif self.timeOfFlight > MAX_TIME:
			self.lethality = endChecks.maxTimeExceeded
			self.go = False
		elif self.lethality == endChecks.forcedSimTermination:
			self.go = False
		if not self.ballistic:
			if self.missDistance < 5.0:
				self.lethality = endChecks.intercept
				self.go = False
			elif self.FLUMslToWaypointRelPos[0] < 0.0:
				self.lethality = endChecks.pointOfClosestApproachPassed
				self.go = False

	def fly(self):

		# Guidance, navigation, and control.
		if not self.ballistic:
			self.guidance()
			self.control()

		# Dynamics.
		self.atmosphere()
		self.massPropertiesAndRocketMotor()
		self.missileMotion()

		# Overhead.
		self.logData()
		self.performanceAndTerminationCheck()

	def main(self):

		while self.go:
			self.fly()
			if round(self.timeOfFlight, 3).is_integer():
				print(f"TIME {self.timeOfFlight:.3f} : ENU {self.ENUPosition}, SPECIFIC FORCE {self.FLUAcceleration}, MACH {self.mach:.4f}, SPEED {self.speed:.4f}")
		wallClockEnd = time.time()
		print(f"TIME {self.timeOfFlight:.3f} : ENU {self.ENUPosition}, SPECIFIC FORCE {self.FLUAcceleration}, MACH {self.mach:.4f}")
		print(f"SIMULATION RESULT : {self.lethality.name}, MISS DISTANCE : {self.missDistance:.4f} {self.FLUMslToWaypointRelPos} METERS")
		print(f"SIMULATION RUN TIME : {wallClockEnd - WALL_CLOCK_START} SECONDS")



if __name__ == "__main__":

	np.set_printoptions(suppress=True, precision=4)
	ballistic = False
	psiDegrees = 0.0
	thetaDegrees = 55.0
	speed = 1.0
	waypoint = npa([3000.0, 0.0, 3000.0])
	x = threeDofHellfire(
		ballistic=ballistic,
		psiDegrees=psiDegrees,
		thetaDegrees=thetaDegrees,
		speed=speed,
		waypoint=waypoint
	)
	x.main()
