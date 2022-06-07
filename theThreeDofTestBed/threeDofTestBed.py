# INCLUDED WITH PYTHON
import time
from enum import Enum

# PIP INSTALLED LIBRARIES
import matplotlib.pyplot as plt
import numpy as np
from numpy import array as npa
from numpy import linalg as la
import pandas as pd

# GRAHAM'S FUNCTIONS
from coordinateTransformations import FLIGHTPATH_TO_LOCAL_TM
from unitVector import unitvector
from returnAzAndElevation import returnAzAndElevation

class endChecks(Enum):
	intercept = 1
	flying = 0
	groundCollision = -1
	pointOfClosestApproachPassed = -2
	notANumber = -3
	maxTimeExceeded = -4
	forcedSimTermination = -5

class threeDofSim:

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
		########################################################################################################################

		# SIM CONTROL
		self.wallClockStart = time.time()
		self.go = True
		self.timeStep = 0.001 # SECONDS
		self.maxTime = 40 # SECONDS
		
		# PREDICTED INTERCEPT POINT
		self.pip = npa([3000.0, 10000.0, 3000.0]) # METERS

		# MISSILE
		self.mslTof = 0.0 # SECONDS
		self.lethality = endChecks.flying # ND
		self.mslPos = np.zeros(3) # METERS
		self.mslVel = npa([200.0, 200.0, 200.0]) # METERS PER SECOND
		mslAz, mslEl = returnAzAndElevation(self.mslVel) # RADIANS
		self.mslLocalOrient = FLIGHTPATH_TO_LOCAL_TM(mslAz, -mslEl) # ND
		self.mslBodyVel = self.mslLocalOrient @ self.mslVel # METERS PER SECOND
		self.forwardLeftUpMslToInterceptRelPos = self.mslLocalOrient @ (self.pip - self.mslPos)
		self.normCommand = 0.0 # METERS PER SECOND^2
		self.sideCommand = 0.0 # METERS PER SECOND^2
		self.mslAcc = np.zeros(3) # METERS PER SECOND^2
		self.mslBodyAcc = np.zeros(3) # METERS PER SECOND^2

	def timeOfFlight(self):
		self.mslTof += self.timeStep

	def guidance(self):
		# PROPORTIONAL NAVIGATION
		self.forwardLeftUpMslToInterceptRelPos = self.mslLocalOrient @ (self.pip - self.mslPos)
		forwardLeftUpMslToInterceptRelPosU = unitvector(self.forwardLeftUpMslToInterceptRelPos) # ND
		closingVel = -1 * self.mslBodyVel # METERS PER SECOND
		closingVelMag = la.norm(closingVel) # METERS PER SECOND
		TEMP1 = np.cross(self.forwardLeftUpMslToInterceptRelPos, closingVel)
		TEMP2 = np.dot( self.forwardLeftUpMslToInterceptRelPos, self.forwardLeftUpMslToInterceptRelPos)
		lineOfSightRate = TEMP1 / TEMP2 # RADIANS PER SECOND
		command = np.cross(-1 * 4 * closingVelMag * forwardLeftUpMslToInterceptRelPosU, lineOfSightRate) # METERS PER SECOND^2
		self.normCommand = command[2] # METERS PER SECOND^2
		self.sideCommand = command[1] # METERS PER SECOND^2
		accMag = la.norm(npa([self.sideCommand, self.normCommand])) # METER PER SECOND^2
		trigonometricRatio = np.arctan2(self.normCommand, self.sideCommand) # ND
		if accMag > 50:
			accMag = 50# METERS PER SECOND^2
		self.sideCommand = accMag * np.cos(trigonometricRatio) # METERS PER SECOND^2
		self.normCommand = accMag * np.sin(trigonometricRatio) # METERS PER SECOND^2

	def integrate(self):
		self.mslBodyAcc = npa([0.0, self.sideCommand, self.normCommand]) # METERS PER SECOND^2
		self.mslAcc = self.mslBodyAcc @ self.mslLocalOrient # METERS PER SECOND^2
		deltaVel = self.mslAcc * self.timeStep # METERS PER SECOND
		self.mslVel += deltaVel # METERS PER SECOND
		deltaPos = self.mslVel * self.timeStep # METERS
		self.mslPos += deltaPos # METERS

	def orient(self):
		mslAz, mslEl = returnAzAndElevation(self.mslVel) # RADIANS
		self.mslLocalOrient = FLIGHTPATH_TO_LOCAL_TM(mslAz, -mslEl) # ND

	def intercept(self):
		self.missDistance = la.norm(self.forwardLeftUpMslToInterceptRelPos)

	def endCheck(self):
		if self.mslPos[2] < 0.0:
			self.lethality = endChecks.groundCollision
			self.go = False
		elif self.missDistance < 0.5:
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

	def fly(self):
		self.timeOfFlight()
		self.guidance()
		self.integrate()
		self.orient()
		self.intercept()
		self.endCheck()

	def main(self):
		while self.go:
			self.fly()
			if round(self.mslTof, 3).is_integer():
				print(f"TIME {self.mslTof:.3f} : EAST {self.mslPos[0]:.2f}, NORTH {self.mslPos[1]:.2f}, UP {self.mslPos[2]:.2f}, BODY ACC {self.mslBodyAcc}")
		wallClockEnd = time.time()
		print(f"TIME {self.mslTof:.3f} : EAST {self.mslPos[0]:.2f}, NORTH {self.mslPos[1]:.2f}, UP {self.mslPos[2]:.2f}, BODY ACC {self.mslBodyAcc}")
		print(f"SIMULATION RESULT : {self.lethality.name}, MISS DISTANCE : {self.missDistance:.4f} {self.forwardLeftUpMslToInterceptRelPos} METERS")
		print(f"SIMULATION RUN TIME : {wallClockEnd - self.wallClockStart} SECONDS")



if __name__ == "__main__":
	np.set_printoptions(suppress=True, precision=4)
	x = threeDofSim()
	x.main()