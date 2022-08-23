# INCLUDED WITH PYTHON
import time
from enum import Enum

# PIP INSTALLED LIBRARIES
import matplotlib.pyplot as plt
import numpy as np
from numpy import array as npa
from numpy import linalg as la
import pandas as pd
import pymap3d

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

	def __init__(self, mslVel, LLA0):

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
		self.timeStep = 0.01 # SECONDS
		self.maxTime = 40 # SECONDS
		
		# PREDICTED INTERCEPT POINT
		self.pip = npa([3000.0, 0.0, 3000.0]) # METERS

		# MISSILE

		# STATE.
		self.mslTof = 0.0 # SECONDS
		self.lethality = endChecks.flying # ND
		self.mslPos = np.zeros(3) # METERS
		self.mslVel = mslVel # METERS PER SECOND

		# ATTITUDE.
		mslAz, mslEl = returnAzAndElevation(self.mslVel) # RADIANS
		self.mslLocalOrient = FLIGHTPATH_TO_LOCAL_TM(mslAz, -mslEl) # ND
		self.mslBodyVel = self.mslLocalOrient @ self.mslVel # METERS PER SECOND

		# GUIDANCE.
		self.forwardLeftUpMslToInterceptRelPos = self.mslLocalOrient @ (self.pip - self.mslPos)
		self.normCommand = 0.0 # METERS PER SECOND^2
		self.sideCommand = 0.0 # METERS PER SECOND^2

		# DERIVATIVES.
		self.mslAcc = np.zeros(3) # METERS PER SECOND^2
		self.mslBodyAcc = np.zeros(3) # METERS PER SECOND^2

		# ROTATING EARTH.
		REARTH = 6370987.308
		RADIUS = self.mslPos[1] + REARTH
		ECEF = np.zeros(3)
		ECEF[2] = RADIUS
		LLA_TO_ECEF = self.LAT_LON_TO_ECEF_TM(
			np.radians(LLA0[1]),
			np.radians(LLA0[0])
		)
		ECEF_TO_LLA = LLA_TO_ECEF.transpose()
		ECEF0 = ECEF_TO_LLA @ ECEF

		print(pymap3d.geodetic2ecef(LLA0[0], LLA0[1], LLA0[2]))
		print(ECEF0)

	@staticmethod
	def LAT_LON_TO_ECEF_TM(longitude, latitude): # Radians, Radians.
		TGE = np.zeros((3, 3))
		CLON = np.cos(longitude)
		SLON = np.sin(longitude)
		CLAT = np.cos(latitude)
		SLAT = np.sin(latitude)
		TGE[0, 0] = -1.0 * SLAT * CLON
		TGE[0, 1] = -1.0 * SLAT * SLON
		TGE[0, 2] = CLAT
		TGE[1, 0] = -1.0 * SLON
		TGE[1, 1] = CLON
		TGE[1, 2] = 0.0
		TGE[2, 0] = -1.0 * CLAT * CLON
		TGE[2, 1] = -1.0 * CLAT * SLON
		TGE[2, 2] = -1.0 * SLAT
		return TGE

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

	def derivative(self):
		self.mslBodyAcc = npa([0.0, self.sideCommand, self.normCommand]) # METERS PER SECOND^2
		self.mslAcc = self.mslBodyAcc @ self.mslLocalOrient # METERS PER SECOND^2

	def integrate(self):
		deltaPos = self.mslVel * self.timeStep # METERS
		self.mslPos += deltaPos # METERS
		deltaVel = self.mslAcc * self.timeStep # METERS PER SECOND
		self.mslVel += deltaVel # METERS PER SECOND

	def orient(self):

		mslAz, mslEl = returnAzAndElevation(self.mslVel) # RADIANS
		self.mslLocalOrient = FLIGHTPATH_TO_LOCAL_TM(mslAz, -mslEl) # ND

		# ROTATING EARTH HERE.

	def intercept(self):
		self.missDistance = la.norm(self.forwardLeftUpMslToInterceptRelPos)

	def endCheck(self):
		if self.mslPos[2] < 0.0:
			self.lethality = endChecks.groundCollision
			self.go = False
		elif self.missDistance < 5.0:
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
		self.derivative()
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
	x = threeDofSim(
		mslVel=npa([150.0, 50.0, 300.0]),
		LLA0=npa([38.8719, 77.0563, 0.0])
	)
	x.main()