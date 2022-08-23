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

# CONSTANTS.
WEII3 = 7.292115e-5 # Rotation speed of earth. Radians per second.
REARTH = 6370987.308

class endChecks(Enum):
	intercept = 1
	flying = 0
	groundCollision = -1
	pointOfClosestApproachPassed = -2
	notANumber = -3
	maxTimeExceeded = -4
	forcedSimTermination = -5

class threeDofSim:

	def __init__(self, INPUT_ENU_VEL, LLA0):

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

		# SIM CONTROL.
		self.WALL_CLOCK_START = time.time()
		self.GO = True
		self.DT = 0.01 # SECONDS
		self.MAX_T = 40 # SECONDS
		
		# WAYPOINT.
		self.WAYPOINT = npa([3000.0, 0.0, 3000.0]) # METERS

		# ORIGINAL STATE.
		self.TOF = 0.0 # SECONDS
		self.LETHALITY = endChecks.flying # ND
		self.POS = np.zeros(3) # METERS
		self.VEL = INPUT_ENU_VEL # METERS PER SECOND

		# ATTITUDE.
		AZ, EL = returnAzAndElevation(self.VEL) # RADIANS
		self.ENUtoFLU = FLIGHTPATH_TO_LOCAL_TM(AZ, -EL) # ND
		self.VEL_B = self.ENUtoFLU @ self.VEL # METERS PER SECOND

		# GUIDANCE.
		self.FLU_REL_POS = self.ENUtoFLU @ (self.WAYPOINT - self.POS)
		self.NORM_COMM = 0.0 # METERS PER SECOND^2
		self.SIDE_COMM = 0.0 # METERS PER SECOND^2

		# ORIGINAL DERIVATIVE.
		self.ACC = np.zeros(3) # METERS PER SECOND^2
		self.SPECIFIC_FORCE = np.zeros(3) # METERS PER SECOND^2

		# STATE FOR ROTATING EARTH.
		self.ECIPOS = np.zeros(3)
		self.ECIVEL = np.zeros(3)

		# ROTATING EARTH.
		self.RADIUS = -1.0 * (LLA0[2] + REARTH)

		# LLA.
		self.GEODETIC = LLA0

		# ECEF.
		self.LLA_TO_ECEF = self.LAT_LON_TO_ECEF_TM(np.radians(LLA0[1]), np.radians(LLA0[0]))
		self.ECEFPOS = self.LLA_TO_ECEF.transpose() @ npa([0.0, 0.0, self.RADIUS])
		ECEFAZ, ECEFEL = returnAzAndElevation(self.ECEFPOS)
		self.ECEFtoFLU = FLIGHTPATH_TO_LOCAL_TM(ECEFAZ, -ECEFEL)
		self.ECEFVEL = self.SPEED_AND_FPA_TO_CARTESIAN(la.norm(INPUT_ENU_VEL), ECEFAZ, -ECEFEL)

		# ENU.
		# self.ENU0 = self.ECEFPOS - self.ECEFPOS0

		# ECI.
		self.ROTATION_TO_ECEF = self.ROTATION_TO_EARTH(self.TOF)
		self.ECIPOS = self.ROTATION_TO_ECEF.transpose() @ self.ECEFPOS # STATE
		TEMP1 = self.ROTATION_TO_ECEF.transpose() @ self.ECEFVEL
		TEMP2 = npa([0.0, 0.0, WEII3])
		LOCALVEL = np.cross(TEMP2, self.ECIPOS)
		self.ECIVEL = TEMP1 + LOCALVEL

		pause = 0

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

	@staticmethod
	def ROTATION_TO_EARTH(time): # Seconds from launch.
		GW_CLONG = 0.0 # Greenwich celestial longitude at start of flight. Radians.
		WEII3 = 7.292115e-5 # Rotation speed of earth. Radians per second.
		TEI = np.eye(3) # TM
		xi = WEII3 * time + GW_CLONG
		sin_xi = np.sin(xi)
		cos_xi = np.cos(xi)
		TEI[0, 0] = cos_xi
		TEI[0, 1] = sin_xi
		TEI[1, 0] = -1.0 * sin_xi
		TEI[1, 1] = cos_xi
		return TEI

	@staticmethod
	def SPEED_AND_FPA_TO_CARTESIAN(SPEED, AZ, EL): # Meters per second, radians, radians.
		RET = npa(
			[
				np.cos(EL) * np.cos(AZ),
				np.cos(EL) * np.sin(AZ),
				-1.0 * np.sin(EL)
			]
		) * SPEED # METERS / SECOND
		return RET

	def guidance(self):
		# PROPORTIONAL NAVIGATION
		self.FLU_REL_POS = self.ENUtoFLU @ (self.WAYPOINT - self.POS) # METERS
		forwardLeftUpMslToInterceptRelPosU = unitvector(self.FLU_REL_POS) # ND
		closingVel = -1 * self.VEL_B # METERS PER SECOND
		closingVelMag = la.norm(closingVel) # METERS PER SECOND
		TEMP1 = np.cross(self.FLU_REL_POS, closingVel)
		TEMP2 = np.dot( self.FLU_REL_POS, self.FLU_REL_POS)
		lineOfSightRate = TEMP1 / TEMP2 # RADIANS PER SECOND
		command = np.cross(-1 * 4 * closingVelMag * forwardLeftUpMslToInterceptRelPosU, lineOfSightRate) # METERS PER SECOND^2
		self.NORM_COMM = command[2] # METERS PER SECOND^2
		self.SIDE_COMM = command[1] # METERS PER SECOND^2
		accMag = la.norm(npa([self.SIDE_COMM, self.NORM_COMM])) # METER PER SECOND^2
		trigonometricRatio = np.arctan2(self.NORM_COMM, self.SIDE_COMM) # ND
		if accMag > 50:
			accMag = 50# METERS PER SECOND^2
		self.SIDE_COMM = accMag * np.cos(trigonometricRatio) # METERS PER SECOND^2
		self.NORM_COMM = accMag * np.sin(trigonometricRatio) # METERS PER SECOND^2

	def derivative(self):
		self.SPECIFIC_FORCE = npa([0.0, self.SIDE_COMM, self.NORM_COMM]) # METERS PER SECOND^2
		self.ACC = self.SPECIFIC_FORCE @ self.ENUtoFLU # METERS PER SECOND^2

	def integrate(self):
		deltaPos = self.VEL * self.DT # METERS
		self.POS += deltaPos # METERS
		deltaVel = self.ACC * self.DT # METERS PER SECOND
		self.VEL += deltaVel # METERS PER SECOND
		self.TOF += self.DT

	def attitude(self):

		mslAz, mslEl = returnAzAndElevation(self.VEL) # RADIANS
		self.ENUtoFLU = FLIGHTPATH_TO_LOCAL_TM(mslAz, -mslEl) # ND

		# ROTATING EARTH HERE.

	def intercept(self):
		self.missDistance = la.norm(self.FLU_REL_POS)

	def endCheck(self):
		if self.POS[2] < 0.0:
			self.LETHALITY = endChecks.groundCollision
			self.GO = False
		elif self.missDistance < 5.0:
			self.LETHALITY = endChecks.intercept
			self.GO = False
		elif self.FLU_REL_POS[0] < 0.0:
			self.LETHALITY = endChecks.pointOfClosestApproachPassed
			self.GO = False
		elif np.isnan(np.sum(self.POS)):
			self.LETHALITY = endChecks.notANumber
			self.GO = False
		elif self.TOF > self.MAX_T:
			self.LETHALITY = endChecks.maxTimeExceeded
			self.GO = False
		elif self.LETHALITY == endChecks.forcedSimTermination:
			self.GO = False

	def fly(self):
		self.guidance()
		self.derivative()
		self.integrate()
		self.attitude()
		self.intercept()
		self.endCheck()

	def main(self):
		while self.GO:
			self.fly()
			if round(self.TOF, 3).is_integer():
				print(f"TIME {self.TOF:.3f} : EAST {self.POS[0]:.2f}, NORTH {self.POS[1]:.2f}, UP {self.POS[2]:.2f}, BODY ACC {self.SPECIFIC_FORCE}")
		wallClockEnd = time.time()
		print(f"TIME {self.TOF:.3f} : EAST {self.POS[0]:.2f}, NORTH {self.POS[1]:.2f}, UP {self.POS[2]:.2f}, BODY ACC {self.SPECIFIC_FORCE}")
		print(f"SIMULATION RESULT : {self.LETHALITY.name}, MISS DISTANCE : {self.missDistance:.4f} {self.FLU_REL_POS} METERS")
		print(f"SIMULATION RUN TIME : {wallClockEnd - self.WALL_CLOCK_START} SECONDS")



if __name__ == "__main__":
	np.set_printoptions(suppress=True, precision=4)
	x = threeDofSim(
		INPUT_ENU_VEL=npa([150.0, 50.0, 300.0]),
		LLA0=npa([38.8719, 77.0563, 0.0])
	)
	x.main()