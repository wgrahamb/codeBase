# INCLUDED WITH PYTHON
import time
from enum import Enum
import copy

# PIP INSTALLED LIBRARIES
import matplotlib.pyplot as plt
import numpy as np
from numpy import array as npa
from numpy import linalg as la
import pandas as pd
import pymap3d

pymap3d.ecef2enuv()

# GRAHAM'S FUNCTIONS
from coordinateTransformations import FLIGHTPATH_TO_LOCAL_TM
from unitVector import unitvector
from returnAzAndElevation import returnAzAndElevation

# CONSTANTS.
WEII3 = 7.292115e-5 # Rotation speed of earth. Radians per second.
REARTH = 6370987.308 # Meters.
SMALL = 9.999999999999999547e-08
DEG_TO_RAD = 0.01745329251994319833

"""

TO DO:

CHECK ECEF VELOCITY CALCULATION
ECEF GRAV
CHECK
IMPLEMENT IN LOOP

"""

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

		# WAYPOINT IN ENU.
		self.WAYPOINT = npa([3000.0, 0.0, 3000.0]) # METERS

		# ROTATING EARTH #####################################################
		# STATE.
		self.TOF = 0.0 # SECONDS
		self.LETHALITY = endChecks.flying # ND

		# LLA.
		LLA0[0] = np.radians(LLA0[0])
		LLA0[1] = np.radians(LLA0[1])
		self.GEODETIC0 = LLA0
		self.GEODETIC = LLA0

		# ENU.
		self.ENUPOS0 = np.zeros(3)
		self.ENUPOS = np.zeros(3)
		self.ENUVEL = np.zeros(3)
		self.ENU_TO_FLU = np.zeros((3, 3))

		self.ENUPOS0 = self.ENUPOS
		self.ENUVEL = INPUT_ENU_VEL
		ENU_AZ, ENU_EL = returnAzAndElevation(INPUT_ENU_VEL) # RADIANS
		self.ENU_TO_FLU = FLIGHTPATH_TO_LOCAL_TM(ENU_AZ, -ENU_EL) # ND
		self.VEL_B = self.ENU_TO_FLU @ INPUT_ENU_VEL # METERS PER SECOND

		# ECEF.
		self.ECEFPOS0 = np.zeros(3)
		self.ECEFPOS = np.zeros(3)
		self.ECEFVEL = np.zeros(3)
		self.ECEF_TO_FLU = np.zeros((3, 3))
		
		self.ECEFPOS = self.LLA_TO_ECEF(LLA0)
		self.ECEFPOS0 = self.ECEFPOS
		
		ECEF_AZ, ECEF_EL = returnAzAndElevation(self.ECEFPOS) # WRONG.
		self.ECEF_TO_FLU = FLIGHTPATH_TO_LOCAL_TM(ECEF_AZ, -ECEF_EL)
		self.ECEFVEL = self.SPEED_AND_FPA_TO_CARTESIAN(la.norm(INPUT_ENU_VEL), ECEF_AZ, -ECEF_EL)

		# ECI.
		self.ECIPOS0 = np.zeros(3)
		self.ECIPOS = np.zeros(3)
		self.ECIVEL = np.zeros(3)
		self.ECI_TO_FLU = np.zeros((3, 3))

		self.ECI_TO_ECEF = self.ECI_TO_ECEF_TM(self.TOF)
		self.ECIPOS = self.ECI_TO_ECEF.transpose() @ self.ECEFPOS # STATE
		self.ECIPOS0 = self.ECIPOS
		TEMP = self.ECI_TO_ECEF.transpose() @ self.ECEFVEL
		OMEGA = npa([0.0, 0.0, WEII3])
		ECIVEL_DUE_TO_ROTATION = np.cross(OMEGA, self.ECIPOS)
		self.ECIVEL = TEMP + ECIVEL_DUE_TO_ROTATION
		self.ECI_TO_FLU = self.ECEF_TO_FLU @ self.ECI_TO_ECEF

		# Test to make sure matrices are initialized correctly.
		print("BODY VELOCITY FROM ENU VELOCITY :", self.ENU_TO_FLU @ INPUT_ENU_VEL)
		print("BODY VELOCITY FROM ECEF VELOCITY :", self.ECEF_TO_FLU @ self.ECEFVEL)
		print("BODY VELOCITY FROM ECI VELOCITY :", self.ECI_TO_FLU @ (self.ECIVEL - ECIVEL_DUE_TO_ROTATION))
		print(LLA0)
		x = self.ECI_TO_LLA(self.ECIPOS, self.TOF)
		print(x)
		y = self.LLA_TO_ECI(x, self.TOF)
		print(self.ECIPOS)
		print(y)
		print(self.ECI_TO_LLA(y, self.TOF))

		# GUIDANCE.
		self.FLU_REL_POS = self.ENU_TO_FLU @ (self.WAYPOINT - np.zeros(3)) # METERS
		self.NORM_COMM = 0.0 # METERS PER SECOND^2
		self.SIDE_COMM = 0.0 # METERS PER SECOND^2

		# DERIVATIVE.
		self.ENUACC = np.zeros(3) # METERS PER SECOND^2
		self.SPECIFIC_FORCE = np.zeros(3) # METERS PER SECOND^2

	@staticmethod
	def ECEF_DISPLACEMENT_TO_ENU(RELPOS, LAT0, LON0):
		TEMP = np.cos(LON0) * RELPOS[0] + np.sin(LON0) * RELPOS[1]
		E = -1.0 * np.sin(LON0) * RELPOS[0] + np.cos(LON0) * RELPOS[1]
		N = np.cos(LAT0) * TEMP + np.sin(LAT0) * RELPOS[2]
		U = -1.0 * np.sin(LAT0) * TEMP + np.cos(LAT0) * RELPOS[2]
		ENU = npa([E, N, U])
		return ENU

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

	@staticmethod
	def ECI_TO_ECEF_TM(TIME): # Seconds from launch.
		GW_CLONG = 0.0 # Greenwich celestial longitude at start of flight. Radians.
		WEII3 = 7.292115e-5 # Rotation speed of earth. Radians per second.
		TEI = np.eye(3) # TM
		XI = WEII3 * TIME + GW_CLONG
		SXI = np.sin(XI)
		CXI = np.cos(XI)
		TEI[0, 0] = CXI
		TEI[0, 1] = SXI
		TEI[1, 0] = -1.0 * SXI
		TEI[1, 1] = CXI
		return TEI

	# Only called once. Then in reverse order for the rest of the sim.
	@staticmethod
	def LLA_TO_ECEF(LLA): # Lat - Rads, Lon - Rads, Alt - Meters.
		REARTH = 6370987.308 # Meters.
		ECEF = np.zeros(3)
		RADIUS = -1.0 * (LLA[2] + REARTH)
		TGE = np.zeros((3, 3))
		CLON = np.cos(LLA[1])
		SLON = np.sin(LLA[1])
		CLAT = np.cos(LLA[0])
		SLAT = np.sin(LLA[0])
		TGE[0, 0] = -1.0 * SLAT * CLON
		TGE[0, 1] = -1.0 * SLAT * SLON
		TGE[0, 2] = CLAT
		TGE[1, 0] = -1.0 * SLON
		TGE[1, 1] = CLON
		TGE[1, 2] = 0.0
		TGE[2, 0] = -1.0 * CLAT * CLON
		TGE[2, 1] = -1.0 * CLAT * SLON
		TGE[2, 2] = -1.0 * SLAT
		ECEF = TGE.transpose() @ npa([0.0, 0.0, RADIUS])
		return ECEF

	@staticmethod
	def ECI_TO_LLA(ECIPOS, TIME): # Inertial Pos - Meters, Time - Seconds from launch.
		LLAREF = np.zeros(3)
		GW_CLONG = 0.0 # Greenwich celestial longitude at start of flight. Radians.
		WEII3 = 7.292115e-5 # Rotation speed of earth. Radians per second.
		SMAJOR_AXIS = 6378137.0 # Meters.
		FLATTENING = 0.00333528106000000003
		COUNT = 0
		LAT0 = 0.0
		ALAMDA = 0.0
		DBI = la.norm(ECIPOS)
		LATG = np.arcsin(ECIPOS[2] / DBI)
		LLAREF[0] = LATG
		GO = True
		while GO:
			LAT0 = LLAREF[0]
			R0 = SMAJOR_AXIS * \
				(
					1.0 - \
					(FLATTENING * (1.0 - np.cos(2.0 * LAT0)) / 2.0) + \
					(5.0 * (FLATTENING ** 2) * (1.0 - np.cos(4 * LAT0)) / 16.0)
				)
			LLAREF[2] = DBI - R0
			DD = FLATTENING * np.sin(2 * LAT0) * (1.0 - FLATTENING / 2.0 - LLAREF[2] / R0)
			LLAREF[0] = LATG + DD
			COUNT += 1
			if COUNT > 100:
				print("GEODETIC LATITUDE DOES NOT CONVERGE")
				return
			if np.abs(LLAREF[0] - LAT0) < SMALL:
				break
		SBII1 = ECIPOS[0]
		SBII2 = ECIPOS[1]
		DUM4 = np.arcsin(SBII2 / np.sqrt(SBII1 ** 2 + SBII2 ** 2))
		if SBII1 >= 0.0 and SBII2 >= 0.0:
			ALAMDA = DUM4
		if SBII1 < 0.0 and SBII2 >= 0.0:
			ALAMDA = (180.0 * DEG_TO_RAD) - DUM4
		if SBII1 < 0.0 and SBII2 < 0.0:
			ALAMDA = (180.0 * DEG_TO_RAD) - DUM4
		if SBII1 > 0.0 and SBII2 < 0.0:
			ALAMDA = (360.0 * DEG_TO_RAD) + DUM4
		LLAREF[1] = ALAMDA - WEII3 * TIME - GW_CLONG
		if LLAREF[1] > (180.0 * DEG_TO_RAD):
			TEMP = -1.0 * ((360.0 * DEG_TO_RAD) - LLAREF[1])
			LLAREF[1] = TEMP
		return LLAREF

	@staticmethod
	def LLA_TO_ECI(LLA, TIME):
		LLAREF = copy.deepcopy(LLA)
		GW_CLONG = 0.0 # Greenwich celestial longitude at start of flight. Radians.
		WEII3 = 7.292115e-5 # Rotation speed of earth. Radians per second.
		SMAJOR_AXIS = 6378137.0 # Meters.
		FLATTENING = 0.00333528106000000003
		SBII = np.zeros(3)
		SBID = np.zeros(3)
		R0 = SMAJOR_AXIS * \
			(
				1.0 - \
				(FLATTENING * (1.0 - np.cos(2.0 * LLAREF[0])) / 2.0) + \
				(5.0 * (FLATTENING ** 2) * (1.0 - np.cos(4 * LLAREF[0])) / 16.0)
			)
		DD = FLATTENING * np.sin(2 * LLAREF[0]) * (1.0 - FLATTENING / 2.0 - LLAREF[2] / R0)
		DBI = R0 + LLAREF[2]
		SBID[0] = -1.0 * DBI * np.sin(DD)
		SBID[1] = 0.0
		SBID[2] = -1.0 * DBI * np.cos(DD)
		LON_CEL = GW_CLONG + WEII3 * TIME + LLAREF[1]
		SLAT = np.sin(LLAREF[0])
		CLAT = np.cos(LLAREF[0])
		SLON = np.sin(LON_CEL)
		CLON = np.cos(LON_CEL)
		SBII[0] = -1.0 * SLAT * CLON * SBID[0] - CLAT * CLON * SBID[2]
		SBII[1] = -1.0 * SLAT * SLON * SBID[0] - CLAT * SLON * SBID[2]
		SBII[2] = CLAT * SBID[0] - SLAT * SBID[2]
		return SBII

	def guidance(self):

		# PROPORTIONAL GUIDANCE
		self.FLU_REL_POS = self.ENU_TO_FLU @ (self.WAYPOINT - self.ENUPOS) # METERS
		FLU_REL_POS_U = unitvector(self.FLU_REL_POS) # ND
		CLOSING_VEL = -1 * self.VEL_B # METERS PER SECOND
		CLOSING_SPEED = la.norm(CLOSING_VEL) # METERS PER SECOND
		TEMP1 = np.cross(self.FLU_REL_POS, CLOSING_VEL)
		TEMP2 = np.dot( self.FLU_REL_POS, self.FLU_REL_POS)
		LOS_RATE = TEMP1 / TEMP2 # RADIANS PER SECOND
		COMMAND = np.cross(-1 * 4 * CLOSING_SPEED * FLU_REL_POS_U, LOS_RATE) # METERS PER SECOND^2
		self.NORM_COMM = COMMAND[2] # METERS PER SECOND^2
		self.SIDE_COMM = COMMAND[1] # METERS PER SECOND^2
		ACC_MAG = la.norm(npa([self.SIDE_COMM, self.NORM_COMM])) # METER PER SECOND^2
		TRIG_RATIO = np.arctan2(self.NORM_COMM, self.SIDE_COMM) # ND
		MANEUVER_LIMIT = 50 # METERS PER SECOND^2
		if ACC_MAG > MANEUVER_LIMIT:
			ACC_MAG = MANEUVER_LIMIT # METERS PER SECOND^2
		self.SIDE_COMM = ACC_MAG * np.cos(TRIG_RATIO) # METERS PER SECOND^2
		self.NORM_COMM = ACC_MAG * np.sin(TRIG_RATIO) # METERS PER SECOND^2

	def derivative(self):
		self.SPECIFIC_FORCE = npa([0.0, self.SIDE_COMM, self.NORM_COMM]) # METERS PER SECOND^2
		self.ENUACC = self.SPECIFIC_FORCE @ self.ENU_TO_FLU # METERS PER SECOND^2

	def integrate(self):
		DELTA_POS = self.ENUVEL * self.DT # METERS
		self.ENUPOS += DELTA_POS # METERS
		DELTA_VEL = self.ENUACC * self.DT # METERS PER SECOND
		self.ENUVEL += DELTA_VEL # METERS PER SECOND
		self.TOF += self.DT # SECONDS.

	def attitude(self):

		ENU_AZ, ENU_EL = returnAzAndElevation(self.ENUVEL) # RADIANS
		self.ENU_TO_FLU = FLIGHTPATH_TO_LOCAL_TM(ENU_AZ, -ENU_EL) # ND

		# ROTATING EARTH HERE.

	def endCheck(self):

		self.MISS_DIST = la.norm(self.FLU_REL_POS)

		if self.ENUPOS[2] < 0.0:
			self.LETHALITY = endChecks.groundCollision
			self.GO = False
		elif self.MISS_DIST < 5.0:
			self.LETHALITY = endChecks.intercept
			self.GO = False
		elif self.FLU_REL_POS[0] < 0.0:
			self.LETHALITY = endChecks.pointOfClosestApproachPassed
			self.GO = False
		elif np.isnan(np.sum(self.ENUPOS)):
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
		self.endCheck()

	def main(self):
		while self.GO:
			self.fly()
			if round(self.TOF, 3).is_integer():
				print(f"TOF {self.TOF:.3f}, ENU {self.ENUPOS}, SPECIFIC FORCE {self.SPECIFIC_FORCE}")
		wallClockEnd = time.time()
		print(f"TOF {self.TOF:.3f}, ENU {self.ENUPOS}, SPECIFIC FORCE {self.SPECIFIC_FORCE}")
		print(f"SIMULATION RESULT : {self.LETHALITY.name}, MISS DISTANCE : {self.MISS_DIST:.4f} {self.FLU_REL_POS} METERS")
		print(f"SIMULATION RUN TIME : {wallClockEnd - self.WALL_CLOCK_START} SECONDS")



if __name__ == "__main__":
	np.set_printoptions(suppress=True, precision=2)
	x = threeDofSim(
		INPUT_ENU_VEL=npa([150.0, 50.0, 300.0]),
		LLA0=npa([38.8719, 77.0563, 0.0])
	)
	x.main()