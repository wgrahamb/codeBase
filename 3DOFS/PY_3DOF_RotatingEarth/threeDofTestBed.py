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

# GRAHAM'S FUNCTIONS
from coordinateTransformations import FLIGHTPATH_TO_LOCAL_TM
from coordinateTransformations import ORIENTATION_TO_LOCAL_TM
from unitVector import unitvector
from returnAzAndElevation import returnAzAndElevation
import loggingFxns as lf

# CONSTANTS.
WEII3 = 7.292115e-5 # Rotation speed of earth. Radians per second.
REARTH = 6370987.308 # Meters.
SMALL = 9.999999999999999547e-08
DEG_TO_RAD = 0.01745329251994319833

class endChecks(Enum):
	intercept = 1
	flying = 0
	groundCollision = -1
	pointOfClosestApproachPassed = -2
	notANumber = -3
	maxTimeExceeded = -4
	forcedSimTermination = -5

class threeDofSim:

	def __init__(self, ID, INPUT_ENU_VEL, INPUT_LLA0, ROTATING_EARTH_FLAG):

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

		self.ROTATING_EARTH = ROTATING_EARTH_FLAG
		if self.ROTATING_EARTH:
			print(ID)
		else:
			print(ID)

		# STATE.
		self.TOF = 0.0 # SECONDS
		self.LETHALITY = endChecks.flying # ND
		self.STATE = {
			"TOF": None,
			"E": None,
			"N": None,
			"U": None,
			"E_V": None,
			"N_V": None,
			"U_V": None,
			"U_DOT": None,
			"V_DOT": None,
			"W_DOT": None,
			"TGT_E": None,
			"TGT_N": None,
			"TGT_U": None,
			"LETHALITY": None
		}

		# LLA.
		self.GEODETIC0 = np.zeros(3)
		self.GEODETIC = np.zeros(3)
		
		self.GEODETIC = npa([np.radians(INPUT_LLA0[0]), np.radians(INPUT_LLA0[1]), INPUT_LLA0[2]]) # Convert to lat, lon to radians.
		self.GEODETIC0 = self.GEODETIC

		# ENU.
		self.ENUPOS0 = np.zeros(3)
		self.ENUPOS = np.zeros(3)
		self.ENUVEL = np.zeros(3)
		self.ENU_TO_FLU = np.zeros((3, 3))

		self.ENUPOS = np.zeros(3)
		self.ENUPOS0 = self.ENUPOS
		self.ENUVEL = INPUT_ENU_VEL
		ENU_AZ, ENU_EL = returnAzAndElevation(INPUT_ENU_VEL) # RADIANS
		self.ENU_TO_FLU = FLIGHTPATH_TO_LOCAL_TM(ENU_AZ, -ENU_EL) # ND
		self.VEL_B = self.ENU_TO_FLU @ INPUT_ENU_VEL # METERS PER SECOND

		# ECEF.
		self.ECEFPOS0 = np.zeros(3)
		self.ECEFPOS = np.zeros(3)
		self.ECEFVEL = np.zeros(3)
		self.ECEF_TO_ENU = np.zeros((3, 3))
		self.ECEF_TO_FLU = np.zeros((3, 3))

		self.ECEFPOS = self.LLA_TO_ECI(self.GEODETIC, self.TOF) # Can use this function for initial earth position.
		self.ECEFPOS0 = self.ECEFPOS
		self.ECEF_TO_ENU = ORIENTATION_TO_LOCAL_TM(
			0.0,
			(np.pi / 2.0) + self.GEODETIC[0],
			self.GEODETIC[1]
		)
		self.ECEF_TO_FLU = self.ENU_TO_FLU @ self.ECEF_TO_ENU
		self.ECEFVEL = self.VEL_B @ self.ECEF_TO_FLU

		# ECI.
		self.ECIPOS0 = np.zeros(3)
		self.ECIPOS = np.zeros(3)
		self.ECIVEL = np.zeros(3)
		self.ECI_TO_ECEF = np.zeros((3, 3))
		self.ECI_TO_FLU = np.zeros((3, 3))

		self.ECIPOS = self.LLA_TO_ECI(self.GEODETIC, self.TOF)
		self.ECI_TO_ECEF = self.ECI_TO_ECEF_TM(self.TOF)
		TEMP = self.ECI_TO_ECEF.transpose() @ self.ECEFVEL
		OMEGA = npa([0.0, 0.0, WEII3])
		ECIVEL_DUE_TO_ROTATION = np.cross(OMEGA, self.ECIPOS)
		self.ECIVEL = TEMP + ECIVEL_DUE_TO_ROTATION
		self.ECI_TO_FLU = self.ECEF_TO_FLU @ self.ECI_TO_ECEF

		# # Test to make sure matrices are initialized correctly.
		# print("\n")
		# print("COORDINATE TRANSFORMATIONS")
		# print("BODY VELOCITY FROM ENU VELOCITY :", self.ENU_TO_FLU @ INPUT_ENU_VEL)
		# print("BODY VELOCITY FROM ECEF VELOCITY :", self.ECEF_TO_FLU @ self.ECEFVEL)
		# print("BODY VELOCITY FROM ECI VELOCITY :", self.ECI_TO_FLU @ (self.ECIVEL - ECIVEL_DUE_TO_ROTATION))
		# print("INPUT LLA0 :", self.GEODETIC)
		# print("CALCULATED ECEF FROM INPUT LLA0 :", self.ECEFPOS)
		# print("CALCULATED ECI FROM INPUT LLA0 :", self.ECIPOS)
		# x = self.ECI_TO_LLA(self.ECIPOS, self.TOF)
		# print("CALCULATED LLA0 FROM CALCULATED ECI, WITH RESPECT TO ELLIPSOID :", x)
		# y = self.LLA_TO_ECI(x, self.TOF)
		# print("CALCULATED ECI FROM CALCULATED LLA0 :", y)
		# z = self.ECI_TO_LLA(y, self.TOF)
		# print("CALCULATED LLA0 FROM CALCULATED ECI :", z)
		# ECIGRAV = self.ECI_GRAV(self.ECIPOS, self.TOF)
		# print("CALCULATED ECI GRAVITY VEC :", ECIGRAV)
		# print("\n")

		# GUIDANCE.
		self.FLU_REL_POS = self.ENU_TO_FLU @ (self.WAYPOINT - self.ENUPOS) # METERS
		self.NORM_COMM = 0.0 # METERS PER SECOND^2
		self.SIDE_COMM = 0.0 # METERS PER SECOND^2

		# DERIVATIVE.
		self.ECIACC = np.zeros(3) # METERS PER SECOND^2
		self.ENUACC = np.zeros(3) # METERS PER SECOND^2
		self.SPECIFIC_FORCE = np.zeros(3) # METERS PER SECOND^2

		# LOGGING.
		self.STATE = self.populateState()
		self.LOGFILE = open(f"3DOFS/PY_3DOF_RotatingEarth/output/{ID}.txt", "w")
		lf.writeHeader(self.STATE, self.LOGFILE)
		lf.writeData(self.STATE, self.LOGFILE)

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

	@staticmethod
	def ECI_GRAV(ECIPOS, TIME):
		GM = 398600440000000.0
		SMAJOR_AXIS = 6378137.0 # Meters.
		C20 = -1.0 * 0.0004841668499999999772
		ECIGRAV = np.zeros(3)
		LLAREF = threeDofSim.ECI_TO_LLA(ECIPOS, TIME)
		DBI = la.norm(ECIPOS)
		DUM1 = GM / (DBI ** 2)
		DUM2 = 3.0 * np.sqrt(5.0)
		DUM3 = (SMAJOR_AXIS / DBI) ** 2
		ECIGRAV[0] = -1.0 * DUM1 * DUM2 * C20 * DUM3 * np.sin(LLAREF[0]) * np.cos(LLAREF[0])
		ECIGRAV[1] = 0.0
		ECIGRAV[2] = DUM1 * (1.0 + (DUM2 / 2.0) * C20 * DUM3 * (3 * (np.sin(LLAREF[0] ** 2)) - 1.0))
		return ECIGRAV

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

	def populateState(self):
		STATE = {
			"TOF": self.TOF,
			"E": self.ENUPOS[0],
			"N": self.ENUPOS[1],
			"U": self.ENUPOS[2],
			"E_V": self.ENUVEL[0],
			"N_V": self.ENUVEL[1],
			"U_V": self.ENUVEL[2],
			"U_DOT": self.SPECIFIC_FORCE[0],
			"V_DOT": self.SPECIFIC_FORCE[1],
			"W_DOT": self.SPECIFIC_FORCE[2],
			"TGT_E": self.WAYPOINT[0],
			"TGT_N": self.WAYPOINT[1],
			"TGT_U": self.WAYPOINT[2],
			"LETHALITY": self.LETHALITY.name

		}
		return STATE

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

	def ECIDerivative(self):

		# Derivative calculated in ECI.
		self.SPECIFIC_FORCE = npa([0.0, self.SIDE_COMM, self.NORM_COMM]) # METERS PER SECOND^2
		self.ECIACC = (self.SPECIFIC_FORCE @ self.ECI_TO_FLU) # METERS PER SECOND^2

	def ECIIntegrate(self):

		# STATE UPDATED IN THIS FUNCTION. IN ORDER.
		# self.ECIPOS = np.zeros(3)
		# self.ECIVEL = np.zeros(3)

		# Euler Integration.
		DELTA_POS = self.ECIVEL * self.DT # METERS
		self.ECIPOS += DELTA_POS # METERS
		DELTA_VEL = self.ECIACC * self.DT # METERS PER SECOND
		self.ECIVEL += DELTA_VEL # METERS PER SECOND
		self.TOF += self.DT # SECONDS.

	def ECIAttitude(self):

		# STATE UPDATED IN THIS FUNCTION. IN ORDER.
		# self.ECI_TO_ECEF = np.zeros((3, 3))
		# self.GEODETIC = np.zeros(3)
		# self.ECEFPOS = np.zeros(3)
		# self.ECEFVEL = np.zeros(3)
		# self.ECEF_TO_ENU = np.zeros((3, 3))
		# self.ENUPOS = np.zeros(3)
		# self.ENUVEL = np.zeros(3)
		# self.ENU_TO_FLU = np.zeros((3, 3))
		# self.ECEF_TO_FLU = np.zeros((3, 3))
		# self.ECI_TO_FLU = np.zeros((3, 3))

		# ECI TO ECEF MATRIX.
		self.ECI_TO_ECEF = self.ECI_TO_ECEF_TM(self.TOF)
		OMEGA = npa([0.0, 0.0, WEII3])
		ECIVEL_DUE_TO_ROTATION = np.cross(OMEGA, self.ECIPOS)

		# LLA STATE.
		self.GEODETIC = self.ECI_TO_LLA(self.ECIPOS, self.TOF)

		# ECEF STATE AND ECEF TO ENU MATRIX.
		self.ECEFPOS = self.ECI_TO_ECEF @ self.ECIPOS
		self.ECEFVEL = self.ECI_TO_ECEF @ (self.ECIVEL - ECIVEL_DUE_TO_ROTATION)
		self.ECEF_TO_ENU = ORIENTATION_TO_LOCAL_TM(
			0.0,
			(np.pi / 2.0) + self.GEODETIC[0],
			self.GEODETIC[1]
		)

		# ENU STATE AND ENU TO BODY MATRIX..
		self.ENUPOS = self.ECEF_TO_ENU @ (self.ECEFPOS - self.ECEFPOS0)
		self.ENUVEL = self.ECEF_TO_ENU @ self.ECEFVEL
		ENU_AZ, ENU_EL = returnAzAndElevation(self.ENUVEL)
		self.ENU_TO_FLU = FLIGHTPATH_TO_LOCAL_TM(ENU_AZ, -1.0 * ENU_EL)
		self.VEL_B = self.ENU_TO_FLU @ self.ENUVEL

		# ECEF TO BODY MATRIX.
		self.ECEF_TO_FLU = self.ENU_TO_FLU @ self.ECEF_TO_ENU

		# ECI TO BODY MATRIX.
		self.ECI_TO_FLU = self.ECEF_TO_FLU @ self.ECI_TO_ECEF

	def ENUDerivative(self):

		# Derivative calculated in ENU.
		self.SPECIFIC_FORCE = npa([0.0, self.SIDE_COMM, self.NORM_COMM]) # METERS PER SECOND^2
		self.ENUACC = (self.SPECIFIC_FORCE @ self.ENU_TO_FLU) # METERS PER SECOND^2

	def ENUIntegrate(self):

		# Euler Integration.
		DELTA_POS = self.ENUVEL * self.DT # METERS
		self.ENUPOS += DELTA_POS # METERS
		DELTA_VEL = self.ENUACC * self.DT # METERS PER SECOND
		self.ENUVEL += DELTA_VEL # METERS PER SECOND
		self.TOF += self.DT # SECONDS.

	def ENUAttitude(self):

		# Attitude.
		ENU_AZ, ENU_EL = returnAzAndElevation(self.ENUVEL) # RADIANS
		self.ENU_TO_FLU = FLIGHTPATH_TO_LOCAL_TM(ENU_AZ, -ENU_EL) # ND
		self.VEL_B = self.ENU_TO_FLU @ self.ENUVEL

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
		if self.ROTATING_EARTH:
			self.ECIDerivative()
			self.ECIIntegrate()
			self.ECIAttitude()
		else:
			self.ENUDerivative()
			self.ENUIntegrate()
			self.ENUAttitude()
		self.endCheck()
		self.STATE = self.populateState()
		lf.writeData(self.STATE, self.LOGFILE)

	def main(self):
		while self.GO:
			self.fly()
			if round(self.TOF, 3).is_integer():
				print(f"TOF {self.TOF:.2f}, ENU {self.ENUPOS}, SPECIFIC FORCE {self.SPECIFIC_FORCE}")
		wallClockEnd = time.time()
		print(f"TOF {self.TOF:.2f}, ENU {self.ENUPOS}, SPECIFIC FORCE {self.SPECIFIC_FORCE}")
		print(f"SIMULATION RESULT : {self.LETHALITY.name}, MISS DISTANCE : {self.MISS_DIST:.2f} {self.FLU_REL_POS} METERS")
		print(f"SIMULATION RUN TIME : {wallClockEnd - self.WALL_CLOCK_START} SECONDS")
		print("\n")



if __name__ == "__main__":

	np.set_printoptions(suppress=True, precision=2)

	x = threeDofSim(
		ID = "ROTATING_EARTH",
		INPUT_ENU_VEL=npa([150.0, 50.0, 300.0]),
		INPUT_LLA0=npa([38.8719, 77.0563, 0.0]),
		ROTATING_EARTH_FLAG=True
	)
	x.main()

	y = threeDofSim(
		ID = "LOCAL_LEVEL_EARTH",
		INPUT_ENU_VEL=npa([150.0, 50.0, 300.0]),
		INPUT_LLA0=npa([38.8719, 77.0563, 0.0]),
		ROTATING_EARTH_FLAG=False
	)
	y.main()
