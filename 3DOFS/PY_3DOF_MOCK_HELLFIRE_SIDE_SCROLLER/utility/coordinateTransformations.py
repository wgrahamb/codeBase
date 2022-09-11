import numpy as np
from numpy import array as npa
import numpy.linalg as la

# CONSTANTS.
WEII3 = 7.292115e-5 # Rotation speed of earth. Radians per second.
REARTH = 6370987.308 # Meters.
SMALL = 9.999999999999999547e-08
DEG_TO_RAD = 0.01745329251994319833

# Input: Azimuth and Elevation in radians.
def BODY_TO_RANGE_AND_ALTITUDE(THETA):
	R = npa(
		[
			[np.cos(THETA), -1.0 * np.sin(THETA)],
			[np.sin(THETA), np.cos(THETA)]
		]
	)
	return R

def FLIGHTPATH_TO_LOCAL_TM(AZIMUTH, ELEVATION):
	FP_TO_L_TM = npa(
		[
			# FORWARD
			[
				np.cos(ELEVATION) * np.cos(AZIMUTH),
				np.cos(ELEVATION) * np.sin(AZIMUTH),
				-np.sin(ELEVATION)
			],

			# LEFT
			[
				-np.sin(AZIMUTH),
				np.cos(AZIMUTH),
				0.0
			],

			# UP
			[
				np.sin(ELEVATION) * np.cos(AZIMUTH),
				np.sin(ELEVATION) * np.sin(AZIMUTH),
				np.cos(ELEVATION)
			]
		]
	)
	return FP_TO_L_TM

# Input: Phi, theta, and psi in radians.
def ORIENTATION_TO_LOCAL_TM(PHI, THETA, PSI):
	O_TO_L_TM = npa(
		[
			[
				np.cos(PSI) * np.cos(THETA),
				np.sin(PSI) * np.cos(THETA),
				-np.sin(THETA)
			],
			[
				np.cos(PSI) * np.sin(THETA) * np.sin(PHI) - np.sin(PSI) * np.cos(PHI),
				np.sin(PSI) * np.sin(THETA) * np.sin(PHI) + np.cos(PSI) * np.cos(PHI),
				np.cos(THETA) * np.sin(PHI)
			],
			[
				np.cos(PSI) * np.sin(THETA) * np.cos(PHI) + np.sin(PSI) * np.sin(PHI),
				np.sin(PSI) * np.sin(THETA) * np.cos(PHI) - np.cos(PSI) * np.sin(PHI),
				np.cos(THETA) * np.cos(PHI)
			]
		]
	)
	return O_TO_L_TM

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

def LLA_TO_ECI(LLA, TIME):
	GW_CLONG = 0.0 # Greenwich celestial longitude at start of flight. Radians.
	WEII3 = 7.292115e-5 # Rotation speed of earth. Radians per second.
	SMAJOR_AXIS = 6378137.0 # Meters.
	FLATTENING = 0.00333528106000000003
	SBII = np.zeros(3)
	SBID = np.zeros(3)
	R0 = SMAJOR_AXIS * \
		(
			1.0 - \
			(FLATTENING * (1.0 - np.cos(2.0 * LLA[0])) / 2.0) + \
			(5.0 * (FLATTENING ** 2) * (1.0 - np.cos(4 * LLA[0])) / 16.0)
		)
	DD = FLATTENING * np.sin(2 * LLA[0]) * (1.0 - FLATTENING / 2.0 - LLA[2] / R0)
	DBI = R0 + LLA[2]
	SBID[0] = -1.0 * DBI * np.sin(DD)
	SBID[1] = 0.0
	SBID[2] = -1.0 * DBI * np.cos(DD)
	LON_CEL = GW_CLONG + WEII3 * TIME + LLA[1]
	SLAT = np.sin(LLA[0])
	CLAT = np.cos(LLA[0])
	SLON = np.sin(LON_CEL)
	CLON = np.cos(LON_CEL)
	SBII[0] = -1.0 * SLAT * CLON * SBID[0] - CLAT * CLON * SBID[2]
	SBII[1] = -1.0 * SLAT * SLON * SBID[0] - CLAT * SLON * SBID[2]
	SBII[2] = CLAT * SBID[0] - SLAT * SBID[2]
	return SBII

def ECI_GRAV(ECIPOS, TIME):
	GM = 398600440000000.0
	SMAJOR_AXIS = 6378137.0 # Meters.
	C20 = -1.0 * 0.0004841668499999999772
	ECIGRAV = np.zeros(3)
	LLAREF = ECI_TO_LLA(ECIPOS, TIME)
	DBI = la.norm(ECIPOS)
	DUM1 = GM / (DBI ** 2)
	DUM2 = 3.0 * np.sqrt(5.0)
	DUM3 = (SMAJOR_AXIS / DBI) ** 2
	ECIGRAV[0] = -1.0 * DUM1 * DUM2 * C20 * DUM3 * np.sin(LLAREF[0]) * np.cos(LLAREF[0])
	ECIGRAV[1] = 0.0
	ECIGRAV[2] = DUM1 * (1.0 + (DUM2 / 2.0) * C20 * DUM3 * (3 * (np.sin(LLAREF[0]) ** 2) - 1.0))
	return ECIGRAV

def EULER_FROM_DCM(TM):
	EULER = np.zeros(3)
	R31 = TM[2, 0]
	THETA = -1.0 * np.arcsin(R31)
	R32 = TM[2, 1]
	R33 = TM[2, 2]
	PSI = np.arctan2((R32 / np.cos(THETA)), (R33 / np.cos(THETA)))
	R21 = TM[1, 0]
	R11 = TM[0, 0]
	PHI = np.arctan2((R21 / np.cos(THETA)), (R11 / np.cos(THETA)))
	EULER[0] = PHI
	EULER[1] = THETA
	EULER[2] = PSI
	return EULER

# def PHYSICAL_GIMBAL_PITCH_AND_ROLL_TO_BODY_TM(THETA, PHI):
# 	GH_TO_B_TM = npa(
# 		[
# 			[
# 				np.cos(THETA),
# 				np.sin(THETA) * np.sin(PHI),
# 				-np.sin(THETA) * np.cos(PHI)
# 			],
# 			[
# 				0.0,
# 				np.cos(PHI),
# 				np.sin(PHI)
# 			],
# 			[
# 				np.sin(THETA),
# 				-np.cos(THETA) * np.sin(PHI),
# 				np.cos(THETA) * np.cos(PHI)
# 			]
# 		]
# 	)
# 	return GH_TO_B_TM

# def VIRTUAL_GIMBAL_PITCH_AND_YAW_TO_BODY_TM(THETA, PSI):
# 	return npa(
# 		[
# 			[
# 				np.cos(THETA) * np.cos(PSI),
# 				np.cos(THETA) * np.sin(PSI),
# 				-np.sin(THETA)
# 			],
# 			[
# 				-np.sin(PSI),
# 				np.cos(PSI),
# 				0.0
# 			],
# 			[
# 				np.sin(THETA) * np.cos(PSI),
# 				np.sin(THETA) * np.sin(PSI),
# 				np.cos(THETA)
# 			]
# 		]
# 	)
