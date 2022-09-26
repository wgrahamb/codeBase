import numpy as np
from numpy import array as npa
import numpy.linalg as la

# Input: Azimuth and Elevation in radians.
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
def ATTITUDE_TO_LOCAL_TM(PHI, THETA, PSI):
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

def PHYSICAL_GIMBAL_PITCH_AND_ROLL_TO_BODY_TM(THETA, PHI):
	GH_TO_B_TM = npa(
		[
			[
				np.cos(THETA),
				np.sin(THETA) * np.sin(PHI),
				-np.sin(THETA) * np.cos(PHI)
			],
			[
				0.0,
				np.cos(PHI),
				np.sin(PHI)
			],
			[
				np.sin(THETA),
				-np.cos(THETA) * np.sin(PHI),
				np.cos(THETA) * np.cos(PHI)
			]
		]
	)
	return GH_TO_B_TM

def VIRTUAL_GIMBAL_PITCH_AND_YAW_TO_BODY_TM(THETA, PSI):
	return npa(
		[
			[
				np.cos(THETA) * np.cos(PSI),
				np.cos(THETA) * np.sin(PSI),
				-np.sin(THETA)
			],
			[
				-np.sin(PSI),
				np.cos(PSI),
				0.0
			],
			[
				np.sin(THETA) * np.cos(PSI),
				np.sin(THETA) * np.sin(PSI),
				np.cos(THETA)
			]
		]
	)
