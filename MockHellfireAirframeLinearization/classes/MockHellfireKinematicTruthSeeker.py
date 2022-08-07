from utility.coordinateTransformations import FLIGHTPATH_TO_LOCAL_TM
from utility.unitVector import unitvector

import numpy as np
from numpy import array as npa
from numpy import linalg as la

class MockHellfireKinematicTruthSeeker:

	def __init__(self):

		self.ENU_TO_FLU_MATRIX = np.zeros((3, 3))
		self.MSL_TO_TGT_RELPOS = np.zeros(3)
		self.LINE_OF_SIGHT_RATE = np.zeros(3)

		print("KINEMATIC TRUTH SEEKER LOADED")

	def update(
		self,
		THETA_RADS,
		PSI_RADS,
		MSL_POS,
		MSL_VEL,
		TGT_POS,
		TGT_VEL,
		DIMENSION_FLAG
	):
		# CREATE TRANSFORMATION MATRIX
		if DIMENSION_FLAG == 2:
			self.ENU_TO_FLU_MATRIX = FLIGHTPATH_TO_LOCAL_TM(0.0, -THETA_RADS)
		else:
			self.ENU_TO_FLU_MATRIX = FLIGHTPATH_TO_LOCAL_TM(PSI_RADS, -THETA_RADS)

		# KINEMATICS.
		# CONVERT RIGHT UP INPUTS TO ENU IF NEEDED.
		N_MSL_POS = None
		N_MSL_VEL = None
		N_TGT_POS = None
		N_TGT_VEL = None
		if DIMENSION_FLAG == 2:
			N_MSL_POS = npa([MSL_POS[0], 0.0, MSL_POS[1]])
			N_MSL_VEL = npa([MSL_VEL[0], 0.0, MSL_VEL[1]])
			N_TGT_POS = npa([TGT_POS[0], 0.0, TGT_POS[1]])
			N_TGT_VEL = npa([TGT_VEL[0], 0.0, TGT_VEL[1]])
		else:
			N_MSL_POS = MSL_POS
			N_MSL_VEL = MSL_VEL
			N_TGT_POS = TGT_POS
			N_TGT_VEL = TGT_VEL

		RELPOS = N_TGT_POS - N_MSL_POS
		FLU_MSL_TO_TGT_RELPOS = self.ENU_TO_FLU_MATRIX @ RELPOS
		RELVEL = N_TGT_VEL - N_MSL_VEL
		CLOSING_VEL = self.ENU_TO_FLU_MATRIX @ RELVEL
		TEMP1 = np.cross(FLU_MSL_TO_TGT_RELPOS, CLOSING_VEL)
		TEMP2 = np.dot(FLU_MSL_TO_TGT_RELPOS, FLU_MSL_TO_TGT_RELPOS)

		self.MSL_TO_TGT_RELPOS = FLU_MSL_TO_TGT_RELPOS
		self.LINE_OF_SIGHT_RATE = TEMP1 / TEMP2

