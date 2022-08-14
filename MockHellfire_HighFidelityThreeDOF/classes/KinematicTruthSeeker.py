from utility.coordinateTransformations import FLIGHTPATH_TO_LOCAL_TM
from utility.unitVector import unitvector

import numpy as np
from numpy import array as npa
from numpy import linalg as la

class KinematicTruthSeeker:

	def __init__(self):

		self.TIME = 0.0
		self.TIME_STEP = (1.0 / 2400.0)
		self.NEXT_UPDATE_TIME = self.TIME + self.TIME_STEP

		self.BODY_TO_R_A_MATRIX = np.zeros((2, 2))
		self.CLOSING_SPEED = 0.0
		self.LINE_OF_SIGHT_RATE = np.zeros(2)

		print("KINEMATIC TRUTH SEEKER LOADED")

	def update(
		self,
		BODY_TO_R_A_MATRIX,
		MSL_POS,
		MSL_VEL,
		TGT_POS,
		TGT_VEL
	):

		# Guidance.
		relPos = TGT_POS - MSL_POS
		relVel = TGT_VEL - MSL_VEL
		velU = unitvector(MSL_VEL)
		velMag = la.norm(MSL_VEL)
		mslLocalOrient = BODY_TO_R_A_MATRIX
		rightUpInterceptorToIntercept = mslLocalOrient @ relPos
		rightUpInterceptorToInterceptU = unitvector(rightUpInterceptorToIntercept)
		rightUpInterceptorToInterceptMag = la.norm(rightUpInterceptorToIntercept)
		rightUpInterceptorToInterceptVel = mslLocalOrient @ relVel
		rightUpInterceptorToInterceptVelU = unitvector(rightUpInterceptorToInterceptVel)
		rightUpInterceptorToInterceptVelMag = la.norm(rightUpInterceptorToInterceptVel)
		T1 = np.cross(rightUpInterceptorToIntercept, rightUpInterceptorToInterceptVel)
		T2 = np.dot(rightUpInterceptorToIntercept, rightUpInterceptorToIntercept)
		omega = T1 / T2

		self.LINE_OF_SIGHT_RATE = omega
		self.CLOSING_SPEED = rightUpInterceptorToInterceptVelMag

		self.TIME += self.TIME_STEP
		self.NEXT_UPDATE_TIME = self.TIME + self.TIME_STEP