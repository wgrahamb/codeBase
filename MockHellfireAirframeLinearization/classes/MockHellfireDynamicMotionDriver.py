import numpy as np
from numpy import array as npa

class MockHellfireDynamicMotionDriver:

	def __init__(self, INITIAL_POSITION, INITIAL_VELOCITY):

		self.INTEGRATION_METHOD = 0 # 0 = Euler, 1 = RK2.
		self.INTEGRATION_PASS = 0

		# DERIVATIVES.
		self.EDOTDOT = 0.0
		self.THETA_DOT = 0.0 # Radians per second.
		self.NORMAL_SPECIFIC_FORCE = 0.0 # Meters per second squared.

		# STATE
		self.E = 0.0
		self.EDOT = 0.0
		self.ACCELERATION = np.zeros(2) # Meters per second squared.
		self.POSITION = INITIAL_POSITION # Meters.
		self.VELOCITY = INITIAL_VELOCITY # Meters per second.
		self.THETA = 0.0 # Radians.
		self.ALPHA_OLD = 0.0 # Radians per second.
		self.ALPHA = 0.0 # Radians.
		self.ALPHA_DOT = 0.0 # Radians per second.
		self.TOF = 0.0 # Seconds.

		print("MOCK HELLFIRE DYNAMICS DRIVER LOADED")

	def update(self, OMEGA_Z, OMEGA_AF, ZETA_AF, KR, K1, TA, K3, DEFLECTION, GRAVITY,  MAX_TIME):

		# CALCULATE NEW TIME STEP.
		DT = (MAX_TIME - self.TOF) / 100.0
		if DT > 0.01:
			DT = 0.01

		# INTEGRATE STATES.
		while self.TOF < MAX_TIME:

			# DIFFERENTIAL EQUATIONS TO SOLVE FOR THE DERIVATIVES IN THE PITCH.
			self.EDOTDOT = (OMEGA_AF ** 2) * (DEFLECTION - self.E - 2 * ZETA_AF * self.EDOT / OMEGA_AF)
			self.E += self.EDOT * DT
			self.EDOT += self.EDOTDOT * DT

			self.THETA_DOT = K3 * (self.E + TA * self.EDOT)
			self.NORMAL_SPECIFIC_FORCE = K1 * (self.E - (self.EDOTDOT / (OMEGA_Z ** 2)))

			# EQUATIONS OF MOTION.
			self.ACCELERATION = npa([0.0, self.NORMAL_SPECIFIC_FORCE * GRAVITY])

			DELTA_POSITION = self.VELOCITY * DT
			self.POSITION += DELTA_POSITION

			DELTA_VELOCITY = self.ACCELERATION * DT
			self.VELOCITY += DELTA_VELOCITY

			self.THETA += self.THETA_DOT * DT
			self.ALPHA_OLD = self.ALPHA
			self.ALPHA = np.arctan2(self.VELOCITY[1], self.VELOCITY[0])
			self.ALPHA_DOT = (self.ALPHA_OLD - self.ALPHA) * (1.0 / DT)

			# ITERATE TIME OF FLIGHT
			self.TOF += DT