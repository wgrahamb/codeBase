import numpy as np

"""

NASA Earth Atmosphere Model
https://www.grc.nasa.gov/www/k-12/airplane/atmos.html

"""

class ATM_IMPERIAL:

	def __init__(self):

		self.rho = 0.0 # sl per ft^3.
		self.p = 0.0 # psf
		self.a = 0.0 # feet per s
		self.g = 0.0 # feet per s^2
		self.q = 0.0 # psf
		self.t = 0.0 # Farenheit
		self.mach = 0.0 # Non dimensional.

		self.RAD = 2.0856e+7 # feet

	def FtoK(self, F):
		return (F + 459.67) * (5.0 / 9.0)

	def update(self, ALT, SPEED): # Feet, Feet per second.

		if ALT < 0.0:
			ALT = 0.0

		if ALT < 36152.0:
			self.t = 59 - 0.00356 * ALT
			self.p = 2116.0 * (((self.t + 459.7) / 518.6) ** 5.256)
		elif 36152.0 <= ALT < 82345:
			self.t = -70.0
			self.p = 473.1 * (np.exp(1.73 - 0.000048 * ALT))
		else:
			self.t = -205.05 + 0.00164 * ALT
			self.p = 51.97 * (((self.t + 459.7) / 389.98) ** (-11.388))

		self.rho = self.p / (1718.0 * (self.t + 459.7))
		self.q = 0.5 * self.rho * SPEED * SPEED
		self.g = 32.2 * ((self.RAD / (self.RAD + ALT)) ** 2)
		self.a = (np.sqrt(self.FtoK(self.t) * 1.4 * 286)) * 3.28084
		self.mach = SPEED / self.a
