import numpy as np
from numpy import array as npa
import pymap3d

class ROTATING_ELLIPTICAL_EARTH:

	def __init__(self, LLA0):
		self.ORIGIN = LLA0
		self.GEODETIC = LLA0
		self.ECEF = pymap3d.geodetic2ecef(
			self.GEODETIC[0],
			self.GEODETIC[1],
			self.GEODETIC[2]
		)
		self.ENU = pymap3d.ecef2enu(
			self.ECEF[0],
			self.ECEF[1],
			self.ECEF[2],
			self.GEODETIC[0],
			self.GEODETIC[1],
			self.GEODETIC[2]
		)

	def update(self):
		pass

if __name__ == "__main__":

	print("HOWDY")

	LLA = npa([33.0, 77.0, 0.0])
	x = ROTATING_ELLIPTICAL_EARTH(
		LLA0=LLA
	)