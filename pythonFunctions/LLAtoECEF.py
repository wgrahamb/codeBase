import numpy as np
from numpy import array as npa

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