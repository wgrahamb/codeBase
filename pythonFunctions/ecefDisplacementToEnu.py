import numpy as np
from numpy import array as npa

@staticmethod
def ECEF_DISPLACEMENT_TO_ENU(RELPOS, LAT0, LON0):
	TEMP = np.cos(LON0) * RELPOS[0] + np.sin(LON0) * RELPOS[1]
	E = -1.0 * np.sin(LON0) * RELPOS[0] + np.cos(LON0) * RELPOS[1]
	N = np.cos(LAT0) * TEMP + np.sin(LAT0) * RELPOS[2]
	U = -1.0 * np.sin(LAT0) * TEMP + np.cos(LAT0) * RELPOS[2]
	ENU = npa([E, N, U])
	return ENU