import numpy as np
from numpy import array as npa

@staticmethod
def SPEED_AND_FPA_TO_CARTESIAN(SPEED, AZ, EL): # Meters per second, radians, radians.
	RET = npa(
		[
			np.cos(EL) * np.cos(AZ),
			np.cos(EL) * np.sin(AZ),
			-1.0 * np.sin(EL)
		]
	) * SPEED # METERS / SECOND
	return RET