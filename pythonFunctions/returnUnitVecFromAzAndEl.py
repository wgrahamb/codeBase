import numpy as np

def returnUnitVecFromAzAndEl(az, el):
	return np.array(
		[
			np.cos(-el) * np.cos(az),
			np.cos(-el) * np.sin(az),
			-np.sin(-el)
		]
	)