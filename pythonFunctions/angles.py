import numpy as np

def returnAzAndElevation(arr):
	az = np.arctan2(arr[1], arr[0])
	el = np.arctan2(arr[2], np.sqrt(arr[0] ** 2 + arr[1] ** 2))
	return az, el

def returnAeroAngles(velB):

	alpha = None
	beta = None
	aoa = None
	phiPrime = None

	speed = np.linalg.norm(velB)

	if (velB[2] == 0 and velB[0] == 0):
		alpha = 0.0
	else:
		alpha = np.arctan2(velB[2], velB[0])

	if speed == 0:
		beta = 0.0
	else:
		beta = np.arcsin(velB[1] / speed)

	if speed == 0:
		aoa = 0.0
	else:
		aoa = np.arccos(velB[0] / speed)

	if speed == 0:
		phiPrime = 0.0
	else:
		phiPrime = np.arctan2(velB[2], velB[1])

	return alpha, beta
