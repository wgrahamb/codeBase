import numpy as np

def skewSymmetric(arr):
	return np.array(
		[
			[0, -arr[2], arr[1]],
			[arr[2], 0, -arr[0]],
			[-arr[1], arr[0], 0]
		]
	)