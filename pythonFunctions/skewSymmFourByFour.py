from numpy import array as npa

def skewSymmFourByFour(arr):
	return npa(
		[
			[0, -1 * arr[0], -1 * arr[1], -1 * arr[2]],
			[arr[0], 0, arr[2], -1 * arr[1]],
			[arr[1], -1 * arr[2], 0, arr[0]],
			[arr[2], arr[1], -1 * arr[0], 0]
		]
	)