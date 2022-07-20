import numpy as np
from numpy import linalg as la

def angleBetweenTwo3DVectors(arr1, arr2):
	return np.arccos(
		np.dot(arr1, arr2) / (la.norm(arr1) * la.norm(arr2))
	)