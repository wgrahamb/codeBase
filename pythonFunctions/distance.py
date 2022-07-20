import numpy as np
from numpy import array as npa
import numpy.linalg as la

# Input: numpy arrays of float type and equivalent shape.
def DISTANCE(a1, a2):
    squared_dist = np.sum((a1 - a2) ** 2, axis=0)
    return np.sqrt(squared_dist)
