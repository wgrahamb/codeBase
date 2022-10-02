import numpy as np
from numpy import array as npa
import numpy.linalg as la

# Input: numpy array of the float type.
def unitvector(a):
    amag = la.norm(a)
    return a / amag

# Input: must be numpy arrays, equivalent shapes, and of the float type. (position vector)
# Output: yields line of sight from a1 to a2.
def displacementUnitVec(a1, a2):
    rel = a2 - a1
    return unitvector(rel)