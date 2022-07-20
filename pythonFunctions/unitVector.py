import numpy as np
from numpy import array as npa
import numpy.linalg as la

# Input: numpy array of the float type.
def unitvector(a):
    amag = la.norm(a)
    return a / amag