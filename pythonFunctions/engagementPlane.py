import numpy as np
from numpy import array as npa
import unitVector

# Input: must be numpy arrays, equivalent shapes, and of the float type.
# Input format: numpy arrays -> [x, y, z, vx, vy, vz] (final states of an interceptor and a threat)
# Input final state of the threat first, then final state of the interceptor.
def ep(tfs, ifs):
    # do the above because at final states, they may have both crossed the plane
    # last position of interceptor
    p1 = npa([
        tfs[0] - ifs[3],
        tfs[1] - ifs[4],
        tfs[2] - ifs[5]
    ])
    # last position of threat
    p2 = npa([
        tfs[0] - tfs[3],
        tfs[1] - tfs[4],
        tfs[2] - tfs[5]
    ])
    relpos = p2 - p1
    z_unit_vector = unitVector.unitvector(relpos)
    n = (0 - (z_unit_vector[0] - z_unit_vector[0]) - (z_unit_vector[1] - z_unit_vector[1])) / z_unit_vector[2]
    y_unit_vector = np.array([z_unit_vector[0], z_unit_vector[1], n])
    x_unit_vector = np.cross(y_unit_vector, z_unit_vector)
    return x_unit_vector, y_unit_vector, z_unit_vector
