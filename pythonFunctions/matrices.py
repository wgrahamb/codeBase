import numpy as np
from numpy import array as npa
import numpy.linalg as la
import unitVector

# engagement plane
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

# M&SofAerospaceVehicleDynamics 3rd Edition Example pg. 41
# Used to project any vector in an inertial frame onto a unit vector in that same inertial frame.
# vector format: np.array([x, y, z])
# unit vector format: np.array([ux, uy, uz])
def projection(unitvector, vector):
    return ((unitvector.reshape(-1, 1) * unitvector) @ vector.reshape(-1, 1)).flatten()

def skewSymmFourByFour(arr):
	return npa(
		[
			[0, -1 * arr[0], -1 * arr[1], -1 * arr[2]],
			[arr[0], 0, arr[2], -1 * arr[1]],
			[arr[1], -1 * arr[2], 0, arr[0]],
			[arr[2], arr[1], -1 * arr[0], 0]
		]
	)

def skewSymmetric(arr):
	return np.array(
		[
			[0, -arr[2], arr[1]],
			[arr[2], 0, -arr[0]],
			[-arr[1], arr[0], 0]
		]
	)

# Inputs must be numpy arrays, equivalent shapes, and of the float type.
def midpoint(a1, a2):
    return np.sum(a1, a2) / 2

# Inputs must be numpy arrays, equivalent shapes, and of the float type.
def parametrize(a1, a2):
    z = 0.0
    t = (z - a1[2]) / (a2[2] - a1[2])
    x = a1[0] + t * (a2[0] - a1[0])
    y = a1[1] + t * (a2[1] - a1[1])
    return npa([x, y, z])

# EXAMPLE
if __name__ == "__main__":
    unitvector = npa([0.2, 0.3, -0.9327])
    vector = npa([7.6, 12.8, -36])
    x = projection(unitvector, vector)
    print(x)