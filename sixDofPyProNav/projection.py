import numpy as np
from numpy import array as npa
import numpy.linalg as la

# M&SofAerospaceVehicleDynamics 3rd Edition Example pg. 41
# Used to project any vector in an inertial frame onto a unit vector in that same inertial frame.
# vector format: np.array([x, y, z])
# unit vector format: np.array([ux, uy, uz])
def projection(unitvector, vector):
    return ((unitvector.reshape(-1, 1) * unitvector) @ vector.reshape(-1, 1)).flatten()

# EXAMPLE
if __name__ == "__main__":
    unitvector = npa([0.2, 0.3, -0.9327])
    vector = npa([7.6, 12.8, -36])
    x = projection(unitvector, vector)
    print(x)