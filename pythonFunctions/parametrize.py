from numpy import array as npa

# Inputs must be numpy arrays, equivalent shapes, and of the float type.
def parametrize(a1, a2):
    z = 0.0
    t = (z - a1[2]) / (a2[2] - a1[2])
    x = a1[0] + t * (a2[0] - a1[0])
    y = a1[1] + t * (a2[1] - a1[1])
    return npa([x, y, z])