# Inputs p, v, and a are three dimensional coordinate (numpy arrays of type "float64").
# Input timestep is a float.
def integrationstep(p, v, a, timestep):
    t1 = 0.5 * timestep
    t2 = t1 * a
    t3 = v + t2
    delta = timestep * t3
    newp = p + delta
    t4 = timestep * a
    newv = v + t4
    return newp, newv