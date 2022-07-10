import numpy as np
import numpy.linalg as la
import numba
from numba import njit, float64

import Guidance.InterceptorBasic
import Guidance.ThreatBasic


# Input must be numpy array and of the float type.
def unit_vector(a):
    amag = la.norm(a)
    return a / amag


# Inputs must be numpy arrays, equivalent shapes, and of the float type.
def distance(a1, a2):
    sqrtx = a2[0] - a1[0]
    x = sqrtx * sqrtx
    sqrty = a2[1] - a1[1]
    y = sqrty * sqrty
    sqrtz = a2[2] - a1[2]
    z = sqrtz * sqrtz
    ret = np.sqrt(x + y + z)
    return ret


# Inputs must be numpy arrays, equivalent shapes, and of the float type.
# Input format: Numpy arrays -> [x, y, z, vx, vy, vz]
def closing_velocities_ep(a1, a2):
    p1 = np.array(
        [
            a1[0] - a2[3],
            a1[1] - a2[4],
            a1[2] - a2[5]
        ]
    )
    p2 = np.array(
        [
            a1[0] - a1[3],
            a1[1] - a1[4],
            a1[2] - a1[5]
        ]
    )
    relpos = p2 - p1
    z_unit_vector = unit_vector(relpos)
    n = (0 - (z_unit_vector[0] - z_unit_vector[0]) - (z_unit_vector[1] - z_unit_vector[1])) / z_unit_vector[2]
    y_unit_vector = np.array([z_unit_vector[0], z_unit_vector[1], n])
    x_unit_vector = np.cross(y_unit_vector, z_unit_vector)
    return x_unit_vector, y_unit_vector, z_unit_vector


# Inputs must be numpy arrays, equivalent shapes, and of the float type.
def los_ufd(a1, a2):
    t1 = a2 - a1
    t2 = la.norm(t1)
    return t1 / t2


# Inputs must be numpy arrays, equivalent shapes, and of the float type.
def midpoint(a1, a2):
    return (a1 + a2) / 2


# Inputs must be numpy arrays, equivalent shapes, and of the float type.
def parametrize(a1, a2):
    z = 0.0
    t = (z - a1[2]) / (a2[2] - a1[2])
    x = a1[0] + t * (a2[0] - a1[0])
    y = a1[1] + t * (a2[1] - a1[1])
    return np.array([x, y, z])


# Input array is a three dimensional coordinate (numpy arrays of type "float64").
# Input engagement plane includes an x base, y base, z base, and an origin (numpy arrays of type "float64").
def rotate_into_ep(a, ep):
    p = a - ep[3]
    t1 = ep[0] * p
    t2 = ep[1] * p
    t3 = ep[2] * p
    epx = t1[0] + t1[1] + t1[2]
    epy = t2[0] + t2[1] + t2[2]
    epz = t3[0] + t3[1] + t3[2]
    return np.array([epx, epy, epz])


# Input array is a three dimensional coordinate (numpy arrays of type "float64").
# Input engagement plane includes an x base, y base, z base, and an origin (numpy arrays of type "float64").
def rotate_into_enu(a, ep):
    m1 = ep[0][0]
    m2 = ep[0][1]
    m3 = ep[0][2]
    m4 = ep[1][0]
    m5 = ep[1][1]
    m6 = ep[1][2]
    m7 = ep[2][0]
    m8 = ep[2][1]
    m9 = ep[2][2]
    det = m1 * m5 * m9 + m4 * m8 * m3 + m7 * m2 * m6 - m1 * m6 * m8 - m3 * m5 * m7 - m2 * m4 * m9
    temp1 = (m5 * m9 - m6 * m8) / det
    temp2 = (m3 * m8 - m2 * m9) / det
    temp3 = (m2 * m6 - m3 * m5) / det
    temp4 = (m6 * m7 - m4 * m9) / det
    temp5 = (m1 * m9 - m3 * m7) / det
    temp6 = (m3 * m4 - m1 * m6) / det
    temp7 = (m4 * m8 - m5 * m7) / det
    temp8 = (m2 * m7 - m1 * m8) / det
    temp9 = (m1 * m5 - m2 * m4) / det
    dcm_inv1 = np.array([temp1, temp2, temp3])
    dcm_inv2 = np.array([temp4, temp5, temp6])
    dcm_inv3 = np.array([temp7, temp8, temp9])
    temp10 = ep[0] * ep[3]
    temp11 = ep[1] * ep[3]
    temp12 = ep[2] * ep[3]
    ox = temp10[0] + temp10[1] + temp10[2]
    oy = temp11[0] + temp11[1] + temp11[2]
    oz = temp12[0] + temp12[1] + temp12[2]
    o = np.array([ox, oy, oz])
    pos_from_origin = a - o
    temp13 = dcm_inv1 * pos_from_origin
    temp14 = dcm_inv2 * pos_from_origin
    temp15 = dcm_inv3 * pos_from_origin
    enux = -temp13[0] - temp13[1] - temp13[2]
    enuy = -temp14[0] - temp14[1] - temp14[2]
    enuz = -temp15[0] - temp15[1] - temp15[2]
    return np.array([enux, enuy, enuz])


# Inputs p, v, and a are three dimensional coordinate (numpy arrays of type "float64").
# Input timestep is a float.
def update(p, v, a, timestep):
    t1 = 0.5 * timestep
    t2 = t1 * a
    t3 = v + t2
    delta = timestep * t3
    newp = p + delta
    t4 = timestep * a
    newv = v + t4
    return newp, newv

# Input any list comprised of numpy arrays of type "float64."
def convert_pylist_to_nblist(lst):
    ret_list = numba.typed.List.empty_list(float64[:])
    for i in lst:
        ret_list.append(i)
    return ret_list


# Input "msg" is a string that reflects the name of the chosen guidance Class.
# Input "config" is a format determined by the guidance class.
# Input "tof" is a float
def projectorize_i(msg, config, tof):
    start = config.copy()
    if msg == "InterceptorBasicGuidance":
        g = Guidance.InterceptorBasic.InterceptorBasicGuidance(config)
    timestep = 0.01
    TOF = tof
    p = [np.array([g.pos[0], g.pos[1], g.pos[2],
                   g.vel[0], g.vel[1], g.vel[2],
                   TOF])]
    fly = True
    i = -1
    while fly:
        i += 1
        TOF = round(TOF + timestep, 2)
        data = g.guide()
        fly = data[1]
        nxt = update(g.pos, g.vel, data[0], timestep)
        g.pos = nxt[0]
        g.vel = nxt[1]
        dp = np.array([g.pos[0], g.pos[1], g.pos[2],
                       g.vel[0], g.vel[1], g.vel[2],
                       TOF])
        p.append(dp)
        if i > 10000:
            fly = False
    return p, start


# Input "msg" is a string that reflects the name of the chosen guidance Class.
# Input "config" is a format determined by the guidance class.
# Input "tof" is a float
def projectorize_t(msg, config, tof):
    start = config.copy()
    if msg == "ThreatBasicGuidance":
        g = Guidance.ThreatBasic.ThreatBasicGuidance(config)
    timestep = 0.01
    TOF = tof
    p = [np.array([g.pos[0], g.pos[1], g.pos[2],
                   g.vel[0], g.vel[1], g.vel[2],
                   TOF])]
    fly = True
    i = -1
    while fly:
        i += 1
        TOF = round(TOF + timestep, 2)
        data = g.guide()
        fly = data[1]
        nxt = update(g.pos, g.vel, data[0], timestep)
        g.pos = nxt[0]
        g.vel = nxt[1]
        dp = np.array([g.pos[0], g.pos[1], g.pos[2],
                       g.vel[0], g.vel[1], g.vel[2],
                       TOF])
        p.append(dp)
        if i > 10000:
            fly = False
    return p, start


# Input is a numpy array of any size. Rounds to two decimals.
def rnd(x):
    out = np.empty_like(x)
    return np.round(x, 2, out)


# Input "pos" is three dimensional numpy array.
# Input "soldierskill" is a one dimensional numpy array of type "float64."
# Input "soldierdrag" is a one dimensional numpy array of type "float64."
# Input "soldiertof" is of type "float64."
# Input "np_eval" is a list of numpy arrays of type "float64." As returned by the projectory FXNS.
def find_firstshot(soldierpos, soldiervel, soldierskill, soldierdrag, soldiertof, np_eval):
    firstshot = False
    iteration = -1
    index = -150
    while not firstshot:
        iteration += 1
        index -= 1
        pipdata = np_eval[index]
        tpos = np.array([pipdata[0], pipdata[1], pipdata[2]])
        ufd = los_ufd(soldierpos, tpos)
        vel = np.array([soldiervel[0] * ufd[0], soldiervel[1] * ufd[1], soldiervel[2]])
        tvel = np.zeros(3)
        config = [soldierpos,
                              vel,
                              tpos,
                              tvel,
                              soldierskill,
                              soldierdrag,
                              ufd]
        shotdata = projectorize_i("InterceptorBasicGuidance",
                                  config,
                                  soldiertof)
        shot = shotdata[0]
        soldierfinalstate = (shot[-1])
        check = soldierfinalstate[-1] / pipdata[-1]
        if check < 1.0:
            break
    retpip = np.append(pipdata, [soldierfinalstate[-1], pipdata[-1]])
    return shot, retpip



# Input "pos" is three dimensional numpy array.
# Input "soldierskill" is a one dimensional numpy array of type "float64."
# Input "soldierdrag" is a one dimensional numpy array of type "float64."
# Input "soldiertof" is of type "float64."
# Input "np_eval" is a list of numpy arrays of type "float64." As returned by the projectory FXNS.
def find_bestshot(soldierpos, soldiervel, soldierskill, soldierdrag, soldiertof, firstshottof, np_eval):
    bestshot = False
    tof1 = np_eval[0, -1]
    tof2 = firstshottof[0]
    iteration = -1
    while not bestshot:
        iteration += 1
        if iteration > 20:
            print("NO BETTER SHOT. REVERTING TO FIRSTSHOT.")
            print("BUHBYE")
            print("")
            break
        print("HOWDY")
        print("BISECT TOFS: ", tof1, tof2)
        temp = np.array([(tof1 + tof2) / 2])
        tof = rnd(temp)
        pipdata = np_eval[np.where(np_eval[:, -1] == tof[0])]
        tpos = np.array([pipdata[0][0], pipdata[0][1], pipdata[0][2]])
        ufd = los_ufd(soldierpos, tpos)
        vel = np.array([soldiervel[0] * ufd[0], soldiervel[1] * ufd[1], soldiervel[2]])
        print("SHOTINITSPEED", np.linalg.norm(vel))
        tvel = np.zeros(3)
        config = [soldierpos,
                  vel,
                  tpos,
                  tvel,
                  soldierskill,
                  soldierdrag,
                  ufd]
        shotdata = projectorize_i("InterceptorBasicGuidance",
                                  config,
                                  soldiertof)
        shot = shotdata[0]
        soldierfinalstate = (shot[-1])
        ratio = soldierfinalstate[-1] / pipdata[0][-1]
        print("SHOTTOF: ", soldierfinalstate[-1])
        print("THREATTOF: ", pipdata[0][-1])
        if ratio > 1.0:
            print("RATIO: ", ratio)
            print("")
            tof1 = tof[0]
        elif ratio < 0.7:
            print("RATIO: ", ratio)
            print("")
            tof2 = tof[0]
        elif 0.7 <= ratio <= 1.0:
            print("RATIO: ", ratio)
            print("CONGRATULATIONS! WHAT DO WE SAY TO THE GOD OF DEATH?")
            print(" ")
            bestshot = True
    retpip = np.append(pipdata[0], [tof1, tof2])
    return shot, retpip, bestshot


# Input "trajectory" is a list of numpy arrays of type "float64." As returned by the projectory FXNS.
# Input "direction" is a unit vector (numpy array).
# Input "dist" is a number of type "float64."
# Input "ep" includes an x base, y base, z base, and an origin (numpy arrays of type "float64").
def shift_trajectory(trajectory, direction, dist, ep):
    shape = np.shape(trajectory)
    t1 = rotate_into_enu(direction * dist,
                         ep)
    t2 = np.append(t1, np.array([0.0, 0.0, 0.0, 0.0]))
    shifts = [t2] * shape[0]
    shift = np.add(trajectory, shifts[0])
    return shift


# Input "pos" is three dimensional numpy array.
# Input "soldierskill" is a one dimensional numpy array of type "float64."
# Input "soldierdrag" is a one dimensional numpy array of type "float64."
# Input "soldiertof" is of type "float64."
# Input "np_eval" is a list of numpy arrays of type "float64." As returned by the projectory FXNS.
def contain_threat(pos, velocity, skill, drag, soldier_tof, np_eval, t1, t2):
    bestshot = False
    tof1 = t1
    tof2 = t2
    iteration = -1
    while not bestshot:
        iteration += 1
        if iteration > 20:
            print("NO GOOD SHOT. YOU'RE SCREWED.")
            print("BUHBYE")
            print("")
            break
        print("HOWDY")
        print("BISECT TOFS: ", tof1, tof2)
        temp = np.array([(tof1 + tof2) / 2])
        tof = rnd(temp)
        pipdata = np_eval[np.where(np_eval[:, -1] == tof)]
        tpos = np.array([pipdata[0][0], pipdata[0][1], pipdata[0][2]])
        ufd = los_ufd(pos, tpos)
        vel = velocity
        print("SHOTINITSPEED", np.linalg.norm(vel))
        tvel = np.zeros(3)
        config = [pos,
                  vel,
                  tpos,
                  tvel,
                  skill,
                  drag,
                  ufd]
        shotdata = projectorize_i("InterceptorBasicGuidance",
                                  config,
                                  soldier_tof)
        shot = shotdata[0]
        soldierfinalstate = (shot[-1])
        ratio = soldierfinalstate[-1] / pipdata[0][-1]
        print("SHOTTOF: ", soldierfinalstate[-1])
        print("THREATTOF: ", pipdata[0][-1])
        print("RATIO: ", ratio)
        if ratio > 1.0:
            tof1 = tof[0]
        elif ratio < 0.7:
            tof2 = tof[0]
        elif 0.7 <= ratio <= 1.0:
            print("ARE YOU NOT CONTAINED??")
            print("")
            bestshot = True
    return shot, pipdata[0], bestshot


def find_containment_limit(trajectory, direction, ep, pos, velocity, skill, drag, soldier_tof, tof1, tof2):
    offsetdist1 = 0
    offsetdist2 = 500
    containment_limit = False
    ep_input = [ep[0],
                ep[1],
                ep[2],
                np.zeros(3)]
    iteration = -1
    while not containment_limit:
        print(offsetdist1, offsetdist2)
        iteration += 1
        if iteration > 20:
            break
        dist_bisect = (offsetdist1 + offsetdist2) / 2
        np_eval = shift_trajectory(trajectory, direction, dist_bisect, ep_input)
        bestshot = contain_threat(pos, velocity, skill, drag, soldier_tof, np_eval, tof1, tof2)
        if bestshot[2]:
            lastshot = bestshot
            offsetdist1 = dist_bisect
            if offsetdist2 - offsetdist1 < 50:
                containment_limit = True
        elif not bestshot[2]:
            if bestshot[0][-1][-1] > 50.0 and iteration > 5:
                break
            lastshot = bestshot
            offsetdist2 = dist_bisect
    return lastshot