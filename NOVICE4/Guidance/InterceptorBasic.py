import Functions as fxn
import numpy as np
import numpy.linalg as la
import numba
from numba import float64
from numba.typed import List
from numba.experimental import jitclass

configuration = List.empty_list(float64[:])

spec = [
    ("config", numba.typeof(configuration)),
    ("pos", float64[:]),
    ("vel", float64[:]),
    ("tpos", float64[:]),
    ("tvel", float64[:]),
    ("skill", float64[:]),
    ("drag", float64[:]),
    ("UFD", float64[:]),
    ("TGO", float64)
]


class InterceptorBasicGuidance:
    def __init__(self, config):
        self.pos = config[0]
        self.vel = config[1]
        self.tpos = config[2]
        self.tvel = config[3]
        self.skill = config[4]
        self.drag = config[5]
        self.UFD = config[6]
        self.TGO = 0.0

    def guide(self):
        acc = np.array([0.0, 0.0, 0.0])
        relpos = self.tpos - self.pos
        relvel = self.tvel - self.vel
        relposmag = la.norm(relpos)
        relvelmag = la.norm(relvel)
        self.TGO = relposmag / relvelmag
        velmag = la.norm(self.vel)
        UMV = fxn.unit_vector(self.vel)
        if self.TGO < 5.0:
            relposXrelvel = np.cross(relpos, relvel)
            relposmagsquared = max(relposmag * relposmag, 0.1)
            omega = relposXrelvel / relposmagsquared
            acc = np.cross(self.vel, omega)
            t1 = -5.0 * relvelmag / velmag
            acc = t1 * acc
        else:
            reldisttogo = relpos
            reldisttogomag = la.norm(relpos)
            reldisttogounit = fxn.unit_vector(reldisttogo)
            t2 = 6.0 * reldisttogounit - 1.5 * self.UFD
            t2XUMV = np.cross(t2, UMV)
            acc = np.cross(UMV, t2XUMV)
            t3 = velmag * velmag / reldisttogomag
            acc = t3 * acc
        accmax = self.skill * velmag
        accmag = la.norm(acc)
        trueaccmag = max(accmag, 0.1)
        if accmax < trueaccmag:
            t4 = accmax / trueaccmag
            acc = t4 * acc
            trueaccmag = la.norm(acc)
        drag = self.drag * trueaccmag
        t5 = UMV * drag
        acc = acc - t5

        dist = fxn.distance(self.pos, self.tpos)
        if dist < 30.0:
            endcheck = False
        else:
            endcheck = True

        return acc, endcheck
