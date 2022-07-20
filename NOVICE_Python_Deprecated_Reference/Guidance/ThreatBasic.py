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
    ("man_alt", float64[:]),
    ("skill", float64[:]),
    ("drag", float64[:]),
    ("UFD", float64[:]),
    ("EP_x_base", float64[:]),
    ("EP_y_base", float64[:]),
    ("EP_z_base", float64[:]),
    ("EP_o", float64[:]),
    ("TGO", float64)
]


class ThreatBasicGuidance:
    def __init__(self, config):
        self.pos = config[0]
        self.vel = config[1]
        self.tpos = config[2]
        self.tvel = config[3]
        self.man_alt = config[4]
        self.skill = config[5]
        self.drag = config[6]
        self.UFD = config[7]
        self.EP_x_base = config[8]
        self.EP_y_base = config[9]
        self.EP_z_base = config[10]
        self.EP_o = config[11]
        self.TGO = 0.0

    def guide(self):
        acc = np.array([0.0, 0.0, 0.0])
        relpos = self.tpos - self.pos
        relvel = self.tvel - self.vel
        relposmag = la.norm(relpos)
        relvelmag = la.norm(relvel)
        self.TGO = relposmag / relvelmag
        if self.pos[2] < self.man_alt:
            velmag = la.norm(self.vel)
            UMV = fxn.unit_vector(self.vel)
            if self.TGO < 5.0:
                t1 = np.cross(relpos, relvel)
                t2 = np.dot(relpos, relpos)
                t3 = max(t2, 0.1)
                omega = t1 / t3
                t4 = np.cross(self.vel, omega)
                t5 = -3.0 * relvelmag / velmag
                acc = t5 * t4
            else:
                reldisttogo = relpos
                reldisttogomag = la.norm(reldisttogo)
                reldisttogounit = fxn.unit_vector(reldisttogo)
                t6 = la.norm(self.UFD)
                if t6 > 0.5:
                    ufdt = self.UFD
                else:
                    ufdt = reldisttogounit
                t7 = 6.0 * reldisttogounit
                t8 = 1.5 * ufdt
                term = t7 - t8
                t9 = velmag * velmag
                t10 = np.cross(term, UMV)
                t11 = np.cross(UMV, t10)
                t12 = t9 / reldisttogomag
                acc = t12 * t11
            accmax = self.skill * velmag
            accmag = la.norm(acc)
            trueaccmag = max(accmag, 0.1)
            if accmax < trueaccmag:
                t13 = accmax / accmag
                acc = acc * t13
            t14 = la.norm(acc)
            drag = self.drag * t14
            t15 = UMV * drag
            acc = acc - t15

        ep = [self.EP_x_base, self.EP_y_base, self.EP_z_base, self.EP_o]
        pos_ep = fxn.rotate_into_ep(self.pos, ep)
        if pos_ep[2] < 0.0:
            endcheck = False
        else:
            endcheck = True

        return acc, endcheck