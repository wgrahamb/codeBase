import numpy as np

def PSI_MONITOR(PSI, DELTA):
    if np.sign(DELTA) == -1.0:
        TEMP = PSI + DELTA
        if TEMP < -1.0:
            TEMP2 = np.fmod(TEMP, 2)
            RET = 1.0 + TEMP2
        else:
            RET = TEMP
    elif np.sign(DELTA) == 1.0:
        TEMP = PSI + DELTA
        if TEMP > 1.0:
            TEMP2 = np.fmod(TEMP, 2)
            RET = -1.0 + TEMP2
        else:
            RET = TEMP
    return RET