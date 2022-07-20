import numpy as np

def PHI_MONITOR(PHI, DELTA):
    if np.sign(DELTA) == -1.0:
        TEMP = PHI + DELTA
        if TEMP < 0.0:
            TEMP2 = np.fmod(TEMP, 2)
            RET = 2.0 + TEMP2
        else:
            RET = TEMP
    elif np.sign(DELTA) == 1.0:
        TEMP = PHI + DELTA
        if TEMP > 2.0:
            TEMP2 = np.fmod(TEMP, 2)
            RET = 0.0 + TEMP2
        else:
            RET = TEMP
    return RET