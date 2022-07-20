import copy
import time
import numpy as np
import pandas as pd
import pickle
import os
import Functions as fxn
from Guidance.InterceptorBasic import InterceptorBasicGuidance
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import matplotlib.animation as ma
from matplotlib.lines import Line2D
import pickle
import os
import pandas
import ffmpeg

if __name__ == "__main__":
    print("howdy!")
    n = fxn.distance
    aTargetLocWCS = np.array([558286, -5.28912e+06, 3.52314e+06])
    trueLocWCS = np.array([556712, -5.27422e+06, 3.53686e+06])
    predLocXYZ = np.array([557564, -5.27479e+06, 3.53684e+06])
    deltaLocXYZ = np.array([851.451, -567.858, -21.0023])

    print(n(trueLocWCS, predLocXYZ))
    print(n(aTargetLocWCS, trueLocWCS))
    print(n(aTargetLocWCS, predLocXYZ))
    print(trueLocWCS[2] - aTargetLocWCS[2])
    print(predLocXYZ[2] - aTargetLocWCS[2])
    print(trueLocWCS - predLocXYZ)
    print(np.linalg.norm(deltaLocXYZ))

    fig = plt.figure(tight_layout=True)
    ax = fig.add_subplot(111, projection='3d')
    ax.view_init(elev=10, azim=315)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")

    ax.scatter(aTargetLocWCS[0], aTargetLocWCS[1], aTargetLocWCS[2], label="aTargetLocWCS")
    ax.scatter(trueLocWCS[0], trueLocWCS[1], trueLocWCS[2], label="trueLocWCS")
    ax.scatter(predLocXYZ[0], predLocXYZ[1], predLocXYZ[2], label="predLocXYZ")
    ax.legend()
    plt.show()