import subprocess
import shutil
import itertools as it
import numpy as np

PHI = 0.0 # ROLL IN LOCAL FRAME

thetas = np.linspace(40, 60, 2)
psis = np.linspace(-15, 15, 2)
tgtEs = np.linspace(3000, 4000, 2)
tgtUs = np.linspace(3000, 4000, 2)
tgtNs = np.linspace(-1000, 1000, 2)
runs = it.product(thetas, psis, tgtEs, tgtNs, tgtUs)

for index, run in enumerate(runs):

	tht = run[0]
	yaw = run[1]
	e = run[2]
	n = run[3]
	u = run[4]

	id = f"{tht}_{yaw}_{e}_{n}_{u}"

	inPutString = f"{id} {PHI} {tht} {yaw} {e} {n} {u}"
	inPutFile = r"CPP_6DOF_SRAAM_V1/input/input.txt"
	with open(inPutFile, "w") as f:
		f.writelines(inPutString)
	process = subprocess.Popen(["./CPP_6DOF_SRAAM_V1/build/missileModel"])
	process.wait()