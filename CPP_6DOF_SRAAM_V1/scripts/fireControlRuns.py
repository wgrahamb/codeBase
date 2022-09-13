import subprocess
import shutil
import itertools as it
import numpy as np

phi = 0.0 # ROLL IN LOCAL FRAME
tgtN = 0.0 # TARGET NORTH

thetas = np.linspace(30, 60, 2)
psis = np.linspace(-30, 30, 2)
tgtEs = np.linspace(1000, 10000, 2)
tgtUs = np.linspace(1000, 10000, 2)
runs = it.product(thetas, psis, tgtEs, tgtUs)

for index, run in enumerate(runs):

	tht = run[0]
	yaw = run[1]
	e = run[2]
	u = run[3]
	inPutString = f"{phi} {tht} {yaw} {e} {tgtN} {u}"
	inPutFile = r"CPP_6DOF_SRAAM_V1/input/input.txt"
	with open(inPutFile, "w") as f:
		f.writelines(inPutString)
	process = subprocess.Popen(["./CPP_6DOF_SRAAM_V1/build/missileModel"])
	process.wait()