import subprocess
import shutil
import itertools as it
import numpy as np

phi = 0.0 # ROLL IN LOCAL FRAME
tgtN = 0.0 # TARGET NORTH

thetas = np.linspace(10, 90, 30)
psis = np.linspace(-60, 60, 30)
tgtEs = np.linspace(1000, 10000, 10)
tgtUs = np.linspace(1000, 10000, 10)
runs = it.product(thetas, psis, tgtEs, tgtUs)
for index, run in enumerate(runs):
	tht = run[0]
	yaw = run[1]
	e = run[2]
	u = run[3]
	inPutString = f"{phi} {tht} {yaw} {e} {tgtN} {u}"
	inPutFile = r"input.txt"
	with open(inPutFile, "w") as f:
		f.writelines(inPutString)
	process = subprocess.Popen(["./sixDofSim"])
	process.wait()

	shutil.copy("/home/graham/docs/sixDofCpp/log.txt", f"/mnt/d/fireControlRunsData/{tht}_{yaw}_{e}_{u}.txt")
	print("\n")
