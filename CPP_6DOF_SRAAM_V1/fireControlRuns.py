import subprocess
import shutil
import itertools as it
import numpy as np

phi = 0.0 # ROLL IN LOCAL FRAME
tgtN = 0.0 # TARGET NORTH

thetas = np.linspace(30, 60, 4)
psis = np.linspace(-30, 30, 4)
tgtEs = np.linspace(1000, 10000, 10)
tgtUs = np.linspace(1000, 10000, 10)
tgtEs = [3000.0]
tgtUs = [3000.0]
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

	shutil.copy("/home/graham/codeBase/sixDofCppFunctional/log.txt", f"/mnt/d/fireControlRunsData/{tht}_{yaw}_{e}_{u}.txt")
	print("\n")
