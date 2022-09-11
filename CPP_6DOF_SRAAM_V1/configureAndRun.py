import subprocess
import shutil

phi = 0.0 # ROLL IN LOCAL FRAME
theta = 40.0 # PITCH ANGLE IN LOCAL FRAME
psi = 10.0 # YAW ANGLE IN LOCAL FRAME, ZERO POINTS TRUE EAST
tgtE = 4000.0  # TARGET EAST
tgtN = 0.0 # TARGET NORTH
tgtU = 3000.0 # TARGET UP

inPutString = f"{phi} {theta} {psi} {tgtE} {tgtN} {tgtU}"
inPutFile = r"CPP_6DOF_SRAAM_V1/input.txt"
with open(inPutFile, "w") as f:
	f.writelines(inPutString)

process = subprocess.Popen(["./CPP_6DOF_SRAAM_V1/sixDofSim"])
process.wait()
