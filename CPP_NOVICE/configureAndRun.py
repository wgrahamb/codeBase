import subprocess
import shutil

phi = 0.0 # ROLL IN LOCAL FRAME
theta = 45.0 # PITCH ANGLE IN LOCAL FRAME
psi = 18.0 # YAW ANGLE IN LOCAL FRAME, ZERO POINTS TRUE EAST
tgtE = 3000.0  # TARGET EAST
tgtN = 0.0 # TARGET NORTH
tgtU = 3000.0 # TARGET UP

inPutString = f"{phi} {theta} {psi} {tgtE} {tgtN} {tgtU}"
inPutFile = r"input.txt"
with open(inPutFile, "w") as f:
	f.writelines(inPutString)

process = subprocess.Popen(["./NOVICE_CPP"])
process.wait()

shutil.copy(
	"output/originalMissile.txt",
	"/mnt/c/Users/graha/Documents/repo/pythonCode/CPP_NOVICE_Visuals"
)