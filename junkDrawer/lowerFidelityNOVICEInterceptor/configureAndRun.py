import subprocess
import shutil

missileAz = 15.0
missileEl = 45.0
tgtEPos = 3000.0
tgtNPos = 0.0
tgtUPos = 3000.0

inPutString = f"{missileAz} {missileEl} {tgtEPos} {tgtNPos} {tgtUPos}"
inPutFile = r"input.txt"
with open(inPutFile, "w") as f:
	f.writelines(inPutString)

process = subprocess.Popen(["./build/interceptor"])
process.wait()

moveToPath = "/mnt/c/Users/graha/Desktop/NOVICE_Visuals/threeDofOutput"

shutil.copy(
	"output/log.txt",
	f"{moveToPath}/threeDofLog.txt"
)