import shutil
import subprocess

tgtEast = 5000.0
tgtNorth = 10000.0
tgtUp = 12500.0

missileVelocityEast = 1000.0
missileVelocityNorth = 170.0
missileVelocityUp = 240.0

newInput = f"{tgtEast} {tgtNorth} {tgtUp} {missileVelocityEast} {missileVelocityNorth} {missileVelocityUp}"

with open("input.txt", "r") as file:
	data = file.readlines()
	file.close()

data[0] = newInput

with open("input.txt", "w") as file:
	file.writelines(data)
	file.close()

process = subprocess.Popen(["./threeDofTestBed"])
process.wait()

shutil.copy(
	"/home/graham/codeBase/cppThreeDofTestBedRK4/output.txt",
	"/mnt/c/Users/graha/Documents/pythonRepo/pythonRepo/junkDrawer/threeDofRK4Integration.txt"
)