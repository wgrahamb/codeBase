import subprocess
import shutil

# SIMULATION INPUT
timeStep = 0.001
integrationStep = 0.001
maxTime = 400.0
phi = 0.0 # ROLL ANGLE IN LOCAL FRAME
tht = 45.0 # PITCH ANGLE IN LOCAL FRAME
psi = 2.0 # YAW ANGLE IN LOCAL FRAME, ZERO POINTS TRUE EAST, POSITIVE ROTATES COUNTER CLOCKWISE
wayPointE = 4000.0
wayPointN = -2000.0
wayPointU = 1500.0

inPutString = f"{timeStep} {integrationStep} {maxTime} {phi} {tht} {psi} {wayPointE} {wayPointN} {wayPointU}"
aeroInPutFile = r"input.txt"
with open(aeroInPutFile, "w") as f:
	f.writelines(inPutString)

process = subprocess.Popen(["./breakOutSRAAM"])
process.wait()

shutil.copy("/home/graham/mmt-service-box/breakOutModelSRAAM/log.txt", "/mnt/c/Users/graha/Documents/pythonRepo/pythonRepo/breakOutVisual/breakOutSRAAM.txt")
