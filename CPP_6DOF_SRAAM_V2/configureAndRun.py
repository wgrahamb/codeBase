import subprocess
import shutil
import os

ballistic = 0 # Boolean.
INTEGRATION_METHOD = 1 # 0 = Euler, 1 = RK2, 2 = RK4.
phi = 0 # Degrees.
theta = 45 # Degrees.
psi = 25 # Degrees.
posE = 0 # Meters.
posN = 0 # Meters.
posU = 0 # Meters.
tgtE = 3000 # Meters.
tgtN = 3000 # Meters.
tgtU = 3000 # Meters.
LogData = 1 # Boolean.
ConsoleReport = 1 # Boolean.

InputString = f"{ballistic} {INTEGRATION_METHOD} {phi} {theta} \
{psi} {posE} {posN} {posU} {tgtE} {tgtN} {tgtU} {LogData} {ConsoleReport}\n"
InputFile = r"CPP_6DOF_SRAAM_V2/input/input.txt"
with open(InputFile, "w") as f:
	f.writelines(InputString)
	f.close()

process = subprocess.Popen(["./CPP_6DOF_SRAAM_V2/build/missileModel"])
process.wait()