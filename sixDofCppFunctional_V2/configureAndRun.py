import subprocess
import shutil
import os

# int ballistic;
# int INTEGRATION_METHOD;
# double phiRads;
# double thetaRads;
# double psiRads;
# double posE;
# double posN;
# double posU;
# double tgtE;
# double tgtN;
# double tgtU;
# int LogData;
# int ConsoleReport;

ballistic = 0 # Boolean.
INTEGRATION_METHOD = 2 # 0 = Euler, 1 = RK2, 2 = RK4.
phi = 0 # Degrees.
theta = 55 # Degrees.
psi = 0 # Degrees.
posE = 0 # Meters.
posN = 0 # Meters.
posU = 0 # Meters.
tgtE = 5000 # Meters.
tgtN = 0 # Meters.
tgtU = 3000 # Meters.
LogData = 1 # Boolean.
ConsoleReport = 1 # Boolean.

InputString = f"{ballistic} {INTEGRATION_METHOD} {phi} {theta} {psi} {posE} {posN} {posU} {tgtE} {tgtN} {tgtU} {LogData} {ConsoleReport}\n"
InputFile = r"input.txt"
with open(InputFile, "w") as f:
	f.writelines(InputString)
	f.close()

process = subprocess.Popen(["./build/missileModel"])
process.wait()