import subprocess
import shutil
import os
import numpy as np
import itertools as it

l1 = np.linspace(-0.5, 0.5, 25)

l2 = []
for index, number in enumerate(l1):
	l2.append([-number, number])

pitchFinCommands = list(l1)
yawFinCommands = [0]
runs = it.product(pitchFinCommands, yawFinCommands)
stopCount = 1000
for index, run in enumerate(l2):
	print("RUN", index)
	ballistic = 1 # Boolean.
	INTEGRATION_METHOD = 2 # 0 = Euler, 1 = RK2, 2 = RK4.
	phi = 0 # Degrees.
	theta = 55 # Degrees.
	psi = 0 # Degrees.
	posE = 0 # Meters.
	posN = 0 # Meters.
	posU = 0 # Meters.
	tgtE = 3000 # Meters.
	tgtN = 0 # Meters.
	tgtU = 3000 # Meters.
	rollFinDeflection = 0.0 # Degrees
	pitchFinDeflection = run[0] # Degrees
	yawFinDeflection = run[1] # Degrees
	LogData = 1 # Boolean.
	ConsoleReport = 1 # Boolean.
	TestControl = 1 # Boolean.

	InputString = f"{ballistic} {INTEGRATION_METHOD} {phi} {theta} {psi} {posE} {posN} {posU} {tgtE} {tgtN} {tgtU} {rollFinDeflection} {pitchFinDeflection} {yawFinDeflection} {LogData} {ConsoleReport} {TestControl}\n"
	InputFile = r"input.txt"
	with open(InputFile, "w") as f:
		f.writelines(InputString)
		f.close()

	process = subprocess.Popen(["./build/missileModel"])
	process.wait()

	if index > stopCount:
		break