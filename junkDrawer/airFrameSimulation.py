import numpy as np
from numpy import array as npa
from numpy import linalg as la
import pandas as pd
import matplotlib.pyplot as plt

# INPUTS
altitude = 50000 # FEET
speed = 3000 # FEET PER SEC
finDeflectionDeg = 5 # DEGREES
finDeflection = np.radians(finDeflectionDeg) # RADIANS
speedOfSound = 1000 # FEET PER SECOND >>> ASSUMED CONSTANT
gravity = 32.2 # FEET PER S^2 >>> ASSUMED CONSTANT
refDiameter = 1 # FEET
noseLength = 3 # FEET
refLength = 20 # FEET
wingHalfSpan = 2 # FEET
wingTipChord = 0 # FEET
wingRootChord = 6 # FEET
tailHalfSpan = 2 # FEET
tailTipChord = 0 # FEET
tailRootChord = 2 # FEET
distanceFromBaseOfNoseToWing = 4 # FEET
centerOfGravityFromNose = 10 # FEET
centerOfDeflectionFromNose = 19.5 # FEET
weight = 1000 # LBF

# CALCULATIONS OF CONSTANTS FOR LOOP ONE AND TWO
wingArea = 0.5 * wingHalfSpan * (wingTipChord + wingRootChord)
tailArea = 0.5 * tailHalfSpan * (tailTipChord + tailRootChord)
refArea = np.pi * (refDiameter ** 2) / 4
noseArea = noseLength * refDiameter
planformArea = (refLength - noseLength) * refDiameter + 0.667 * noseLength * refDiameter
mach = speed / speedOfSound
beta = np.sqrt(mach ** 2 - 1)
noseCenterOfPressure = 0.67 * noseLength
wingCenterOfPressure = noseLength + distanceFromBaseOfNoseToWing + 0.7 * wingRootChord - 0.2 * wingTipChord
AN = 0.67 * noseLength * refDiameter
AB = (refLength - noseLength) * refDiameter
bodyCenterOfPressure = (0.67 * AN * noseLength + AB * (noseLength + 0.5 * (refLength - noseLength))) / (AN + AB)
if altitude <= 30000:
	rho = 0.002378 * np.exp(-altitude / 30000)
else:
	rho = 0.0034 * np.exp(-altitude / 22000)
dynamicPressure = 0.5 * rho * speed * speed
transverseMomentOfInertia = (weight * (3 * ((0.5 * refDiameter) ** 2) + refLength ** 2)) / (12 * gravity)

# SIMULATION ONE PARAMETERS
alpha = 0.0
thetaDot = 0.0
time = 0.0
timeStep = 0.001
maxTime = 4
loopOneStorage = {
	"TIME OF FLIGHT": [],
	"ALPHA": [],
	"CN": [],
	"CM": [],
	"NORMAL ACCEL": []
}

while time <= maxTime:

	# ITERATE TIME OF FLIGHT
	time += timeStep
	loopOneStorage["TIME OF FLIGHT"].append(time)

	# NORMAL COEFFICIENT AND PITCHING MOMENT COEFFICIENT CALCULATION
	normalCoefficient = 2 * alpha + (1.5 * planformArea * alpha * alpha) / refArea + (8 * wingArea * alpha) / (beta * refArea) + (8 * tailArea * (alpha + finDeflection)) / (beta * refArea)
	pitchingMomentCoefficient = 2 * alpha * ((centerOfGravityFromNose - noseCenterOfPressure) / refDiameter) + ((1.5 * planformArea * alpha * alpha) / refArea) * ((centerOfGravityFromNose - bodyCenterOfPressure) / refDiameter) + ((8 * wingArea * alpha) / (beta * refArea)) * ((centerOfGravityFromNose - wingCenterOfPressure) / refDiameter) + ((8 * tailArea * (alpha + finDeflection)) / (beta * refArea)) * ((centerOfGravityFromNose - centerOfDeflectionFromNose) / refDiameter)

	# STORE DATA AT CURRENT CONDITIONS
	loopOneStorage["ALPHA"].append(alpha)
	loopOneStorage["CN"].append(normalCoefficient)
	loopOneStorage["CM"].append(pitchingMomentCoefficient)

	# UPDATE SIMULATION PARAMETERS
	thetaDotDot = (dynamicPressure * refArea * refDiameter * pitchingMomentCoefficient) / transverseMomentOfInertia
	thetaDot += thetaDotDot * timeStep
	NB = ((dynamicPressure * refArea * normalCoefficient) * (gravity / weight)) / gravity
	loopOneStorage["NORMAL ACCEL"].append(NB)
	alphaDot = thetaDot - ((NB * gravity) / speed)
	alpha += alphaDot * timeStep

# OUTPUT
print(f"LOOP ONE FINISHED.")

# CALCULATION OF CONSTANTS FOR SIMULATION TWO
TEMP1 = (centerOfGravityFromNose - wingCenterOfPressure) / refDiameter
TEMP2 = (centerOfGravityFromNose - centerOfDeflectionFromNose) / refDiameter
TEMP3 = (centerOfGravityFromNose - bodyCenterOfPressure) / refDiameter
TEMP4 = (centerOfGravityFromNose - noseCenterOfPressure) / refDiameter
Y1 = 2 * TEMP4 + 8 * wingArea * TEMP1 / (beta * refArea) + 8 * tailArea * TEMP2 / (beta * refArea)
Y2 = 1.5 * planformArea * TEMP3 / refArea
Y3 = 8 * tailArea * TEMP2 * finDeflection / (beta * refArea)
alphaTrim = (-Y1 - np.sqrt((Y1 ** 2) - 4 * Y2 * Y3)) / (2 * Y2)
CNA = 2 + 1.5 * planformArea * alphaTrim / refArea + 8 * wingArea / (beta * refArea) + 8 * tailArea / (beta * refArea)
CND = 8 * tailArea / (beta * refArea)
ZA = -1 * gravity * dynamicPressure * refArea * CNA / (weight * speed)
ZD = -1 * gravity * dynamicPressure * refArea * CND / (weight * speed)
CMAP = 2 * TEMP4 + 1.5 * planformArea * alphaTrim * TEMP3 / refArea + 8 * wingArea * TEMP1 / (beta * refArea)
CMA = CMAP + 8 * tailArea * TEMP2 / (beta * refArea)
CMD = 8 * tailArea * TEMP2 / (beta * refArea)
MA = dynamicPressure * refArea * refDiameter * CMA / transverseMomentOfInertia
MD = dynamicPressure * refArea * refDiameter * CMD / transverseMomentOfInertia
omegaZ = np.sqrt((MA * ZD - MD * ZA) / ZD)
omegaAF = np.sqrt(-1 * MA)
zetaAF = ZA * omegaAF / (2 * MA)
K1 = -1 * speed * ((MA * ZD - ZA * MD) / (1845 * MA))
K2 = K1
K3 = 1845 * K1 / speed
TA = MD / (MA * ZD - ZA * MD)

# SIMULATION TWO PARAMETERS
e = 0.0
eDot = 0.0
time = 0.0
timeStep = 0.001
maxTime = 4
loopTwoStorage = {
	"TIME OF FLIGHT": [],
	"NORMAL ACCEL": []
}

while time <= maxTime:

	# ITERATE TIME OF FLIGHT
	time += timeStep
	loopTwoStorage["TIME OF FLIGHT"].append(time)
	eOld = e
	eDotOld = eDot
	eDotDot = (omegaAF ** 2) * (finDeflectionDeg - e - 2 * zetaAF * eDot / omegaAF)
	normalAccel = K1 * (e - (eDotDot / (omegaZ ** 2)))
	loopTwoStorage["NORMAL ACCEL"].append(normalAccel)
	e += timeStep * eDot
	eDot += timeStep * eDotDot
	e = (e + eOld + timeStep * eDot) / 2
	eDot = (eDot + eDotOld + timeStep * eDotDot) / 2


# OUTPUT
print(f"LOOP TWO FINISHED.")

# PLOT
df1 = pd.DataFrame(loopOneStorage)
df2 = pd.DataFrame(loopTwoStorage)

fig = plt.figure()

# ax1 = fig.add_subplot(221)
# ax1.set_xlabel("TIME OF FLIGHT")
# ax1.set_ylabel("ALPHA - RADIANS")
# ax1.plot(df1.iloc[:]["TIME OF FLIGHT"], df1.iloc[:]["ALPHA"], color="b")
# ax1.plot(df2.iloc[:]["TIME OF FLIGHT"], df2.iloc[:]["ALPHA"], color="r")

# ax2 = fig.add_subplot(222)
# ax2.set_xlabel("TIME OF FLIGHT")
# ax2.set_ylabel("NORMAL COEFFICIENT")
# ax2.plot(df1.iloc[:]["TIME OF FLIGHT"], df1.iloc[:]["CN"])

# ax3 = fig.add_subplot(223)
# ax3.set_xlabel("TIME OF FLIGHT")
# ax3.set_ylabel("PITCHING MOMENT COEFFICIENT")
# ax3.plot(df1.iloc[:]["TIME OF FLIGHT"], df1.iloc[:]["CM"])

ax4 = fig.add_subplot(111)
ax4.set_xlabel("TIME OF FLIGHT")
ax4.set_ylabel("NORMAL ACCEL (Gs)")
ax4.plot(df1.iloc[:]["TIME OF FLIGHT"], df1.iloc[:]["NORMAL ACCEL"], color="b")
ax4.plot(df2.iloc[:]["TIME OF FLIGHT"], df2.iloc[:]["NORMAL ACCEL"], color="r")

plt.get_current_fig_manager().full_screen_toggle()
plt.show()