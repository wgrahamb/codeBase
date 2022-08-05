import numpy as np
from numpy import array as npa
from numpy import linalg as la
import pandas as pd
import matplotlib.pyplot as plt
from ambiance import Atmosphere as atm

MM_TO_M = 1.0 / 1000.0

# INPUTS
altitude = 15000 # Meters
speed = 130 # Meters per sec
finDeflectionDeg = 1 # DEGREES
finDeflectionRadians = np.radians(finDeflectionDeg) # Radians.
speedOfSound = 343 # Meters per second
gravity = 9.81 # meters per second squared

refDiameter = 0.18 # Meters
noseLength = 0.249733 # Meters
refLength = 1.6 # Meters

wingHalfSpan = 66.1175 * MM_TO_M / 2.0 # Meters
wingTipChord = 91.047 * MM_TO_M # Meters
wingRootChord = 0.123564 # Meters

tailHalfSpan = 71.3548 * MM_TO_M / 2.0 # Meters
tailRootChord = 0.48084 # Meters
tailTipChord = 0.387894 # Meters

distanceFromBaseOfNoseToWing = 0.323925 # Meters
finalCenterOfGravityFromNose =  0.644605 # Meters
centerOfDeflectionFromNose = 1.8059 - noseLength # Meters (correction here due to oversight in drawing)
weight = 20 # Kg

wingArea = 0.5 * wingHalfSpan * (wingTipChord + wingRootChord)
tailArea = 0.5 * tailHalfSpan * (tailTipChord + tailRootChord)
refArea = np.pi * (refDiameter ** 2) / 4
noseArea = noseLength * refDiameter
planformArea = (refLength - noseLength) * refDiameter + 0.667 * noseLength * refDiameter
mach = speed / speedOfSound
if mach > 1:
	beta = np.sqrt(mach ** 2 - 1)
else:
	beta = mach
noseCenterOfPressure = 0.67 * noseLength
wingCenterOfPressure = noseLength + distanceFromBaseOfNoseToWing + 0.7 * wingRootChord - 0.2 * wingTipChord
AN = 0.67 * noseLength * refDiameter
AB = (refLength - noseLength) * refDiameter
bodyCenterOfPressure = (0.67 * AN * noseLength + AB * (noseLength + 0.5 * (refLength - noseLength))) / (AN + AB)
atmos = atm(altitude)
rho = atmos.density[0]
dynamicPressure = 0.5 * rho * speed * speed
transverseMomentOfInertia = (weight * (3 * ((0.5 * refDiameter) ** 2) + refLength ** 2)) / (12 * gravity)

# CALCULATIONS OF CONSTANTS FOR LOOP ONE
accelCommand = 1 # Gs
TEMP1 = (finalCenterOfGravityFromNose - wingCenterOfPressure) / refDiameter
TEMP2 = (finalCenterOfGravityFromNose - centerOfDeflectionFromNose) / refDiameter
TEMP3 = (finalCenterOfGravityFromNose - bodyCenterOfPressure) / refDiameter
TEMP4 = (finalCenterOfGravityFromNose - noseCenterOfPressure) / refDiameter
CNTRIM = weight * accelCommand / (dynamicPressure * refArea)
Y1 = 2 + 8 * wingArea / (beta * refArea) + 8 * tailArea / (beta * refArea)
Y2 = 1.5 * planformArea / refArea
Y3 = 8 * tailArea / (beta * refArea)
Y4 = 2 * TEMP4 + 8 * wingArea * TEMP1 / (beta * refArea) + 8 * tailArea * TEMP2 / (beta * refArea)
Y5 = 1.5 * planformArea * TEMP3 / refArea
Y6 = 8 * tailArea * TEMP2 / (beta * refArea)
P2 = Y2 - (Y3 * Y5) / Y6
P3 = Y1 - (Y3 * Y4) / Y6
alphaTrim = (-1 * P3 + np.sqrt(P3 * P3 + 4 * P2 * CNTRIM)) / (2 * P2)
deltaTrim = (-1 * Y4 * alphaTrim - Y5 * alphaTrim * alphaTrim) / Y6
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
KR = 0.1
K1 = -1 * speed * ((MA * ZD - ZA * MD) / (1845 * MA))
TA = MD / (MA * ZD - MD * ZA)
K3 = 1845 * K1 / speed
KDC = (1 - KR * K3) / (K1 * KR)

# SIMULATION TWO PARAMETERS
e = 0.0
eDot = 0.0
time = 0.0
timeStep = 0.0001
maxTime = 1
storage = {
	"TIME OF FLIGHT": [],
	"NORMAL ACCEL": [],
	"COMMANDED ACCEL": [],
	"DEFLECTION": []
}

while time <= maxTime:

	# ITERATE TIME OF FLIGHT
	time += timeStep
	storage["TIME OF FLIGHT"].append(time)
	storage["COMMANDED ACCEL"].append(accelCommand)
	eOld = e
	eDotOld = eDot
	THD = K3 * (e + TA * eDot)
	deflection = KR * (KDC * accelCommand + THD)
	storage["DEFLECTION"].append(deflection)
	eDotDot = (omegaAF ** 2) * (deflection - e - 2 * zetaAF * eDot / omegaAF)
	normalAccel = K1 * (e - (eDotDot / (omegaZ ** 2)))
	storage["NORMAL ACCEL"].append(normalAccel)
	e += timeStep * eDot
	eDot += timeStep * eDotDot
	e = (e + eOld + timeStep * eDot) / 2
	eDot = (eDot + eDotOld + timeStep * eDotDot) / 2

DF = pd.DataFrame(storage)
plt.plot(DF.iloc[:]["TIME OF FLIGHT"], DF.iloc[:]["NORMAL ACCEL"], label="ACHIEVED", color="b")
plt.plot(DF.iloc[:]["TIME OF FLIGHT"], DF.iloc[:]["COMMANDED ACCEL"], label="COMMANDED", color="r")
plt.plot(DF.iloc[:]["TIME OF FLIGHT"], DF.iloc[:]["DEFLECTION"], label="DEFLECTION", color="g")
plt.legend()
plt.show()