import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from numpy import array as npa
from unitVector import unitvector
from numpy import linalg as la
from ambiance import Atmosphere as atm
from interpolationGivenTwoVectors import linearInterpolation


np.set_printoptions(suppress=True, precision=2)

# Constants.
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
wingArea = 0.5 * wingHalfSpan * (wingTipChord + wingRootChord)
tailArea = 0.5 * tailHalfSpan * (tailTipChord + tailRootChord)
refArea = np.pi * (refDiameter ** 2) / 4
noseArea = noseLength * refDiameter
planformArea = (refLength - noseLength) * refDiameter + 0.667 * noseLength * refDiameter
noseCenterOfPressure = 0.67 * noseLength
wingCenterOfPressure = noseLength + distanceFromBaseOfNoseToWing + 0.7 * wingRootChord - 0.2 * wingTipChord
AN = 0.67 * noseLength * refDiameter
AB = (refLength - noseLength) * refDiameter
bodyCenterOfPressure = (0.67 * AN * noseLength + AB * (noseLength + 0.5 * (refLength - noseLength))) / (AN + AB)
transverseMomentOfInertia = (weight * (3 * ((0.5 * refDiameter) ** 2) + refLength ** 2)) / (12 * gravity)
TEMP1 = (centerOfGravityFromNose - wingCenterOfPressure) / refDiameter
TEMP2 = (centerOfGravityFromNose - centerOfDeflectionFromNose) / refDiameter
TEMP3 = (centerOfGravityFromNose - bodyCenterOfPressure) / refDiameter
TEMP4 = (centerOfGravityFromNose - noseCenterOfPressure) / refDiameter

# Missile.
missileTof = 0.0
missilePos = npa([0.0, 0.0])
missileVel = npa([400.0, 1000.0])
missileAcc = npa([0.0, 0.0])

# Target.
targetPos = npa([30000.0, 10000.0])
targetVel = npa([-80.0, 0.0])

# Simulation control.
timeStep = 0.001
go = True
simData = {"TOF": [], "X": [], "Y": [], "TX": [], "TY": [], "COMMAND": [], "ACHIEVED": []}
e = 0
eDot = 0

while go:

	# Target update.
	# targetAcc = np.random.randint(-250, 1, 2)
	targetAcc = np.zeros(2)
	targetVel += (targetAcc * timeStep)
	targetPos += (targetVel * timeStep)

	# Time of flight.
	missileTof += timeStep

	# Guidance.
	relPos = targetPos - missilePos
	relVel = targetVel - missileVel
	velU = unitvector(missileVel)
	velMag = la.norm(missileVel)
	axis = npa(
		[
			velU[0],
			velU[1]
		]
	)
	normal = npa(
		[
			-1 * velU[1],
			velU[0]
		]
	)
	mslLocalOrient = npa([axis, normal])
	rightUpInterceptorToIntercept = mslLocalOrient @ relPos
	rightUpInterceptorToInterceptU = unitvector(rightUpInterceptorToIntercept)
	rightUpInterceptorToInterceptMag = la.norm(rightUpInterceptorToIntercept)
	rightUpInterceptorToInterceptVel = mslLocalOrient @ relVel
	rightUpInterceptorToInterceptVelU = unitvector(rightUpInterceptorToInterceptVel)
	rightUpInterceptorToInterceptVelMag = la.norm(rightUpInterceptorToInterceptVel)
	if rightUpInterceptorToIntercept[0] < 1.0:
		print(f"{missileTof:.2f} MISS DISTANCE {rightUpInterceptorToIntercept} {rightUpInterceptorToInterceptMag}")
		break
	T1 = np.cross(rightUpInterceptorToIntercept, rightUpInterceptorToInterceptVel)
	T2 = np.dot(rightUpInterceptorToIntercept, rightUpInterceptorToIntercept)
	omega = T1 / T2
	normalAccCommand = 2 * omega * rightUpInterceptorToInterceptVelMag
	accCommMag = np.abs(normalAccCommand)
	limit = 10
	if accCommMag > limit:
		new = limit * np.sign(normalAccCommand)
		normalAccCommand = new
	normalAccCommand = 0.0

	# Atmosphere.
	altitude = missilePos[1] # Feet.
	atmosphere = atm(altitude)
	speedOfSound = 3.28084 * atmosphere.speed_of_sound[0] # Feet per second.
	mach = velMag / speedOfSound
	if altitude <= 30000:
		rho = 0.002378 * np.exp(-altitude / 30000)
	else:
		rho = 0.0034 * np.exp(-altitude / 22000)
	dynamicPressure = 0.5 * rho * velMag * velMag

	# Aerodynamics.
	beta = np.sqrt(np.abs(mach ** 2 - 1))
	CNTRIM = weight * normalAccCommand / (dynamicPressure * refArea)
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
	CMAP = 2 * TEMP4 + 1.5 * planformArea * alphaTrim * TEMP3 / refArea + 8 * wingArea * TEMP1 / (beta * refArea)
	CMA = CMAP + 8 * tailArea * TEMP2 / (beta * refArea)
	CMD = 8 * tailArea * TEMP2 / (beta * refArea)
	ZA = -1 * gravity * dynamicPressure * refArea * CNA / (weight * velMag)
	ZD = -1 * gravity * dynamicPressure * refArea * CND / (weight * velMag)
	MA = dynamicPressure * refArea * refDiameter * CMA / transverseMomentOfInertia
	MD = dynamicPressure * refArea * refDiameter * CMD / transverseMomentOfInertia

	# Control
	KR = 0.15
	K1 = -1 * velMag * ((MA * ZD - ZA * MD) / (1845 * MA))
	TA = MD / (MA * ZD - MD * ZA)
	K3 = 1845 * K1 / velMag # 1845 = Some kind of gain.
	KDC = (1 - KR * K3) / (K1 * KR)
	THD = K3 * (e + TA * eDot)

	# Actuator.
	deflection = KR * (KDC * normalAccCommand + THD)

	# Motion.
	eOld = e
	eDotOld = eDot
	omegaZ = np.sqrt((MA * ZD - MD * ZA) / ZD)
	omegaAF = np.sqrt(-1 * MA)
	zetaAF = ZA * omegaAF / (2 * MA)
	eDotDot = (omegaAF ** 2) * (deflection - e - 2 * zetaAF * eDot / omegaAF)
	normalAccel = K1 * (e - (eDotDot / (omegaZ ** 2))) * gravity
	e += timeStep * eDot
	eDot += timeStep * eDotDot
	e = (e + eOld + timeStep * eDot) / 2
	eDot = (eDot + eDotOld + timeStep * eDotDot) / 2

	# Euler semi implicit integration.
	bodyAcc = npa([0.0, normalAccel])
	localAcc = bodyAcc @ mslLocalOrient
	deltaVel = localAcc * timeStep
	missileVel += deltaVel
	deltaPos = missileVel * timeStep
	missilePos += deltaPos

	# Store data.
	simData["TOF"].append(missileTof)
	simData["X"].append(missilePos[0])
	simData["Y"].append(missilePos[1])
	simData["TX"].append(targetPos[0])
	simData["TY"].append(targetPos[1])
	simData["COMMAND"].append(normalAccCommand)
	simData["ACHIEVED"].append(normalAccel / gravity)

	# Console report.
	if round(missileTof, 3).is_integer():
		print(f"TOF {missileTof:.1f} POS {missilePos}")

# Visual.
fig = plt.figure()

trajectory = fig.add_subplot(121)
trajectory.set_xlabel("Down Range (ft).")
trajectory.set_ylabel("Altitude (ft).")
trajectory.plot(simData["X"], simData["Y"], label="Interceptor", color="b")
trajectory.plot(simData["TX"], simData["TY"], label="Threat", color="r")
trajectory.scatter(targetPos[0], targetPos[1], color="r")
trajectory.legend()

accels = fig.add_subplot(122)
# accels.set_ylim([-30, 30])
accels.plot(simData["TOF"], simData["COMMAND"], label="Acceleration command (Gs).", color="r")
accels.plot(simData["TOF"], simData["ACHIEVED"], label="Acceleration achieved (Gs).", color="b")
accels.legend()

# plt.get_current_fig_manager().full_screen_toggle()
plt.show()