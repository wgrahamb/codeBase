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
noseCenterOfPressure = 0.67 * noseLength
wingCenterOfPressure = noseLength + distanceFromBaseOfNoseToWing + 0.7 * wingRootChord - 0.2 * wingTipChord
AN = 0.67 * noseLength * refDiameter
AB = (refLength - noseLength) * refDiameter
bodyCenterOfPressure = (0.67 * AN * noseLength + AB * (noseLength + 0.5 * (refLength - noseLength))) / (AN + AB)
transverseMomentOfInertia = (weight * (3 * ((0.5 * refDiameter) ** 2) + refLength ** 2)) / (12 * gravity)
TEMP1 = (finalCenterOfGravityFromNose - wingCenterOfPressure) / refDiameter
TEMP2 = (finalCenterOfGravityFromNose - centerOfDeflectionFromNose) / refDiameter
TEMP3 = (finalCenterOfGravityFromNose - bodyCenterOfPressure) / refDiameter
TEMP4 = (finalCenterOfGravityFromNose - noseCenterOfPressure) / refDiameter

# Missile.
missileTof = 0.0
missilePos = npa([0.0, 0.0])
missileVel = npa([50.0, 100.0])
missileAcc = npa([0.0, 0.0])

# Target.
targetPos = npa([2000.0, 2000.0])
targetVel = npa([-200.0, -200.0])

# Simulation control.
timeStep = 0.001
go = True
simData = {"TOF": [], "X": [], "Y": [], "TX": [], "TY": [], "COMMAND": [], "ACHIEVED": [], "ALPHA": [], "THETA_DOT": [], "ALPHA_DOT": []}

alpha = 0.0
alphaOld = 0.0
alphaDot = 0.0
e = 0
eDot = 0

while go:

	# Target update.
	targetAcc = np.random.randint(-1, 1, 2)
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
	normalAccCommand = 4 * omega * rightUpInterceptorToInterceptVelMag
	accCommMag = np.abs(normalAccCommand)
	limit = 35
	if accCommMag > limit:
		new = limit * np.sign(normalAccCommand)
		normalAccCommand = new

	# Atmosphere.
	altitude = missilePos[1] # Meters
	atmosphere = atm(altitude)
	speedOfSound = atmosphere.speed_of_sound[0] # Feet per second.
	mach = velMag / speedOfSound
	rho = atmosphere.density[0]
	dynamicPressure = 0.5 * rho * velMag * velMag

	# Aerodynamics.
	if mach > 1:
		beta = np.sqrt(mach ** 2 - 1)
	else:
		beta = 0.7
	CNTRIM = weight * normalAccCommand / (dynamicPressure * refArea)

	Y1 = 2 + 8 * wingArea / (beta * refArea) + 8 * tailArea / (beta * refArea)
	Y2 = 1.5 * planformArea / refArea
	Y3 = 8 * tailArea / (beta * refArea)
	Y4 = 2 * TEMP4 + 8 * wingArea * TEMP1 / (beta * refArea) + 8 * tailArea * TEMP2 / (beta * refArea)
	Y5 = 1.5 * planformArea * TEMP3 / refArea
	Y6 = 8 * tailArea * TEMP2 / (beta * refArea)
	P2 = Y2 - (Y3 * Y5) / Y6
	P3 = Y1 - (Y3 * Y4) / Y6

	# Kludge limiter on alpha trim.
	temporaryOne = P3 * P3
	temporaryTwo = 4 * P2 * CNTRIM
	temporaryThree = temporaryOne + temporaryTwo
	if temporaryThree < 0:
		temporaryThree =1

	alphaTrim = (-1 * P3 + np.sqrt(temporaryThree)) / (2 * P2)
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
	KR = 0.1
	K1 = -1 * velMag * ((MA * ZD - ZA * MD) / (1845 * MA))
	TA = MD / (MA * ZD - MD * ZA)
	K3 = 1845 * K1 / velMag # 1845 = Some kind of gain.
	KDC = (1 - KR * K3) / (K1 * KR)
	THD = K3 * (e + TA * eDot) # theta dot

	# Actuator.
	deflection = KR * (KDC * normalAccCommand + THD)

	# Motion.
	eOld = e
	eDotOld = eDot
	omegaZ = np.sqrt((MA * ZD - MD * ZA) / ZD)
	omegaAF = np.sqrt(-1 * MA)
	zetaAF = ZA * omegaAF / (2 * MA)
	eDotDot = (omegaAF ** 2) * (deflection - e - 2 * zetaAF * eDot / omegaAF)

	normalAccel = K1 * (e - (eDotDot / (omegaZ ** 2)))
	alphaDot = THD - ((normalAccel * gravity) / velMag)
	alpha += alphaDot * timeStep

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
	simData["ACHIEVED"].append(normalAccel)
	simData["ALPHA"].append(alpha)
	simData['THETA_DOT'].append(THD)
	simData["ALPHA_DOT"].append(alphaDot)

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

performance = fig.add_subplot(122)
# accels.set_ylim([-30, 30])
performance.plot(simData["TOF"], simData["COMMAND"], label="Acceleration command (m/s^2).", color="r")
performance.plot(simData["TOF"], simData["ACHIEVED"], label="Acceleration achieved (m/s^2).", color="b")
performance.plot(simData["TOF"], simData["ALPHA"], label="Alpha Radians.")
performance.plot(simData["TOF"], simData["THETA_DOT"], label="Theta Dot Radians Per Second.")
# performance.plot(simData["TOF"], simData["ALPHA_DOT"], label="Alpha Dot Radians Per Second.")
performance.legend()

# plt.get_current_fig_manager().full_screen_toggle()
plt.show()