import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from numpy import array as npa
from unitVector import unitvector
from numpy import linalg as la
from ambiance import Atmosphere as atm
from interpolationGivenTwoVectors import linearInterpolation

np.set_printoptions(suppress=True, precision=2)

missileTof = 0.0
missilePos = npa([0.0, 1000.0])
missileVel = npa([800.0, 100.0])
missileAcc = npa([0.0, 0.0])
targetPos = npa([30000.0, 2000.0])
targetVel = npa([-800.0, 0.0])

timeStep = 0.001
go = True
simData = {"TOF": [], "X": [], "Y": [], "TX": [], "TY": [], "COMMAND": [], "ACHIEVED": []}
e = 0
eDot = 0

while go:
	targetAcc = np.random.randint(-50, 50, 2)
	targetVel += (targetAcc * timeStep)
	targetPos += (targetVel * timeStep)
	simData["TX"].append(targetPos[0])
	simData["TY"].append(targetPos[1])
	atmos = atm(missilePos[1])
	gravity = atmos.grav_accel[0]
	rho = atmos.density[0]
	speed = la.norm(missileVel)
	speedOfSound = atmos.speed_of_sound[0]
	mach = speed / speedOfSound
	missileTof += timeStep
	simData["TOF"].append(missileTof)
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
		go = False
	TEMP1 = np.cross(rightUpInterceptorToIntercept, rightUpInterceptorToInterceptVel)
	TEMP2 = np.dot(rightUpInterceptorToIntercept, rightUpInterceptorToIntercept)
	omega = TEMP1 / TEMP2
	normalAccCommand = 4 * omega * rightUpInterceptorToInterceptVelMag
	accCommMag = np.abs(normalAccCommand)
	if accCommMag > 25:
		new = 25 * np.sign(normalAccCommand)
		normalAccCommand = new
	simData["COMMAND"].append(normalAccCommand)

	altitude = missilePos[1] # FEET
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
	wingArea = 0.5 * wingHalfSpan * (wingTipChord + wingRootChord)
	tailArea = 0.5 * tailHalfSpan * (tailTipChord + tailRootChord)
	refArea = np.pi * (refDiameter ** 2) / 4
	noseArea = noseLength * refDiameter
	planformArea = (refLength - noseLength) * refDiameter + 0.667 * noseLength * refDiameter
	mach = speed / speedOfSound
	beta = np.sqrt(np.abs(mach ** 2 - 1))
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

	# CALCULATIONS OF CONSTANTS FOR LOOP ONE
	TEMP1 = (centerOfGravityFromNose - wingCenterOfPressure) / refDiameter
	TEMP2 = (centerOfGravityFromNose - centerOfDeflectionFromNose) / refDiameter
	TEMP3 = (centerOfGravityFromNose - bodyCenterOfPressure) / refDiameter
	TEMP4 = (centerOfGravityFromNose - noseCenterOfPressure) / refDiameter
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
	eOld = e
	eDotOld = eDot
	THD = K3 * (e + TA * eDot)
	deflection = KR * (KDC * normalAccCommand + THD)
	eDotDot = (omegaAF ** 2) * (deflection - e - 2 * zetaAF * eDot / omegaAF)
	normalAccel = K1 * (e - (eDotDot / (omegaZ ** 2)))
	simData["ACHIEVED"].append(normalAccel)
	e += timeStep * eDot
	eDot += timeStep * eDotDot
	e = (e + eOld + timeStep * eDot) / 2
	eDot = (eDot + eDotOld + timeStep * eDotDot) / 2

	bodyAcc = npa([0.0, normalAccel])
	localAcc = bodyAcc @ mslLocalOrient
	deltaVel = localAcc * timeStep
	missileVel += deltaVel
	deltaPos = missileVel * timeStep
	missilePos += deltaPos
	simData["X"].append(missilePos[0])
	simData["Y"].append(missilePos[1])

	if round(missileTof, 3).is_integer():
		print(f"TOF {missileTof:.1f} POS {missilePos}")
		print(CNTRIM)
	
fig = plt.figure()

trajectory = fig.add_subplot(121)
trajectory.plot(simData["X"], simData["Y"])
trajectory.plot(simData["TX"], simData["TY"])
trajectory.scatter(targetPos[0], targetPos[1])

accels = fig.add_subplot(122)
accels.set_ylim([-30, 30])
accels.plot(simData["TOF"], simData["COMMAND"], label="COMMAND")
accels.plot(simData["TOF"], simData["ACHIEVED"], label="ACHIEVED")
accels.legend()

plt.get_current_fig_manager().full_screen_toggle()
plt.show()