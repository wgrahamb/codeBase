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
targetVel = npa([-600.0, 0.0])

timeStep = 0.001
go = True

simData = {"TOF": [], "X": [], "Y": [], "COMMAND": [], "ACHIEVED": []}
MACHS = np.linspace(0, 5, 100)
CNAs = np.linspace(5.5, 6.5, 100)
CNDs = np.linspace(2.5, 4.0, 100)
CMAs = np.linspace(-20.0, -16.0, 100)
CMDs = np.linspace(-19.0, -15.0, 100)
refArea = 0.1
refDiameter = np.sqrt(refArea * 4 / np.pi)
weight = 1000
transverseMomentOfInertia = 1000
e = 0
eDot = 0

while go:
	targetAcc = np.random.randint(-50, 50, 2)
	targetVel += (targetAcc * timeStep)
	targetPos += (targetVel * timeStep)
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


	dynamicPressure = 0.5 * rho * speed * speed
	CNA = linearInterpolation(mach, MACHS, CNAs)
	CND = linearInterpolation(mach, MACHS, CNDs)
	ZA = -1 * gravity * dynamicPressure * refArea * CNA / (weight * speed)
	ZD = -1 * gravity * dynamicPressure * refArea * CND / (weight * speed)
	CMA = linearInterpolation(mach, MACHS, CMAs)
	CMD = linearInterpolation(mach, MACHS, CMDs)
	MA = dynamicPressure * refArea * refDiameter * CMA / transverseMomentOfInertia
	MD = dynamicPressure * refArea * refDiameter * CMD / transverseMomentOfInertia
	omegaZ = np.sqrt((MA * ZD - MD * ZA) / ZD)
	omegaAF = np.sqrt(-1 * MA)
	zetaAF = ZA * omegaAF / (2 * MA)
	KR = 0.09
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
	
fig = plt.figure()

trajectory = fig.add_subplot(121)
trajectory.plot(simData["X"], simData["Y"])
trajectory.scatter(targetPos[0], targetPos[1])

accels = fig.add_subplot(122)
accels.set_ylim([-30, 30])
accels.plot(simData["TOF"], simData["COMMAND"], label="COMMAND")
accels.plot(simData["TOF"], simData["ACHIEVED"], label="ACHIEVED")
accels.legend()

plt.get_current_fig_manager().full_screen_toggle()
plt.show()