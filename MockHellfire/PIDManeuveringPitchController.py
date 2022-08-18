import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
from numpy import array as npa
from utility.unitVector import unitvector
from numpy import linalg as la
from utility.interpolationGivenTwoVectors import linearInterpolation
from utility import coordinateTransformations as ct
from utility.returnAzAndElevation import returnEl
from classes.Atmosphere import Atmosphere
from classes.MockHellfireMassAndMotor import MockHellfireMassAndMotor

np.set_printoptions(suppress=True, precision=2)

##################################################################################################################################################

# Input.
INITIAL_ALTITUDE = 1000 # Meters.
INITIAL_AIRSPEED = 130 # Meters per second.

# Constants.
MM_TO_M = 1.0 / 1000.0
RAD_TO_DEG = 57.2957795130823
SEA_LEVEL_PRESSURE = 101325

REFERENCE_LENGTH = 1.6 # Meters.
# REFERENCE_LENGTH = 1.85026 # Meters.
REFERENCE_DIAMETER = 0.18 # Meters.

# Mass and Motor.
MassAndMotor = MockHellfireMassAndMotor()
MassAndMotor.update(0.0, SEA_LEVEL_PRESSURE)
MASS = MassAndMotor.MASS
TMOI = MassAndMotor.TRANSVERSE_MOI
XCG = MassAndMotor.XCG

# Atmosphere.
Atm = Atmosphere()
Atm.update(INITIAL_ALTITUDE, INITIAL_AIRSPEED)
RHO = Atm.rho # Pascals.
P = Atm.p
Q = Atm.q
G = Atm.g
A = Atm.a
MACH = Atm.mach

# CALCULATIONS OF CONSTANTS FOR LINEAR EQUATIONS OF MOTION AND DIFFERENTIAL EQUATIONS OF MOTION
NOSE_LENGTH = 0.249733 # Meters.
WING_HALF_SPAN = 66.1175 * MM_TO_M / 2.0 # Meters.
WING_TIP_CHORD = 91.047 * MM_TO_M # Meters.
WING_ROOT_CHORD = 0.123564 # Meters.
TAIL_HALF_SPAN = 71.3548 * MM_TO_M / 2.0 # Meters.
TAIL_TIP_CHORD = 0.387894 # Meters.
TAIL_ROOT_CHORD = 0.48084 # Meters.
DISTANCE_FROM_BASE_OF_NOSE_TO_WING = 0.323925 # Meters.
STARTING_CG_FROM_NOSE =  0.644605 # Meters.
LAST_CG_FROM_NOSE =  NOSE_LENGTH # Meters.
CENTER_OF_DEFLECTION_FROM_NOSE = 1.8059 - NOSE_LENGTH # Meters (correction here due to oversight in drawing).
# CENTER_OF_DEFLECTION_FROM_NOSE = 1.8059 # Meters.
WING_AREA = 0.5 * WING_HALF_SPAN * (WING_TIP_CHORD + WING_ROOT_CHORD) # Meters squared.
TAIL_AREA = 0.5 * TAIL_HALF_SPAN * (TAIL_TIP_CHORD + TAIL_ROOT_CHORD) # Meters squared.
REFERENCE_AREA = np.pi * (REFERENCE_DIAMETER ** 2) / 4 # Meters squared.
NOSE_AREA = NOSE_LENGTH * REFERENCE_DIAMETER # Meters squared.
PLANFORM_AREA = (REFERENCE_LENGTH - NOSE_LENGTH) * REFERENCE_DIAMETER + 0.667 * NOSE_LENGTH * REFERENCE_DIAMETER # Meters squared.
MACH = INITIAL_AIRSPEED / A # Non dimensional.
if MACH > 1:
	BETA = np.sqrt(MACH ** 2 - 1) # Non dimensional.
else:
	BETA = MACH # Non dimensional.
NOSE_CENTER_OF_PRESSURE = 0.67 * NOSE_LENGTH # Meters.
WING_CENTER_OF_PRESSURE = NOSE_LENGTH + DISTANCE_FROM_BASE_OF_NOSE_TO_WING + 0.7 * WING_ROOT_CHORD - 0.2 * WING_TIP_CHORD # Meters.
AN = 0.67 * NOSE_LENGTH * REFERENCE_DIAMETER # Meters squared.
AB = (REFERENCE_LENGTH - NOSE_LENGTH) * REFERENCE_DIAMETER # Meters squared.
BODY_CENTER_OF_PRESSURE = (0.67 * AN * NOSE_LENGTH + AB * (NOSE_LENGTH + 0.5 * (REFERENCE_LENGTH - NOSE_LENGTH))) / (AN + AB) # Meters.
TEMP1 = (XCG - WING_CENTER_OF_PRESSURE) / REFERENCE_DIAMETER
TEMP2 = (XCG - CENTER_OF_DEFLECTION_FROM_NOSE) / REFERENCE_DIAMETER
TEMP3 = (XCG - BODY_CENTER_OF_PRESSURE) / REFERENCE_DIAMETER
TEMP4 = (XCG - NOSE_CENTER_OF_PRESSURE) / REFERENCE_DIAMETER
Y1 = 2 + 8 * WING_AREA / (BETA * REFERENCE_AREA) + 8 * TAIL_AREA / (BETA * REFERENCE_AREA)
Y2 = 1.5 * PLANFORM_AREA / REFERENCE_AREA
Y3 = 8 * TAIL_AREA / (BETA * REFERENCE_AREA)
Y4 = 2 * TEMP4 + 8 * WING_AREA * TEMP1 / (BETA * REFERENCE_AREA) + 8 * TAIL_AREA * TEMP2 / (BETA * REFERENCE_AREA)
Y5 = 1.5 * PLANFORM_AREA * TEMP3 / REFERENCE_AREA
Y6 = 8 * TAIL_AREA * TEMP2 / (BETA * REFERENCE_AREA)
P2 = Y2 - (Y3 * Y5) / Y6
P3 = (Y1 - (Y3 * Y4) / Y6)

##################################################################################################################################################

TIME_STEP = 0.001 # Seconds.
MAX_TIME = 10 # Seconds.

# STATE.
TOF = 0.0
DEFL = 0.0
E_0 = 0.0
EDOT_0 = 0.0
THT_0 = 0.0
POS_0 = npa([0.0, INITIAL_ALTITUDE])
VEL_0 = npa([INITIAL_AIRSPEED, 90.0])

# TARGET.
TGT_POS = npa([3000.0, 2000.0])
TGT_VEL = np.zeros(2)
TGT_ACC = np.zeros(2)

while TOF < MAX_TIME:

	# Target update.
	TGT_VEL += (TGT_ACC * TIME_STEP)
	TGT_POS += (TGT_VEL * TIME_STEP)

	# Set up.
	speed = la.norm(VEL_0)
	velU = unitvector(VEL_0)
	FLIGHT_PATH_ANGLE = returnEl(velU[1], velU[0])
	mslLocalOrient = ct.BODY_TO_RANGE_AND_ALTITUDE(-FLIGHT_PATH_ANGLE)
	
	# Guidance.
	relPos = TGT_POS - POS_0
	relVel = TGT_VEL - VEL_0
	rightUpInterceptorToIntercept = mslLocalOrient @ relPos
	rightUpInterceptorToInterceptU = unitvector(rightUpInterceptorToIntercept)
	rightUpInterceptorToInterceptMag = la.norm(rightUpInterceptorToIntercept)
	rightUpInterceptorToInterceptVel = mslLocalOrient @ relVel
	rightUpInterceptorToInterceptVelU = unitvector(rightUpInterceptorToInterceptVel)
	rightUpInterceptorToInterceptVelMag = la.norm(rightUpInterceptorToInterceptVel)
	T1 = np.cross(rightUpInterceptorToIntercept, rightUpInterceptorToInterceptVel)
	T2 = np.dot(rightUpInterceptorToIntercept, rightUpInterceptorToIntercept)
	omega = T1 / T2
	normalAccCommand = 2 * omega * rightUpInterceptorToInterceptVelMag
	accCommMag = np.abs(normalAccCommand)
	limit = 5
	if accCommMag > limit:
		new = limit * np.sign(normalAccCommand)
		normalAccCommand = new

	# Aerodynamics.
	CNTRIM = MASS * normalAccCommand / (Q * REFERENCE_AREA)
	alphaTrim = (-1 * P3 + np.sqrt(P3 * P3 + 4 * P2 * CNTRIM)) / (2 * P2)
	deltaTrim = (-1 * Y4 * alphaTrim - Y5 * alphaTrim * alphaTrim) / Y6
	CNA = 2 + 1.5 * PLANFORM_AREA * alphaTrim / REFERENCE_AREA + 8 * WING_AREA / (BETA * REFERENCE_AREA) + 8 * TAIL_AREA / (BETA * REFERENCE_AREA)
	CND = 8 * TAIL_AREA / (BETA * REFERENCE_AREA)
	CMAP = 2 * TEMP4 + 1.5 * PLANFORM_AREA * alphaTrim * TEMP3 / REFERENCE_AREA + 8 * WING_AREA * TEMP1 / (BETA * REFERENCE_AREA)
	CMA = CMAP + 8 * TAIL_AREA * TEMP2 / (BETA * REFERENCE_AREA)
	CMD = 8 * TAIL_AREA * TEMP2 / (BETA * REFERENCE_AREA)
	ZA = -1 * G * Q * REFERENCE_AREA * CNA / (MASS * speed)
	ZD = -1 * G * Q * REFERENCE_AREA * CND / (MASS * speed)
	# ZA = -1 * Q * REFERENCE_AREA * CNA / (MASS * speed)
	# ZD = -1 * Q * REFERENCE_AREA * CND / (MASS * speed)
	MA = Q * REFERENCE_AREA * REFERENCE_DIAMETER * CMA / TMOI
	MD = Q * REFERENCE_AREA * REFERENCE_DIAMETER * CMD / TMOI

	# Control
	KR = 0.15
	K1 = -1 * speed * ((MA * ZD - ZA * MD) / (1845 * MA))
	TA = MD / (MA * ZD - MD * ZA)
	K3 = 1845 * K1 / speed # 1845 = Some kind of gain.
	KDC = (1 - KR * K3) / (K1 * KR)
	THD = K3 * (E_0 + TA * EDOT_0)
	omegaZ = np.sqrt((MA * ZD - MD * ZA) / ZD)
	omegaAF = np.sqrt(-1 * MA)
	zetaAF = ZA * omegaAF / (2 * MA)

	# Derivatives.
	EDOTDOT_0 = (omegaAF ** 2) * (DEFL - E_0 - 2 * zetaAF * EDOT_0 / omegaAF)
	NORMAL_SPECIFIC_FORCE = K1 * (E_0 - (EDOTDOT_0 / (omegaZ ** 2))) * G
	BODYACC = npa([0.0, NORMAL_SPECIFIC_FORCE])
	ACC_0 = BODYACC @ mslLocalOrient
	THD_0 = THD

	# Integration.
	E_0 += EDOT_0 * TIME_STEP
	EDOT_0 += EDOTDOT_0 * TIME_STEP
	THT_0 += THD_0 * TIME_STEP
	POS_0 += VEL_0 * TIME_STEP
	VEL_0 += ACC_0 * TIME_STEP
	TOF += TIME_STEP

	# Actuator.
	DEFL = KR * (KDC * normalAccCommand - THD)

	# Console report.
	if round(TOF, 3).is_integer():
		print(f"TOF {TOF:.1f} POS {POS_0}")

	# End check.
	MISS_DISTANCE = la.norm(rightUpInterceptorToIntercept)
	if POS_0[0] < 0.0:
		print(f"{TOF} GROUND")
		break
	elif np.isnan(np.sum(POS_0)):
		print(f"{TOF} NAN")
		break
	elif MISS_DISTANCE < 5.0:
		print(f"{TOF} INTERCEPT")
		break
	elif rightUpInterceptorToIntercept[0] < 0.0:
		print(f"{TOF} POCA")
		break






# # Constants.
# gravity = STANDARD_GRAVITY
# refDiameter = REFERENCE_DIAMETER
# noseLength = NOSE_LENGTH
# refLength = REFERENCE_LENGTH
# wingHalfSpan = WING_HALF_SPAN
# wingTipChord = WING_TIP_CHORD
# wingRootChord = WING_ROOT_CHORD
# tailHalfSpan = TAIL_HALF_SPAN
# tailTipChord = TAIL_TIP_CHORD
# tailRootChord = TAIL_ROOT_CHORD
# distanceFromBaseOfNoseToWing = DISTANCE_FROM_BASE_OF_NOSE_TO_WING
# centerOfGravityFromNose = LAST_CG_FROM_NOSE
# centerOfDeflectionFromNose = CENTER_OF_DEFLECTION_FROM_NOSE
# weight = MASS

# wingArea = 0.5 * wingHalfSpan * (wingTipChord + wingRootChord)
# tailArea = 0.5 * tailHalfSpan * (tailTipChord + tailRootChord)
# refArea = np.pi * (refDiameter ** 2) / 4
# noseArea = noseLength * refDiameter
# planformArea = (refLength - noseLength) * refDiameter + 0.667 * noseLength * refDiameter
# noseCenterOfPressure = 0.67 * noseLength
# wingCenterOfPressure = noseLength + distanceFromBaseOfNoseToWing + 0.7 * wingRootChord - 0.2 * wingTipChord
# AN = 0.67 * noseLength * refDiameter
# AB = (refLength - noseLength) * refDiameter
# bodyCenterOfPressure = (0.67 * AN * noseLength + AB * (noseLength + 0.5 * (refLength - noseLength))) / (AN + AB)
# transverseMomentOfInertia = TRANSVERSE_MOMENT_OF_INERTIA
# TEMP1 = (centerOfGravityFromNose - wingCenterOfPressure) / refDiameter
# TEMP2 = (centerOfGravityFromNose - centerOfDeflectionFromNose) / refDiameter
# TEMP3 = (centerOfGravityFromNose - bodyCenterOfPressure) / refDiameter
# TEMP4 = (centerOfGravityFromNose - noseCenterOfPressure) / refDiameter

# # Missile.
# missileTof = 0.0
# missilePos = npa([0.0, 0.0])
# missileVel = npa([400.0, 1000.0])
# missileAcc = npa([0.0, 0.0])

# # Target.
# targetPos = npa([10000.0, 10000.0])
# targetVel = npa([-80.0, 0.0])

# # Simulation control.
# timeStep = 0.001
# go = True
# simData = {"TOF": [], "X": [], "Y": [], "TX": [], "TY": [], "COMMAND": [], "ACHIEVED": []}
# e = 0
# eDot = 0

# while go:

# 	# Target update.
# 	# targetAcc = np.random.randint(-250, 1, 2)
# 	targetAcc = np.zeros(2)
# 	targetVel += (targetAcc * timeStep)
# 	targetPos += (targetVel * timeStep)

# 	# Time of flight.
# 	missileTof += timeStep

# 	# Guidance.
# 	relPos = targetPos - missilePos
# 	relVel = targetVel - missileVel
# 	velU = unitvector(missileVel)
# 	velMag = la.norm(missileVel)
# 	fpa = returnEl(velU[1], velU[0])
# 	mslLocalOrient = ct.BODY_TO_RANGE_AND_ALTITUDE(-fpa)
# 	rightUpInterceptorToIntercept = mslLocalOrient @ relPos
# 	rightUpInterceptorToInterceptU = unitvector(rightUpInterceptorToIntercept)
# 	rightUpInterceptorToInterceptMag = la.norm(rightUpInterceptorToIntercept)
# 	rightUpInterceptorToInterceptVel = mslLocalOrient @ relVel
# 	rightUpInterceptorToInterceptVelU = unitvector(rightUpInterceptorToInterceptVel)
# 	rightUpInterceptorToInterceptVelMag = la.norm(rightUpInterceptorToInterceptVel)
	
# 	T1 = np.cross(rightUpInterceptorToIntercept, rightUpInterceptorToInterceptVel)
# 	T2 = np.dot(rightUpInterceptorToIntercept, rightUpInterceptorToIntercept)
# 	omega = T1 / T2
# 	normalAccCommand = 2 * omega * rightUpInterceptorToInterceptVelMag
# 	accCommMag = np.abs(normalAccCommand)
# 	limit = 5
# 	if accCommMag > limit:
# 		new = limit * np.sign(normalAccCommand)
# 		normalAccCommand = new

# 	# Atmosphere.
# 	altitude = missilePos[1] # Feet.
# 	atmosphere = atm(altitude)
# 	speedOfSound = 3.28084 * atmosphere.speed_of_sound[0] # Feet per second.
# 	mach = velMag / speedOfSound
# 	if altitude <= 30000:
# 		rho = 0.002378 * np.exp(-altitude / 30000)
# 	else:
# 		rho = 0.0034 * np.exp(-altitude / 22000)
# 	dynamicPressure = 0.5 * rho * velMag * velMag

# 	# Aerodynamics.
# 	beta = np.sqrt(np.abs(mach ** 2 - 1))
# 	CNTRIM = weight * normalAccCommand / (dynamicPressure * refArea)
# 	Y1 = 2 + 8 * wingArea / (beta * refArea) + 8 * tailArea / (beta * refArea)
# 	Y2 = 1.5 * planformArea / refArea
# 	Y3 = 8 * tailArea / (beta * refArea)
# 	Y4 = 2 * TEMP4 + 8 * wingArea * TEMP1 / (beta * refArea) + 8 * tailArea * TEMP2 / (beta * refArea)
# 	Y5 = 1.5 * planformArea * TEMP3 / refArea
# 	Y6 = 8 * tailArea * TEMP2 / (beta * refArea)
# 	P2 = Y2 - (Y3 * Y5) / Y6
# 	P3 = Y1 - (Y3 * Y4) / Y6
# 	alphaTrim = (-1 * P3 + np.sqrt(P3 * P3 + 4 * P2 * CNTRIM)) / (2 * P2)
# 	deltaTrim = (-1 * Y4 * alphaTrim - Y5 * alphaTrim * alphaTrim) / Y6
# 	CNA = 2 + 1.5 * planformArea * alphaTrim / refArea + 8 * wingArea / (beta * refArea) + 8 * tailArea / (beta * refArea)
# 	CND = 8 * tailArea / (beta * refArea)
# 	CMAP = 2 * TEMP4 + 1.5 * planformArea * alphaTrim * TEMP3 / refArea + 8 * wingArea * TEMP1 / (beta * refArea)
# 	CMA = CMAP + 8 * tailArea * TEMP2 / (beta * refArea)
# 	CMD = 8 * tailArea * TEMP2 / (beta * refArea)
# 	ZA = -1 * gravity * dynamicPressure * refArea * CNA / (weight * velMag)
# 	ZD = -1 * gravity * dynamicPressure * refArea * CND / (weight * velMag)
# 	MA = dynamicPressure * refArea * refDiameter * CMA / transverseMomentOfInertia
# 	MD = dynamicPressure * refArea * refDiameter * CMD / transverseMomentOfInertia

# 	# Control
# 	KR = 0.15
# 	K1 = -1 * velMag * ((MA * ZD - ZA * MD) / (1845 * MA))
# 	TA = MD / (MA * ZD - MD * ZA)
# 	K3 = 1845 * K1 / velMag # 1845 = Some kind of gain.
# 	KDC = (1 - KR * K3) / (K1 * KR)
# 	THD = K3 * (e + TA * eDot)

# 	# Actuator.
# 	deflection = KR * (KDC * normalAccCommand + THD)

# 	# Motion.
# 	eOld = e
# 	eDotOld = eDot
# 	omegaZ = np.sqrt((MA * ZD - MD * ZA) / ZD)
# 	omegaAF = np.sqrt(-1 * MA)
# 	zetaAF = ZA * omegaAF / (2 * MA)
# 	eDotDot = (omegaAF ** 2) * (deflection - e - 2 * zetaAF * eDot / omegaAF)
# 	normalAccel = K1 * (e - (eDotDot / (omegaZ ** 2))) * gravity
# 	e += timeStep * eDot
# 	eDot += timeStep * eDotDot
# 	e = (e + eOld + timeStep * eDot) / 2
# 	eDot = (eDot + eDotOld + timeStep * eDotDot) / 2

# 	# Euler semi implicit integration.
# 	bodyAcc = npa([0.0, normalAccel])
# 	localAcc = bodyAcc @ mslLocalOrient
# 	deltaVel = localAcc * timeStep
# 	missileVel += deltaVel
# 	deltaPos = missileVel * timeStep
# 	missilePos += deltaPos

# 	# Store data.
# 	simData["TOF"].append(missileTof)
# 	simData["X"].append(missilePos[0])
# 	simData["Y"].append(missilePos[1])
# 	simData["TX"].append(targetPos[0])
# 	simData["TY"].append(targetPos[1])
# 	simData["COMMAND"].append(normalAccCommand)
# 	simData["ACHIEVED"].append(normalAccel / gravity)

# 	# Console report.
# 	if round(missileTof, 3).is_integer():
# 		print(f"TOF {missileTof:.1f} POS {missilePos}")

# 	if rightUpInterceptorToIntercept[0] < 1.0:
# 		print(f"{missileTof:.2f} MISS DISTANCE {rightUpInterceptorToIntercept} {rightUpInterceptorToInterceptMag}")
# 		break

# # Visual.
# fig = plt.figure()

# trajectory = fig.add_subplot(121)
# trajectory.set_xlabel("Down Range (ft).")
# trajectory.set_ylabel("Altitude (ft).")
# trajectory.plot(simData["X"], simData["Y"], label="Interceptor", color="b")
# trajectory.plot(simData["TX"], simData["TY"], label="Threat", color="r")
# trajectory.scatter(targetPos[0], targetPos[1], color="r")
# trajectory.legend()

# accels = fig.add_subplot(122)
# # accels.set_ylim([-30, 30])
# accels.plot(simData["TOF"], simData["COMMAND"], label="Acceleration command (Gs).", color="r")
# accels.plot(simData["TOF"], simData["ACHIEVED"], label="Acceleration achieved (Gs).", color="b")
# accels.legend()

# # plt.get_current_fig_manager().full_screen_toggle()
# plt.show()