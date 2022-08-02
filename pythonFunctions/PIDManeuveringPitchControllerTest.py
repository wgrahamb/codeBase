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

# Missile constant characteristics.
REFERENCE_DIAMETER = 1 # FEET
NOSE_LENGTH = 3 # FEET
REFERENCE_LENGTH = 20 # FEET
WING_HALF_SPAN = 2 # FEET
WING_TIP_CHORD = 0 # FEET
WING_ROOT_CHORD = 6 # FEET
TAIL_HALF_SPAN = 2 # FEET
TAIL_TIP_CHORD = 0 # FEET
TAIL_ROOT_CHORD = 2 # FEET
DISTANCE_FROM_BASE_OF_NOSE_TO_WING = 4 # FEET
CENTER_OF_GRAVITY_FROM_NOSE = 10 # FEET
CENTER_OF_DEFLECTION_FROM_NOSE = 19.5 # FEET

# Missile calculated constant characteristics.
WING_AREA = 0.5 * WING_HALF_SPAN * (WING_TIP_CHORD + WING_ROOT_CHORD)
TAIL_AREA = 0.5 * TAIL_HALF_SPAN * (TAIL_TIP_CHORD + TAIL_ROOT_CHORD)
REFERENCE_AREA = np.pi * (REFERENCE_DIAMETER ** 2) / 4
NOSE_AREA = NOSE_LENGTH * REFERENCE_DIAMETER
PLANFORM_AREA = (REFERENCE_LENGTH - NOSE_LENGTH) * REFERENCE_DIAMETER + 0.667 * NOSE_LENGTH * REFERENCE_DIAMETER
NOSE_CENTER_OF_PRESSURE = 0.67 * NOSE_LENGTH
WING_CENTER_OF_PRESSURE = NOSE_LENGTH + DISTANCE_FROM_BASE_OF_NOSE_TO_WING + 0.7 * WING_ROOT_CHORD - 0.2 * WING_TIP_CHORD
AN = 0.67 * NOSE_LENGTH * REFERENCE_DIAMETER
AB = (REFERENCE_LENGTH - NOSE_LENGTH) * REFERENCE_DIAMETER
BODY_CENTER_OF_PRESSURE = (0.67 * AN * NOSE_LENGTH + AB * (NOSE_LENGTH + 0.5 * (REFERENCE_LENGTH - NOSE_LENGTH))) / (AN + AB)
TEMP1 = (CENTER_OF_GRAVITY_FROM_NOSE - WING_CENTER_OF_PRESSURE) / REFERENCE_DIAMETER
TEMP2 = (CENTER_OF_GRAVITY_FROM_NOSE - CENTER_OF_DEFLECTION_FROM_NOSE) / REFERENCE_DIAMETER
TEMP3 = (CENTER_OF_GRAVITY_FROM_NOSE - BODY_CENTER_OF_PRESSURE) / REFERENCE_DIAMETER
TEMP4 = (CENTER_OF_GRAVITY_FROM_NOSE - NOSE_CENTER_OF_PRESSURE) / REFERENCE_DIAMETER

class target:

	def __init__(self, targetRightUpPosition, targetRightUpVelocity):

		self.timeOfFlight = 0
		self.timeStep = 0.01
		self.rightUpTargetPosition = targetRightUpPosition
		self.rightUpTargetVelocity = targetRightUpVelocity
		print("TARGET CONSTRUCTED")

	def update(self, simTime):
		while self.timeOfFlight < simTime:
			deltaPos = self.timeStep * self.rightUpTargetVelocity
			self.rightUpTargetPosition += deltaPos
			self.timeOfFlight += self.timeStep

class atmosphere:

	def __init__(self):

		self.rho = None
		self.p = None
		self.a = None
		self.gravity = None
		self.q = None
		self.mach = None

		print("ATMOSPHERE CONSTRUCTED")

	def update(self, altitude, speed, unitFlag):

		atmos = atm(altitude)
		rho = atmos.density[0] # kg / m^3
		p = atmos.pressure[0] # pascals
		a = atmos.speed_of_sound[0] # m/s
		gravity = atmos.grav_accel[0] # m/s^2
		q = 0.5 * rho * speed * speed # pascals
		mach = speed / a

		if unitFlag == "IMPERIAL":
			self.rho = rho * 0.00194032 # kg/m^3 to lb/ft^3
			self.p = p * 0.02 # pascals to psi
			self.a = a * 3.28084 # m/s to ft/s
			self.gravity = gravity * 3.28084 # m/s^2 to ft/s^2
			self.q = q * 0.02 # pascals to psi
			self.mach = mach

		elif unitFlag == "METRIC":
			self.rho = rho
			self.p = p
			self.a = a
			self.gravity = gravity
			self.q = q
			self.mach = mach

class guidance:

	def __init__(self):
		self.guidanceTimer = 0
		self.guidanceTimeStep = 1.0 / 1000.0
		self.command = None
		self.rightUpMissileToTargetRelativePosition = np.zeros(2)
		self.mslLocalOrient = None
		print("GUIDANCE CONSTRUCTED")

	def udpate(self, simTime, targetRightUpPosition, targetRightUpVelocity, missileRightUpPosition, missileRightUpVelocity):
		
		while self.guidanceTimer < simTime:
			self.guidanceTimer += self.guidanceTimeStep
			relPos = targetRightUpPosition - missileRightUpPosition
			relVel = targetRightUpVelocity - missileRightUpVelocity
			velU = unitvector(missileRightUpVelocity)
			velMag = la.norm(missileRightUpVelocity)
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
			self.mslLocalOrient = npa([axis, normal])
			rightUpInterceptorToIntercept = self.mslLocalOrient @ relPos
			self.rightUpMissileToTargetRelativePosition = rightUpInterceptorToIntercept
			rightUpInterceptorToInterceptU = unitvector(rightUpInterceptorToIntercept)
			rightUpInterceptorToInterceptMag = la.norm(rightUpInterceptorToIntercept)
			rightUpInterceptorToInterceptVel = self.mslLocalOrient @ relVel
			rightUpInterceptorToInterceptVelU = unitvector(rightUpInterceptorToInterceptVel)
			rightUpInterceptorToInterceptVelMag = la.norm(rightUpInterceptorToInterceptVel)
			GUIDANCET1 = np.cross(rightUpInterceptorToIntercept, rightUpInterceptorToInterceptVel)
			GUIDANCET2 = np.dot(rightUpInterceptorToIntercept, rightUpInterceptorToIntercept)
			omega = GUIDANCET1 / GUIDANCET2

			self.command = 4 * omega * rightUpInterceptorToInterceptVelMag
			accCommMag = np.abs(self.command)
			if accCommMag > 50:
				new = 50 * np.sign(self.command)
				self.command = new

class control:
	
	def __init__(self):

		self.controlTimer = 0
		self.controlTimeStep = 1 / 1000.0
		self.e = 0.0
		self.eDot = 0.0
		self.deflection = 0.0
		self.normalAccel = 0.0

	def update(self, simTime, gravity, dynamicPressure, mach, speed, weight, TMOI, normalAccCommand):

		while self.controlTimer < simTime:
			self.controlTimer += self.controlTimeStep
			if mach > 1:
				beta = np.sqrt(mach ** 2 - 1)
			else: 
				beta = 0.7
			CNTRIM = weight * normalAccCommand / (dynamicPressure * REFERENCE_AREA)
			Y1 = 2 + 8 * WING_AREA / (beta * REFERENCE_AREA) + 8 * TAIL_AREA / (beta * REFERENCE_AREA)
			Y2 = 1.5 * PLANFORM_AREA / REFERENCE_AREA
			Y3 = 8 * TAIL_AREA / (beta * REFERENCE_AREA)
			Y4 = 2 * TEMP4 + 8 * WING_AREA * TEMP1 / (beta * REFERENCE_AREA) + 8 * TAIL_AREA * TEMP2 / (beta * REFERENCE_AREA)
			Y5 = 1.5 * PLANFORM_AREA * TEMP3 / REFERENCE_AREA
			Y6 = 8 * TAIL_AREA * TEMP2 / (beta * REFERENCE_AREA)
			P2 = Y2 - (Y3 * Y5) / Y6
			P3 = Y1 - (Y3 * Y4) / Y6

			aaa = P3 ** 2
			bbb = 4 * P2 * CNTRIM
			check = aaa + bbb
			sign = np.sign(check)
			if sign < 0:
				bbb = (-aaa + 1 + P3)

			alphaTrim = (-1 * P3 + np.sqrt(aaa + bbb)) / (2 * P2)
			deltaTrim = (-1 * Y4 * alphaTrim - Y5 * alphaTrim * alphaTrim) / Y6
			CNA = 2 + 1.5 * PLANFORM_AREA * alphaTrim / REFERENCE_AREA + 8 * WING_AREA / (beta * REFERENCE_AREA) + 8 * TAIL_AREA / (beta * REFERENCE_AREA)
			CND = 8 * TAIL_AREA / (beta * REFERENCE_AREA)
			CMAP = 2 * TEMP4 + 1.5 * PLANFORM_AREA * alphaTrim * TEMP3 / REFERENCE_AREA + 8 * WING_AREA * TEMP1 / (beta * REFERENCE_AREA)
			CMA = CMAP + 8 * TAIL_AREA * TEMP2 / (beta * REFERENCE_AREA)
			CMD = 8 * TAIL_AREA * TEMP2 / (beta * REFERENCE_AREA)

			ZA = -1 * gravity * dynamicPressure * REFERENCE_AREA * CNA / (weight * speed)
			ZD = -1 * gravity * dynamicPressure * REFERENCE_AREA * CND / (weight * speed)
			MA = dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER * CMA / TMOI
			MD = dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER * CMD / TMOI
			omegaZ = np.sqrt((MA * ZD - MD * ZA) / ZD)
			omegaAF = np.sqrt(-1 * MA)
			zetaAF = ZA * omegaAF / (2 * MA)
			KR = 0.1
			K1 = -1 * speed * ((MA * ZD - ZA * MD) / (1845 * MA))
			TA = MD / (MA * ZD - MD * ZA)
			K3 = 1845 * K1 / speed
			KDC = (1 - KR * K3) / (K1 * KR)
			eOld = self.e
			eDotOld = self.eDot
			THD = K3 * (self.e + TA * self.eDot)

			self.deflection = KR * (KDC * normalAccCommand + THD)
			FinLimit = 75 # Degrees
			if np.abs(self.deflection) > FinLimit:
				new = np.sign(self.deflection) * FinLimit
				self.deflection = new

			eDotDot = (omegaAF ** 2) * (self.deflection - self.e - 2 * zetaAF * self.eDot / omegaAF)
			self.normalAccel = K1 * (self.e - (eDotDot / (omegaZ ** 2)))
			self.e += self.controlTimeStep * self.eDot
			self.eDot += self.controlTimeStep * eDotDot
			self.e = (self.e + eOld + self.controlTimeStep * self.eDot) / 2
			self.eDot = (self.eDot + eDotOld + self.controlTimeStep * eDotDot) / 2


class MissileWithPIDManeuveringPitchController:

	def __init__(self, initialTargetRightUpPosition, initialTargetRightUpVelocity, initialMissileRightUpVelocity):

		# State.
		self.dynamicsTimeOfFlight = 0.0
		self.missileRightUpPosition = npa([0.0, 0.0])
		self.missileRightUpVelocity = initialMissileRightUpVelocity
		self.missileRightUpAcceleration = npa([0.0, 0.0])
		self.dynamicsTimeStep = 0.001
		
		# Simulation control.
		self.simulate = True
		self.simulationTime = 0
		self.simulationTimeStep = 0.02
		self.simData = {"TOF": [], "X": [], "Y": [], "TX": [], "TY": [], "COMMAND": [], "ACHIEVED": [], "FINDEFLECTION": []}

		# Components.
		self.target = target(targetRightUpPosition=initialTargetRightUpPosition, targetRightUpVelocity=initialTargetRightUpVelocity)
		self.atmosphere = atmosphere()
		self.guidance = guidance()
		self.control = control()

	def missileMotion(self, normalAccel, mslLocalOrient):

		# Euler semi implicit integration.
		bodyAcc = npa([0.0, normalAccel])
		self.missileRightUpAcceleration = bodyAcc @ mslLocalOrient

		deltaVel = self.missileRightUpAcceleration * self.dynamicsTimeStep
		self.missileRightUpVelocity += deltaVel

		deltaPos = self.missileRightUpVelocity * self.dynamicsTimeStep
		self.missileRightUpPosition += deltaPos

		self.dynamicsTimeOfFlight += self.dynamicsTimeStep

	def logData(self):
			# Store data.
			self.simData["TOF"].append(self.dynamicsTimeOfFlight)
			self.simData["X"].append(self.missileRightUpPosition[0])
			self.simData["Y"].append(self.missileRightUpPosition[1])
			self.simData["TX"].append(self.target.rightUpTargetPosition[0])
			self.simData["TY"].append(self.target.rightUpTargetPosition[1])
			self.simData["COMMAND"].append(self.guidance.command)
			self.simData["ACHIEVED"].append(la.norm(self.control.normalAccel))
			self.simData["FINDEFLECTION"].append(self.control.deflection)

	def update(self):
		
		self.simulationTime += self.simulationTimeStep
		self.target.update(self.simulationTime)
		self.atmosphere.update(self.missileRightUpPosition[1], la.norm(self.missileRightUpVelocity), "IMPERIAL")
		self.guidance.udpate(self.simulationTime ,self.target.rightUpTargetPosition, self.target.rightUpTargetVelocity, self.missileRightUpPosition, self.missileRightUpVelocity)
		weight = 800 # LBF
		transverseMomentOfInertia = (weight * (3 * ((0.5 * REFERENCE_DIAMETER) ** 2) + REFERENCE_LENGTH ** 2)) / (12 * self.atmosphere.gravity)
		# self.control.update(self.simulationTime, self.atmosphere.gravity, self.atmosphere.q, self.atmosphere.mach, la.norm(self.missileRightUpVelocity), weight, transverseMomentOfInertia, self.guidance.command)
		while self.dynamicsTimeOfFlight < self.simulationTime:
			self.missileMotion(self.guidance.command, self.guidance.mslLocalOrient)
		self.logData()

	def fly(self, maxTime):
		while self.simulationTime < maxTime:
			print(self.simulationTime)
			self.update()
			if round(self.dynamicsTimeOfFlight, 4).is_integer():
				print(f"TOF {self.dynamicsTimeOfFlight:.1f} POS {self.missileRightUpPosition}")

	def plot(self):

		# Visual.
		fig = plt.figure()

		trajectory = fig.add_subplot(121)
		trajectory.set_xlabel("Down Range (ft).")
		trajectory.set_ylabel("Altitude (ft).")
		trajectory.plot(self.simData["X"], self.simData["Y"], label="Interceptor", color="b")
		trajectory.plot(self.simData["TX"], self.simData["TY"], label="Threat", color="r")
		trajectory.scatter(self.simData["TX"][-1], self.simData["TY"][-1], color="r")
		trajectory.legend()

		accels = fig.add_subplot(122)
		accels.set_ylim([-30, 30])
		accels.plot(self.simData["TOF"], self.simData["COMMAND"], label="Acceleration command (m/s^2).", color="r")
		accels.plot(self.simData["TOF"], self.simData["ACHIEVED"], label="Acceleration achieved (m/s^2).", color="b")
		accels.plot(self.simData["TOF"], self.simData["FINDEFLECTION"], label="Fin deflection ().", color="g")
		accels.legend()

		plt.show()


if __name__ == "__main__":
	initialTargetRightUpPosition = npa([5000.0, 5000.0])
	initialTargetRightUpVelocity = npa([-110.0, -90.0])
	initialMissileRightUpVelocity = npa([400.0, 400.0])
	x = MissileWithPIDManeuveringPitchController(initialTargetRightUpPosition=initialTargetRightUpPosition, initialTargetRightUpVelocity=initialTargetRightUpVelocity, initialMissileRightUpVelocity=initialMissileRightUpVelocity)
	x.fly(20.0)
	x.plot()

# # Need to be inputs.
# gravity = 32.2 # FEET PER S^2 >>> ASSUMED CONSTANT
# weight = 800 # LBF
# transverseMomentOfInertia = (weight * (3 * ((0.5 * REFERENCE_DIAMETER) ** 2) + REFERENCE_LENGTH ** 2)) / (12 * gravity)

# # State.
# timeOfFlight = 0.0
# missileRightUpPosition = npa([0.0, 1000.0])
# missileRightUpVelocity = npa([150.0, 50.0])
# missileRightUpAcceleration = npa([0.0, 0.0])
# targetRightUpPosition = npa([30000.0, 30000.0])
# targetRightUpVelocity = npa([-800.0, -800.0])
# timeStep = 0.005
# simulate = True
# simData = {"TOF": [], "X": [], "Y": [], "TX": [], "TY": [], "COMMAND": [], "ACHIEVED": [], "FINDEFLECTION": []}
# e = 0
# eDot = 0

# while simulate:

# 	# Target update.
# 	targetAcc = np.random.randint(-10, 10, 2)
# 	targetRightUpVelocity += (targetAcc * timeStep)
# 	targetRightUpPosition += (targetRightUpVelocity * timeStep)

# 	# Time of flight.
# 	timeOfFlight += timeStep

# 	# Guidance.
# 	relPos = targetRightUpPosition - missileRightUpPosition
# 	relVel = targetRightUpVelocity - missileRightUpVelocity
# 	velU = unitvector(missileRightUpVelocity)
# 	velMag = la.norm(missileRightUpVelocity)
# 	axis = npa(
# 		[
# 			velU[0],
# 			velU[1]
# 		]
# 	)
# 	normal = npa(
# 		[
# 			-1 * velU[1],
# 			velU[0]
# 		]
# 	)
# 	mslLocalOrient = npa([axis, normal])
# 	rightUpInterceptorToIntercept = mslLocalOrient @ relPos
# 	rightUpInterceptorToInterceptU = unitvector(rightUpInterceptorToIntercept)
# 	rightUpInterceptorToInterceptMag = la.norm(rightUpInterceptorToIntercept)
# 	rightUpInterceptorToInterceptVel = mslLocalOrient @ relVel
# 	rightUpInterceptorToInterceptVelU = unitvector(rightUpInterceptorToInterceptVel)
# 	rightUpInterceptorToInterceptVelMag = la.norm(rightUpInterceptorToInterceptVel)
# 	if rightUpInterceptorToIntercept[0] < 1.0:
# 		print(f"{timeOfFlight:.2f} MISS DISTANCE {rightUpInterceptorToIntercept} {rightUpInterceptorToInterceptMag}")
# 		break
# 	GUIDANCET1 = np.cross(rightUpInterceptorToIntercept, rightUpInterceptorToInterceptVel)
# 	GUIDANCET2 = np.dot(rightUpInterceptorToIntercept, rightUpInterceptorToIntercept)
# 	omega = GUIDANCET1 / GUIDANCET2
# 	normalAccCommand = 4 * omega * rightUpInterceptorToInterceptVelMag
# 	accCommMag = np.abs(normalAccCommand)
# 	if accCommMag > 25:
# 		new = 25 * np.sign(normalAccCommand)
# 		normalAccCommand = new

# 	# Atmosphere.
# 	altitude = missileRightUpPosition[1] # Feet.
# 	atmosphere = atm(altitude)
# 	speedOfSound = 3.28084 * atmosphere.speed_of_sound[0] # Feet per second.
# 	mach = velMag / speedOfSound
# 	if altitude <= 30000:
# 		rho = 0.002378 * np.exp(-altitude / 30000)
# 	else:
# 		rho = 0.0034 * np.exp(-altitude / 22000)
# 	dynamicPressure = 0.5 * rho * velMag * velMag


# 	# Autopilot. Aerodynamic feedback coefficients are calculated on line but other sims have look up tables.
# 	if mach > 1:
# 		beta = np.sqrt(mach ** 2 - 1)
# 	else: 
# 		beta = 0.7

# 	CNTRIM = weight * normalAccCommand / (dynamicPressure * REFERENCE_AREA)
# 	Y1 = 2 + 8 * WING_AREA / (beta * REFERENCE_AREA) + 8 * TAIL_AREA / (beta * REFERENCE_AREA)
# 	Y2 = 1.5 * PLANFORM_AREA / REFERENCE_AREA
# 	Y3 = 8 * TAIL_AREA / (beta * REFERENCE_AREA)
# 	Y4 = 2 * TEMP4 + 8 * WING_AREA * TEMP1 / (beta * REFERENCE_AREA) + 8 * TAIL_AREA * TEMP2 / (beta * REFERENCE_AREA)
# 	Y5 = 1.5 * PLANFORM_AREA * TEMP3 / REFERENCE_AREA
# 	Y6 = 8 * TAIL_AREA * TEMP2 / (beta * REFERENCE_AREA)
# 	P2 = Y2 - (Y3 * Y5) / Y6
# 	P3 = Y1 - (Y3 * Y4) / Y6

# 	aaa = P3 ** 2
# 	bbb = 4 * P2 * CNTRIM

# 	# Limit TEMP2, Limiting derivative of CNTRIM, Limiting Guidance Command.
# 	check = aaa + bbb
# 	sign = np.sign(check)
# 	if sign < 0:
# 		bbb = (-aaa + 1 + P3)

# 	alphaTrim = (-1 * P3 + np.sqrt(aaa + bbb)) / (2 * P2)
# 	deltaTrim = (-1 * Y4 * alphaTrim - Y5 * alphaTrim * alphaTrim) / Y6
# 	CNA = 2 + 1.5 * PLANFORM_AREA * alphaTrim / REFERENCE_AREA + 8 * WING_AREA / (beta * REFERENCE_AREA) + 8 * TAIL_AREA / (beta * REFERENCE_AREA)
# 	CND = 8 * TAIL_AREA / (beta * REFERENCE_AREA)
# 	CMAP = 2 * TEMP4 + 1.5 * PLANFORM_AREA * alphaTrim * TEMP3 / REFERENCE_AREA + 8 * WING_AREA * TEMP1 / (beta * REFERENCE_AREA)
# 	CMA = CMAP + 8 * TAIL_AREA * TEMP2 / (beta * REFERENCE_AREA)
# 	CMD = 8 * TAIL_AREA * TEMP2 / (beta * REFERENCE_AREA)

# 	# I'm not sure what "e" is or the "1845" value. Gonna have to read.
# 	ZA = -1 * gravity * dynamicPressure * REFERENCE_AREA * CNA / (weight * velMag)
# 	ZD = -1 * gravity * dynamicPressure * REFERENCE_AREA * CND / (weight * velMag)
# 	MA = dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER * CMA / transverseMomentOfInertia
# 	MD = dynamicPressure * REFERENCE_AREA * REFERENCE_DIAMETER * CMD / transverseMomentOfInertia
# 	omegaZ = np.sqrt((MA * ZD - MD * ZA) / ZD)

# 	omegaAF = np.sqrt(-1 * MA)

# 	zetaAF = ZA * omegaAF / (2 * MA)
# 	KR = 0.1
# 	K1 = -1 * velMag * ((MA * ZD - ZA * MD) / (1845 * MA))
# 	TA = MD / (MA * ZD - MD * ZA)
# 	K3 = 1845 * K1 / velMag
# 	KDC = (1 - KR * K3) / (K1 * KR)
# 	eOld = e
# 	eDotOld = eDot
# 	THD = K3 * (e + TA * eDot)

# 	deflection = KR * (KDC * normalAccCommand + THD)
# 	FinLimit = 75
# 	if np.abs(deflection) > FinLimit:
# 		new = np.sign(deflection) * FinLimit
# 		deflection = new

# 	eDotDot = (omegaAF ** 2) * (deflection - e - 2 * zetaAF * eDot / omegaAF)
# 	normalAccel = K1 * (e - (eDotDot / (omegaZ ** 2)))
# 	e += timeStep * eDot
# 	eDot += timeStep * eDotDot
# 	e = (e + eOld + timeStep * eDot) / 2
# 	eDot = (eDot + eDotOld + timeStep * eDotDot) / 2


# 	# Euler semi implicit integration.
# 	bodyAcc = npa([0.0, normalAccel])
# 	localAcc = bodyAcc @ mslLocalOrient
# 	deltaVel = localAcc * timeStep
# 	missileRightUpVelocity += deltaVel
# 	deltaPos = missileRightUpVelocity * timeStep
# 	missileRightUpPosition += deltaPos

# 	# Store data.
# 	simData["TOF"].append(timeOfFlight)
# 	simData["X"].append(missileRightUpPosition[0])
# 	simData["Y"].append(missileRightUpPosition[1])
# 	simData["TX"].append(targetRightUpPosition[0])
# 	simData["TY"].append(targetRightUpPosition[1])
# 	simData["COMMAND"].append(normalAccCommand)
# 	simData["ACHIEVED"].append(normalAccel)
# 	simData["FINDEFLECTION"].append(deflection)

# 	# Console report.
# 	if round(timeOfFlight, 3).is_integer():
# 		print(f"TOF {timeOfFlight:.1f} POS {missileRightUpPosition}")

# # Visual.
# fig = plt.figure()

# trajectory = fig.add_subplot(121)
# trajectory.set_xlabel("Down Range (ft).")
# trajectory.set_ylabel("Altitude (ft).")
# trajectory.plot(simData["X"], simData["Y"], label="Interceptor", color="b")
# trajectory.plot(simData["TX"], simData["TY"], label="Threat", color="r")
# trajectory.scatter(targetRightUpPosition[0], targetRightUpPosition[1], color="r")
# trajectory.legend()

# accels = fig.add_subplot(122)
# accels.set_ylim([-30, 30])
# accels.plot(simData["TOF"], simData["COMMAND"], label="Acceleration command (m/s^2).", color="r")
# accels.plot(simData["TOF"], simData["ACHIEVED"], label="Acceleration achieved (m/s^2).", color="b")
# accels.plot(simData["TOF"], simData["FINDEFLECTION"], label="Fin deflection ().", color="g")
# accels.legend()

# # plt.get_current_fig_manager().full_screen_toggle()
# plt.show()