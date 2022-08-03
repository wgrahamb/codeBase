import numpy as np
import matplotlib.pyplot as plt
from numpy import array as npa
from unitVector import unitvector
from numpy import linalg as la
from ambiance import Atmosphere as atm
import math

np.set_printoptions(suppress=True, precision=2)

# Simulation constants.
TIME_STEP = 1.0 / 500.0
MAX_TIME = 100.0
INTEGRATION_PASS = 0 # RK2 Integration. Controlled by Missile::motion().

# Math constants.
STANDARD_GRAVITY = 9.81

# Missile constant characteristics.
BURN_TIME = 2.95 # Seconds.
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

class Target:

	def __init__(self, initialTargetRightUpPosition, initialTargetRightUpVelocity):
		self.targetTimeOfFlight = 0.0
		self.targetRightUpPosition = initialTargetRightUpPosition
		self.targetRightUpVelocity = initialTargetRightUpVelocity

		print("TARGET CONSTRUCTED")

	def update(self):
		deltaPos = TIME_STEP * self.targetRightUpVelocity
		self.targetRightUpPosition += deltaPos
		self.targetTimeOfFlight += TIME_STEP

class MassPropertiesAndRocketMotor:

	def __init__(self):
		self.ISP = 250 # Seconds
		self.initialTotalMass = 45 # Kilograms.
		self.finalTotalMass = 20 # Kilograms.
		self.currentMass = self.initialTotalMass # Kilograms.
		self.deltaV = np.log(self.initialTotalMass / self.finalTotalMass) * self.ISP * STANDARD_GRAVITY
		self.massFlowRate = (self.initialTotalMass - self.finalTotalMass) / BURN_TIME
		self.acceleration = 0.0
		self.force = 0.0
		self.flag = 0
		self.transverseMomentOfInertia = (self.currentMass * (3 * ((0.5 * REFERENCE_DIAMETER) ** 2) + REFERENCE_LENGTH ** 2)) / (12)

	def update(self, missileTimeOfFlight):
		fuelUsed = self.massFlowRate * missileTimeOfFlight
		self.currentMass = (self.initialTotalMass - fuelUsed)
		if self.currentMass > self.finalTotalMass:
			self.acceleration = self.massFlowRate * self.deltaV / self.currentMass
			self.force = self.acceleration * self.currentMass
			self.transverseMomentOfInertia = (self.currentMass * (3 * ((0.5 * REFERENCE_DIAMETER) ** 2) + REFERENCE_LENGTH ** 2)) / (12)
			# print(missileTimeOfFlight, self.currentMass, self.acceleration, self.force)
		else:
			self.acceleration = 0
			self.force = 0
			if self.flag == 0:
				self.transverseMomentOfInertia = (self.currentMass * (3 * ((0.5 * REFERENCE_DIAMETER) ** 2) + REFERENCE_LENGTH ** 2)) / (12)
				print(missileTimeOfFlight, "BURNOUT", self.transverseMomentOfInertia)
				self.flag = 1

class Atmosphere:

	def __init__(self):
		self.rho = None
		self.p = None
		self.a = None
		self.gravity = None
		self.q = None
		self.mach = None

		print("ATMOSPHERE CONSTRUCTED")

	def update(self, altitudeMeters, speedMetersPerSecond, unitFlag):

		atmos = atm(altitudeMeters) # input meters
		rho = atmos.density[0] # kg / m^3
		p = atmos.pressure[0] # pascals
		a = atmos.speed_of_sound[0] # m/s
		gravity = atmos.grav_accel[0] # m/s^2
		q = 0.5 * rho * speedMetersPerSecond * speedMetersPerSecond # pascals
		mach = speedMetersPerSecond / a

		if unitFlag == "IMPERIAL":
			self.rho = rho * 0.00194032 # kg/m^3 to slugs/ft^3
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

class Aerodynamics:

	def __init__(self):
		self.CD = 0.82
		self.force = np.zeros(2) # aerodynamic force
		self.moment = np.zeros(2) # aerodynamic moment

	def update(self, dynamicPressure):
		self.force[0] = self.CD * REFERENCE_AREA * dynamicPressure
		self.force[1] = 0.0
		self.moment[0] = 0.0
		self.moment[1] = 0.0 

class MissileDynamics:

	def __init__(self, initialMissileRightUpPosition, initialMissileRightUpVelocity):
		self.missileTimeOfFlight = 0.0
		self.missileRightUpPosition = initialMissileRightUpPosition
		self.missileRightUpVelocity = initialMissileRightUpVelocity

		self.rocketAcceleration = 0.0
		self.normalAcceleration = 0.0
		self.aerodynamicForce = np.zeros(2)
		self.aerodynamicMoment = np.zeros(2)
		self.localAcceleration = np.zeros(2)

		print("CONSTRUCT MISSILE")

	def calculateDerivatives(self):
		velU = unitvector(self.missileRightUpVelocity)
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

		# axialAcceleration = 

		bodyAcceleration = npa([self.rocketAcceleration, self.normalAcceleration])
		self.localAcceleration = bodyAcceleration @ mslLocalOrient

	def motion(self):
		deltaVel = TIME_STEP * self.localAcceleration
		self.missileRightUpVelocity += deltaVel
		deltaPos = TIME_STEP * self.missileRightUpVelocity
		self.missileRightUpPosition += deltaPos
		self.missileTimeOfFlight += TIME_STEP

	def update(self, rocketAcceleration, normalAcceleration):
		self.rocketAcceleration = rocketAcceleration
		self.normalAcceleration = normalAcceleration
		self.calculateDerivatives()
		self.motion()

class Guidance:

	def __init__(self):
		self.PROPORTIONAL_GAIN = 4.0
		self.command = 0.0
		self.limit = 100.0
		print("GUIDANCE CONSTRUCTED")

	def update(self, missileRightUpPosition, missileRightUpVelocity, targetRightUpPosition, targetRightUpVelocity):
		relPos = targetRightUpPosition - missileRightUpPosition
		relVel = targetRightUpVelocity - missileRightUpVelocity
		velU = unitvector(missileRightUpVelocity)
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
		rightUpInterceptorToInterceptVel = mslLocalOrient @ relVel
		rightUpInterceptorToInterceptVelMag = la.norm(rightUpInterceptorToInterceptVel)
		T1 = np.cross(rightUpInterceptorToIntercept, rightUpInterceptorToInterceptVel)
		T2 = np.dot(rightUpInterceptorToIntercept, rightUpInterceptorToIntercept)
		omega = T1 / T2
		bodyAccelCommand = 4 * omega * rightUpInterceptorToInterceptVelMag
		accCommMag = np.abs(bodyAccelCommand)
		if accCommMag > self.limit:
			new = self.limit * np.sign(bodyAccelCommand)
			bodyAccelCommand = new
		self.command = bodyAccelCommand

class Simulation:

	def __init__(self, initialTargetRightUpPosition, initialTargetRightUpVelocity, initialMissileRightUpPosition, initialMissileRightUpVelocity):

		# Simulation.
		self.SimulationTime = 0.0
		self.SimulationTimeStep = 1.0 / 100.0
		self.Data = {"X":[], "Y":[], "VX":[], "VY":[], "TX":[], "TY":[]}

		# Target.
		self.Target = Target(initialTargetRightUpPosition=initialTargetRightUpPosition, initialTargetRightUpVelocity=initialTargetRightUpVelocity)

		# Missile.
		self.RocketMotor = MassPropertiesAndRocketMotor()
		self.Atmoshpere = Atmosphere()
		self.MassProperties = None
		self.Missile = MissileDynamics(initialMissileRightUpPosition=initialMissileRightUpPosition, initialMissileRightUpVelocity=initialMissileRightUpVelocity)

		# Guidance and Control.
		self.Guidance = Guidance()
		

	def update(self):

		# Update sim time.
		self.SimulationTime += self.SimulationTimeStep

		# Update target.
		while self.Target.targetTimeOfFlight < self.SimulationTime:
			self.Target.update()

		# Update atmosphere, guidance, missile.
		while self.Missile.missileTimeOfFlight < self.SimulationTime:
			self.RocketMotor.update(self.Missile.missileTimeOfFlight)
			self.Atmoshpere.update(self.Missile.missileRightUpPosition[1], la.norm(self.Missile.missileRightUpVelocity), "METRIC")
			self.Missile.update(self.RocketMotor.acceleration, self.Guidance.command)
			self.Guidance.update(self.Missile.missileRightUpPosition, self.Missile.missileRightUpVelocity, self.Target.targetRightUpPosition, self.Target.targetRightUpVelocity)

		# Log data.
		self.Data["X"].append(self.Missile.missileRightUpPosition[0])
		self.Data["Y"].append(self.Missile.missileRightUpPosition[1])
		self.Data["VX"].append(self.Missile.missileRightUpVelocity[0])
		self.Data["VY"].append(self.Missile.missileRightUpVelocity[1])
		self.Data["TX"].append(self.Target.targetRightUpPosition[0])
		self.Data["TY"].append(self.Target.targetRightUpPosition[1])

	def plot(self):
		fig = plt.figure()
		trajectory = fig.add_subplot(111)
		trajectory.set_xlabel("Down Range (ft).")
		trajectory.set_ylabel("Altitude (ft).")
		xMin = min(self.Data["X"] + self.Data["TX"])
		xMax = max(self.Data["X"] + self.Data["TX"])
		yMin = min(self.Data["Y"] + self.Data["TY"])
		yMax = max(self.Data["Y"] + self.Data["TY"])
		trajectory.set_xlim([xMin, xMax])
		trajectory.set_ylim([yMin, yMax])
		trajectory.plot(self.Data["X"], self.Data["Y"], label="Interceptor", color="b")
		trajectory.plot(self.Data["TX"], self.Data["TY"], label="Threat", color="r")
		trajectory.legend()
		plt.show()

	def simulate(self):

		while self.SimulationTime < MAX_TIME:

			# Update simulation.
			self.update()

			# Intercept check.
			missDistance = la.norm(self.Target.targetRightUpPosition - self.Missile.missileRightUpPosition)
			if missDistance < 10:
				print(f"SUCCESSFUL INTERCEPT, MISS DISTANCE = {missDistance}")
				break

			# Poca check.
			velU = unitvector(self.Missile.missileRightUpVelocity)
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
			relPos = self.Target.targetRightUpPosition - self.Missile.missileRightUpPosition
			bodyRelPos = mslLocalOrient @ relPos
			if bodyRelPos[0] < 0:
				break

			# Console report.
			if (round(self.Missile.missileTimeOfFlight, 2).is_integer()):
				print(f"{self.Missile.missileTimeOfFlight:.2f} {self.Missile.missileRightUpPosition}")

		self.plot()

if __name__ == "__main__":
	initialTargetRightUpPosition = npa([6000.0, 6000.0])
	initialTargetRightUpVelocity = npa([-50.0, -50.0])
	initialMissileRightUpPosition = npa([0.0, 0.0])
	initialMissileRightUpVelocity = npa([10.0, 18.0])
	x = Simulation(initialTargetRightUpPosition, initialTargetRightUpVelocity, initialMissileRightUpPosition, initialMissileRightUpVelocity)
	x.simulate()
