import numpy as np
import math
import matplotlib.pyplot as plt
from numpy import array as npa
from unitVector import unitvector
from numpy import linalg as la
from ambiance import Atmosphere as atm
np.set_printoptions(suppress=True, precision=2)

# Simulation constants.
TIME_STEP = 1.0 / 100.0
HALF_TIME_STEP = TIME_STEP / 2.0
MAX_TIME = 75.0
INTEGRATION_PASS = 0 # RK2 Integration. Controlled by MissileDynamics.

# Math constants.
STANDARD_GRAVITY = 9.81 # Meters per second squared.
STANDARD_PRESSURE = 101325 # Pascals.

# Missile constant characteristics. Rough approximation of a Hellfire missile.
BURN_TIME = 2.95 # Seconds. Hellfire.
NOZZLE_EXIT_AREA = 0.004 # Meters squared. Hellfire.
REFERENCE_DIAMETER = 0.1778 # Meters. Hellfire.
REFERENCE_LENGTH = 1.6 # Meters. Hellfire.

NOSE_LENGTH = 3 # FEET
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
REFERENCE_AREA = np.pi * (REFERENCE_DIAMETER ** 2) / 4 # Meters squared. Hellfire.

WING_AREA = 0.5 * WING_HALF_SPAN * (WING_TIP_CHORD + WING_ROOT_CHORD)
TAIL_AREA = 0.5 * TAIL_HALF_SPAN * (TAIL_TIP_CHORD + TAIL_ROOT_CHORD)
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

class MassPropertiesAndRocketMotor:

	def __init__(self):
		self.ISP = 250 # Seconds
		self.initialTotalMass = 45 # Kilograms.
		self.finalTotalMass = 20 # Kilograms.
		self.currentMass = self.initialTotalMass # Kilograms.
		self.exitVelocity = self.ISP * STANDARD_GRAVITY # Meters per second.
		self.deltaV = np.log(self.initialTotalMass / self.finalTotalMass) * self.exitVelocity # Meters per second.
		self.massFlowRate = (self.initialTotalMass - self.finalTotalMass) / BURN_TIME # Kilograms per second.
		self.thrust = 0.0 # Newtons.
		self.flag = 0
		self.transverseMomentOfInertia = (self.currentMass * (3 * ((0.5 * REFERENCE_DIAMETER) ** 2) + REFERENCE_LENGTH ** 2)) / (12) # Kilograms * meters squared.

	def update(self, missileTimeOfFlight):
		fuelUsed = self.massFlowRate * missileTimeOfFlight # Kilograms.
		self.currentMass = (self.initialTotalMass - fuelUsed) # Kilograms.
		if self.currentMass > self.finalTotalMass:
			self.thrust = self.ISP * self.massFlowRate * (missileTimeOfFlight) # Newtons.
			self.transverseMomentOfInertia = (self.currentMass * (3 * ((0.5 * REFERENCE_DIAMETER) ** 2) + REFERENCE_LENGTH ** 2)) / (12) # Kilograms * meters squared.
		else:
			if self.flag == 0:
				self.thrust = 0.0 # Newtons.
				self.transverseMomentOfInertia = (self.currentMass * (3 * ((0.5 * REFERENCE_DIAMETER) ** 2) + REFERENCE_LENGTH ** 2)) / (12) # Kilograms * meters squared.
				print(missileTimeOfFlight, "BURNOUT", self.thrust, self.transverseMomentOfInertia)
				self.flag = 1

class Aerodynamics:

	def __init__(self):
		self.CD = 0.0 # Drag coefficient of a long cylinder.
		self.force = np.zeros(2) # aerodynamic force
		self.moment = np.zeros(2) # aerodynamic moment

	def update(self, dynamicPressure):
		self.force[0] = -self.CD * REFERENCE_AREA * dynamicPressure
		self.force[1] = 0.0
		self.moment[0] = 0.0
		self.moment[1] = 0.0 

class MissileDynamics:

	def __init__(self, initialMissileRightUpPosition, initialMissileRightUpVelocity):

		self.missileTimeOfFlight = 0.0
		self.missileRightUpPosition = initialMissileRightUpPosition
		self.missileRightUpVelocity = initialMissileRightUpVelocity
		self.flightPathAngleRad = 0.0
		self.flightPathAngleDeg = 0.0

		# Mass of the missile.
		self.mass = 0.0 # Kilograms.

		# Acceleration due to gravity.
		self.gravity = 0.0 # Meters per second squared.

		# Axial acceleration.
		self.rocketForce = 0.0 # Newtons.

		# Normal acceleration.
		self.normalAcceleration = 0.0
		self.mslLocalOrient = np.zeros((2, 2))
		self.bodyAcceleration = np.zeros(2)
		self.localAcceleration = np.zeros(2)

		# Add rk2 integration.
		# RK2 Integraiton.
		self.P0 = np.zeros(3)
		self.V0 = np.zeros(3)

		print("CONSTRUCTED MISSILE")

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
		self.mslLocalOrient = npa([axis, normal])

		axialAcceleration = None
		rocketAcceleration = self.rocketForce / self.mass
		axialAcceleration = rocketAcceleration

		self.flightPathAngleRad = math.atan2(self.missileRightUpVelocity[1], self.missileRightUpVelocity[0])
		self.flightPathAngleDeg = np.degrees(self.flightPathAngleRad)

		self.bodyAcceleration = npa([axialAcceleration, self.normalAcceleration])
		self.localAcceleration = (self.bodyAcceleration @ self.mslLocalOrient)
		self.localAcceleration += npa([0.0, -1.0 * self.gravity])

	def motion(self):

		global INTEGRATION_PASS

		if INTEGRATION_PASS == 0:

			INTEGRATION_PASS += 1

			self.P0 = self.missileRightUpPosition
			self.V0 = self.missileRightUpVelocity

			deltaPos = HALF_TIME_STEP * self.missileRightUpVelocity
			self.missileRightUpPosition += deltaPos
			deltaVel = HALF_TIME_STEP * self.localAcceleration
			self.missileRightUpVelocity += deltaVel
			self.missileTimeOfFlight += HALF_TIME_STEP

		elif INTEGRATION_PASS == 1:

			INTEGRATION_PASS = 0

			deltaPos = TIME_STEP * self.missileRightUpVelocity
			self.missileRightUpPosition = self.P0 + deltaPos
			deltaVel = TIME_STEP * self.localAcceleration
			self.missileRightUpVelocity =  self.V0 + deltaVel
			self.missileTimeOfFlight += HALF_TIME_STEP

			self.P0 = np.zeros(3)
			self.V0 = np.zeros(3)

	def update(self, gravity, mass, rocketForce, normalAcceleration):
		self.gravity = gravity
		self.mass = mass
		self.rocketForce = rocketForce
		self.normalAcceleration = normalAcceleration
		self.calculateDerivatives()
		self.motion()

class Guidance:

	def __init__(self):
		self.PROPORTIONAL_GAIN = 4.0
		self.command = 0.0
		self.limit = 200.0
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

	def __init__(self, ballistic, initialTargetRightUpPosition, initialTargetRightUpVelocity, initialMissileRightUpPosition, initialMissileRightUpVelocity):

		# Simulation.
		self.SimulationTime = 0.0
		self.SimulationTimeStep = 1.0 / 100.0
		self.Data = {"TOF":[], "X":[], "Y":[], "VX":[], "VY":[], "SPEED":[], "AX":[], "AY":[], "TX":[], "TY":[], "PITCH_ANGLE": []}

		# Target.
		self.Target = Target(initialTargetRightUpPosition=initialTargetRightUpPosition, initialTargetRightUpVelocity=initialTargetRightUpVelocity)

		# Missile Dynamics..
		self.ballistic = ballistic
		self.Atmoshpere = Atmosphere()
		self.MassPropertiesAndRocketMotor = MassPropertiesAndRocketMotor()
		self.Aerodynamics = Aerodynamics()
		self.MissileDynamics = MissileDynamics(initialMissileRightUpPosition=initialMissileRightUpPosition, initialMissileRightUpVelocity=initialMissileRightUpVelocity)

		# Guidance and Control.
		self.Guidance = Guidance()
		

	def update(self):

		# Update sim time.
		self.SimulationTime += self.SimulationTimeStep

		# Update target.
		while self.Target.targetTimeOfFlight < self.SimulationTime:
			self.Target.update()

		# Update atmosphere, guidance, missile.
		while self.MissileDynamics.missileTimeOfFlight < self.SimulationTime:
			self.Atmoshpere.update(self.MissileDynamics.missileRightUpPosition[1], la.norm(self.MissileDynamics.missileRightUpVelocity), "METRIC")
			self.MassPropertiesAndRocketMotor.update(self.MissileDynamics.missileTimeOfFlight)
			self.Aerodynamics.update(self.Atmoshpere.q)
			self.MissileDynamics.update(self.Atmoshpere.gravity, self.MassPropertiesAndRocketMotor.currentMass, self.MassPropertiesAndRocketMotor.thrust, self.Guidance.command)
			if not self.ballistic:
				self.Guidance.update(self.MissileDynamics.missileRightUpPosition, self.MissileDynamics.missileRightUpVelocity, self.Target.targetRightUpPosition, self.Target.targetRightUpVelocity)

		# Log data.
		self.Data["TOF"].append(self.MissileDynamics.missileTimeOfFlight)
		self.Data["X"].append(self.MissileDynamics.missileRightUpPosition[0])
		self.Data["Y"].append(self.MissileDynamics.missileRightUpPosition[1])
		self.Data["VX"].append(self.MissileDynamics.missileRightUpVelocity[0])
		self.Data["VY"].append(self.MissileDynamics.missileRightUpVelocity[1])
		self.Data["AX"].append(self.MissileDynamics.localAcceleration[0])
		self.Data["AY"].append(self.MissileDynamics.localAcceleration[1])
		speed = np.sqrt(self.MissileDynamics.missileRightUpVelocity[0] * self.MissileDynamics.missileRightUpVelocity[0] + self.MissileDynamics.missileRightUpVelocity[1] * self.MissileDynamics.missileRightUpVelocity[1])
		self.Data["SPEED"].append(speed)
		self.Data["TX"].append(self.Target.targetRightUpPosition[0])
		self.Data["TY"].append(self.Target.targetRightUpPosition[1])
		self.Data["PITCH_ANGLE"].append(self.MissileDynamics.flightPathAngleDeg)

	def plot(self):

		fig = plt.figure()

		trajectory = fig.add_subplot(121)
		trajectory.set_xlabel("Down Range (meters).")
		trajectory.set_ylabel("Altitude (meters).")
		trajectory.plot(self.Data["X"], self.Data["Y"], label="Interceptor", color="k")
		if not self.ballistic:
			trajectory.plot(self.Data["TX"], self.Data["TY"], label="Threat", color="k", linestyle = "dotted")
		trajectory.legend()

		performance = fig.add_subplot(122)
		performance.set_xlabel("Time Of Flight")
		performance.plot(self.Data["TOF"], self.Data["VX"], label="Right Velocity (m/s)", color="g")
		performance.plot(self.Data["TOF"], self.Data["VY"], label="Up Velocity (m/s)", color="r")
		performance.plot(self.Data["TOF"], self.Data["AX"], label="Right Accel (m/s^2)", color="b")
		performance.plot(self.Data["TOF"], self.Data["AY"], label="Up Accel (m/s^2)", color="cyan")
		performance.plot(self.Data["TOF"], self.Data["SPEED"], label="Speed (m/s)", color="k")
		performance.plot(self.Data["TOF"], self.Data["PITCH_ANGLE"], label="Pitch (deg)", color="pink")
		performance.legend()

		plt.show()

	def simulate(self):

		go = True
		while go:

			# Update simulation.
			self.update()

			# Performance and termination check.
			if not self.ballistic:

				# Intercept check.
				missDistance = la.norm(self.Target.targetRightUpPosition - self.MissileDynamics.missileRightUpPosition)
				if missDistance < 10:
					print(f"SUCCESSFUL INTERCEPT, MISS DISTANCE = {missDistance}")
					go = False

				# Poca check.
				velU = unitvector(self.MissileDynamics.missileRightUpVelocity)
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
				relPos = self.Target.targetRightUpPosition - self.MissileDynamics.missileRightUpPosition
				bodyRelPos = mslLocalOrient @ relPos
				if bodyRelPos[0] < 0:
					print(f"POINT OF CLOSEST APPROACH PASSED, MISS DISTANCE = {missDistance}")
					go = False

			# Ground check.
			if self.MissileDynamics.missileRightUpPosition[1] < 0:
				print(f"GROUND COLLISION")
				go = False
			# Nan check.
			if np.isnan(np.sum(self.MissileDynamics.missileRightUpPosition)):
				print(f"NOT A NUMBER")
				go = False
			# Max time check.
			if self.SimulationTime > MAX_TIME:
				print(f"MAX TIME")
				go = False

			# Console report.
			if (round(self.MissileDynamics.missileTimeOfFlight, 2).is_integer()):
				print(f"{self.MissileDynamics.missileTimeOfFlight:.2f} {self.MissileDynamics.missileRightUpPosition}")

		self.plot()

if __name__ == "__main__":
	ballistic = True
	initialTargetRightUpPosition = npa([6000.0, 6000.0])
	initialTargetRightUpVelocity = npa([-50.0, -50.0])
	initialMissileRightUpPosition = npa([0.0, 0.0])
	initialMissileRightUpVelocity = npa([10.0, 10.0])
	x = Simulation(ballistic, initialTargetRightUpPosition, initialTargetRightUpVelocity, initialMissileRightUpPosition, initialMissileRightUpVelocity)
	x.simulate()
