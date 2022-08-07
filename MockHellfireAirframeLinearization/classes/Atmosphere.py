from ambiance import Atmosphere as atm

class Atmosphere:

	def __init__(self):

		atmosphere = atm(0.0)
		self.rho = atmosphere.density[0] # Kilograms per meter cubed.
		self.p = atmosphere.pressure[0] # Pascals.
		self.a = atmosphere.speed_of_sound[0] # Meters per second.
		self.gravity = atmosphere.grav_accel[0] # Meters per second squared.
		self.q = self.rho * 0.5 * 0.0 * 0.0 # Pascals.
		self.mach = 0.0 / self.a # Non dimensional.

		print("ATMOSPHERE LOADED")

	def update(self, altitudeMeters, speedMperSec):

		atmosphere = atm(altitudeMeters)
		self.rho = atmosphere.density[0] # Kilograms per meter cubed.
		self.p = atmosphere.pressure[0] # Pascals.
		self.a = atmosphere.speed_of_sound[0] # Meters per second.
		self.gravity = atmosphere.grav_accel[0] # Meters per second squared.
		self.q = self.rho * 0.5 * speedMperSec * speedMperSec # Pascals.
		self.mach = speedMperSec / self.a # Non dimensional.
