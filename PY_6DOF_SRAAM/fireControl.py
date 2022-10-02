import numpy as np
from numpy import linalg as la
from numpy import array as npa
np.set_printoptions(suppress=True, precision=2)
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use("WebAgg")
import pandas as pd
from sixDofSim import sixDofSim as msl
from sixDofSim import endChecks
from utility.Target import Target
import copy

# Two on one.
class CPP_SRAAM_FIRE_CONTROL:

	# Meant to represent an incoming data packet from a radar for one target.
	def __init__(
		self,
		tgtEnuPos,
		tgtEnuVel,
		launcherEnuPos,
		launcherEnuEuler
	):

		# Input.
		self.tgtEnuPos = tgtEnuPos
		self.tgtEnuVel = tgtEnuVel
		self.launcherEnuPos = launcherEnuPos
		self.launcherEnuEuler = launcherEnuEuler

		# Simulation control
		self.MAXT = 100.0
		self.T = 0.0
		self.DT = 0.01

		# Missile Launcher.
		self.launcher = {
			"enuPos": launcherEnuPos,
			"enuEuler": launcherEnuEuler,
			"msls": []
		}
		for i in range(3):
			newMsl = msl(
				id=f"msl_{i}",
				enuPos=self.launcher["enuPos"],
				enuAttitude=self.launcher["enuEuler"],
				pip=self.tgtEnuPos
			)
			self.launcher["msls"].append(newMsl)

		# Fire control
		self.targetPropagation = None

	# Called once.
	def propagateThreat(self):

		# propagate target.
		x = Target(self.tgtEnuPos, self.tgtEnuVel)
		now = 0
		max = self.MAXT
		trajectory = []
		while now < max:
			now += self.DT
			x.update(self.DT)
			dp = [copy.deepcopy(x.t_tof), copy.deepcopy(x.enuPos)]
			trajectory.append(dp)
			if x.enuPos[2] < 0.0:
				break
		self.targetPropagation = copy.deepcopy(trajectory)

	# Called once.
	def assessThreat(self):

		# Bisection search along trajectory.
		# Assume for now the target is moving in a general
		# direction toward the launch site.
		lIndex = 0
		hIndex = len(self.targetPropagation) - 1
		goodFirstShot=False
		loopCount = 0

		while not goodFirstShot:

			# update index
			index = int((lIndex + hIndex) / 2)
			
			# update loop
			loopCount += 1

			# grab target data
			tgtTofAtPip = self.targetPropagation[index][0]
			pip = self.targetPropagation[index][1]
			
			# create missile
			newMsl = msl(
				id=f"msl_predictor_{loopCount}",
				enuPos=self.launcher["enuPos"],
				enuAttitude=self.launcher["enuEuler"],
				pip=pip
			)

			# run trajectory
			while True:
				newMsl.update(0.5)
				if newMsl.lethality == endChecks.maxTimeExceeded:
					newMsl.go = True
				if newMsl.lethality != endChecks.maxTimeExceeded:
					break
			
			# assess shot
			if newMsl.lethality != endChecks.intercept:
				# scoot up. assuming out of range.
				print(f"REPORT")
				print(f"  RESULT: {newMsl.lethality.name}.")
				print(f"  LOOP_COUNT: {loopCount}.")
				print(f"  INDEX: {index}.")
				print(f"  MSL TOF: {newMsl.mslTof}.")
				print(f"  MSL POS: {newMsl.mslPosEnu}.")
				print(f"  TGT TOF: {tgtTofAtPip}.")
				print(f"  TGT POS: {pip}.")
				lIndex = index
			else:
				diff = np.abs(newMsl.mslTof - tgtTofAtPip)
				percentDiff = None
				# arrived before the target. good shot.
				if newMsl.mslTof < tgtTofAtPip:
					print(f"GOODSHOT!")
					print(f"  RESULT: {newMsl.lethality.name}.")
					print(f"  LOOP_COUNT: {loopCount}.")
					print(f"  INDEX: {index}.")
					print(f"  MSL TOF: {newMsl.mslTof}.")
					print(f"  MSL POS: {newMsl.mslPosEnu}.")
					print(f"  TGT TOF: {tgtTofAtPip}.")
					print(f"  TGT POS: {pip}.")
					goodFirstShot = True
				# arrived after the target. bad shot.
				else:
					# need to scoot up.
					lIndex = index
			
			# break if not converged
			if loopCount > 10:
				break

	def flyMissile(self, missile, flyForThisLong):
		missile.update(flyForThisLong)
		if missile.lethality == msl.maxTimeExceeded:
			missile.go = True

	def main(self):

		self.propagateThreat()
		self.assessThreat()
		lastT = None
		while self.T < self.MAXT:
			self.T += self.DT
			if round(self.T, 2).is_integer() and self.T != lastT:
				print(f"SIMTIME {self.T:.0f}")
				lastT = self.T


if __name__ == "__main__":

	x = CPP_SRAAM_FIRE_CONTROL(
		tgtEnuPos=npa([10000.0, 10000.0, 10000.0]),
		tgtEnuVel=npa([-200.0, -250.0, -150.0]),
		launcherEnuPos=npa([100.0, 100.0, 1.0]),
		launcherEnuEuler=npa([0.0, np.radians(45.0), np.radians(10.0)])
	)
	x.main()