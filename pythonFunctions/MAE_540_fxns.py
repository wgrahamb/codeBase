import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from numpy import array as npa
from numpy import linalg as la

# Constants.
NauticalMilesToFeet = 6076 # Feet.
MilesToFeet = 5280 # Feet.
R_E = 3963 # Miles.
Mu_E = 1.4076e16 # Feet cubed per second squared.
SecondsInADay = 86400 # Seconds.

def linearInterpolation(x, xx, yy):
          lowIndex = None
          highIndex = None
          index = -1
          for number in xx:
                    index += 1
                    if number > x:
                              if highIndex == None:
                                        highIndex = index
                              elif number < xx[highIndex]:
                                        highIndex = index
                    if number < x:
                              if lowIndex == None:
                                        lowIndex = index
                              elif number > xx[lowIndex]:
                                        lowIndex = index
          if lowIndex == None:
                    lowIndex = 0
          if highIndex == None:
                    highIndex = -1
          # print(x, xx[lowIndex], xx[highIndex], yy[lowIndex], yy[highIndex])
          y = yy[lowIndex] + ((x - xx[lowIndex]) * ((yy[highIndex] - yy[lowIndex]) / (xx[highIndex] - xx[lowIndex])))
          return y

# Rocket Propulsion Textbook 2.11
def Textbook_2_11(ALTITUDE, ORBIT_LATITUDE):

	US_LAT = np.radians(28.5) # Radians
	FR_LAT = np.radians(3) # Radians

	Vc = np.sqrt( Mu_E / ( (R_E * MilesToFeet) + (ALTITUDE * NauticalMilesToFeet) ) )
	print("CIRCULAR VELOCITY", Vc)

	FR_Vpc = 2 * Vc * np.sin((np.abs(FR_LAT - ORBIT_LATITUDE)) / 2)
	print("FRANCE PLANE CHANGE VELOCITY", FR_Vpc)

	FR_Vsurf = (2 * np.pi * (R_E * MilesToFeet) * np.cos(FR_LAT)) / SecondsInADay
	print("FRANCE SURFACE VELOCITY", FR_Vsurf)

	FR_DELTA_V = Vc - FR_Vsurf + FR_Vpc
	print("FRANCE DELTA VELOCITY", FR_DELTA_V)

	US_Vpc = 2 * Vc * np.sin((np.abs(US_LAT - ORBIT_LATITUDE)) / 2)
	print("US PLANE CHANGE VELOCITY", US_Vpc)

	US_Vsurf = (2 * np.pi * (R_E * MilesToFeet) * np.cos(US_LAT)) / SecondsInADay
	print("US SURFACE VELOCITY", US_Vsurf)

	US_DELTA_V = Vc - US_Vsurf + US_Vpc
	print("US DELTA VELOCITY", US_DELTA_V)

	print("\n")

# Rocket Propulsion Textbook 3.3
def Textbook_3_3(
	iPropMassFraction,
	iPayloadMass,
	iDeltaVIdeal,
	iSpecificImpulse
):

	propMassFraction = iPropMassFraction
	payloadMass = iPayloadMass # lbm
	deltaV = iDeltaVIdeal # feet per sec
	ISP = iSpecificImpulse # seconds
	grav = 32.2 # feet per second squared

	MR = np.exp(deltaV / (ISP * grav))
	propMass = payloadMass * (MR - 1) / (MR - ((MR - 1) / propMassFraction))
	inertMass = (propMass - propMass * propMassFraction) / propMassFraction

	print(f"PROP MASS {propMass}; INERT MASS {inertMass}")

	return propMass, inertMass

# Rocket Propulsion Textbook 3.4
def burnTimeForRocket(
	finalAltitude, # Meters.
	gravity, # Meters per second squared.
	ISP, # Seconds.
	M0, # Kilograms.
	MF # Kilograms.
):
	
	TEMP1 = finalAltitude - (gravity / 2.0) * (ISP ** 2) * ((np.log(M0 / MF)) ** 2)
	TEMP2 = gravity * ISP
	TEMP3 = TEMP2 * ((MF / (M0 - MF)) * np.log(MF / M0) + 1)
	TEMP4 = TEMP2 * np.log(M0 / MF)
	TEMP5 = TEMP1 / (TEMP3 - TEMP4)
	return TEMP5

# HW6
def machAtExit(exitPressure, chamberPressure, gamma):
	check = chamberPressure / exitPressure
	tolerance = 0.001
	machs = np.linspace(0.0, 100.0, 100000)
	ret = None
	for index, mach in enumerate(machs):
		guess = (1 + ((gamma - 1) / 2) * mach * mach) ** (gamma / (gamma - 1))
		if np.abs(check / guess) < (1 - tolerance):
			ret = mach
			break
	return ret

def nozzleAreaRatio(mach, gamma):
	t1 = (1 / mach)
	t2 = 2 + (gamma - 1) * mach * mach
	t3 = gamma + 1
	t = t1 * (t2 / t3) ** ((gamma + 1) / (2.0 * (gamma - 1)))
	return t

def isentropicPressureRatio(gamma, mach):
	t1 = 1
	t2 = (gamma - 1) / 2.0
	t3 = mach * mach
	t4 = gamma / (gamma - 1)
	ret = (t1 + t2 * t3) ** t4
	return ret

def forceCoefficient(gamma, sigma, mach):
	t1 = 2 * gamma * gamma / (gamma - 1)
	t2 = 2 / (gamma + 1)
	t3 = (gamma + 1) / (gamma - 1)
	t4 = 1
	t5 = 1.0 / (isentropicPressureRatio(gamma, mach))
	t6 = (gamma - 1) / gamma
	t7 = (1.0 / (isentropicPressureRatio(gamma, mach))) * sigma
	ret = np.sqrt(t1 * (t2 ** t3) * (t4 - t5 ** t6)) + t7
	return ret

def findMach(gamma, sigma):
	low = 0
	high = 10
	index = 0
	while True:
		index += 1
		machGuess = (low + high) / 2.0
		x = nozzleAreaRatio(machGuess, gamma)
		check = x / sigma
		print(f"REPORT\n  INDEX {index}\n  LOW {low}\n  HIGH {high}\n  MACH {machGuess:.2f}\n  CHECK {check:.2f}\n")
		if 0.98 < check < 1.02:
			print(f"Solution found: {machGuess}\n")
			break
		elif check < 0.98:
			low = machGuess
		elif check > 1.02:
			high = machGuess
		if index == 20:
			break

# pascals, m^2, Kelvin, kg / kg-mol, ND
def mDot(Pc, At, Tc, M, gamma):
	Ru = 8317 # N * m / kg * mol * Kelvin
	t1 = Pc * At # pascals * m^2 = N
	t2 = np.sqrt( Ru * Tc / (gamma * M) )
	t3 = 2 / (gamma + 1)
	t4 = (-1.0 * (gamma + 1)) / (2.0 * (gamma - 1))
	ret = t1 / (t2 * (t3 ** t4))
	return ret

if __name__ == "__main__":

	### TEXTBOOK 2.11 ###
	print("\n")
	ORBIT_ALT_1 = 85 # Miles
	ORBIT_LAT_1 = np.radians(0.0)
	ORBIT_ALT_2 = 90 # Miles
	ORBIT_LAT_2 = np.radians(35.0)
	Textbook_2_11(ORBIT_ALT_1, ORBIT_LAT_1)
	Textbook_2_11(ORBIT_ALT_2, ORBIT_LAT_2)
	
	### TEXTBOOK 3.3 ###
	print("\n")
	# Stage 2.
	propMass, inertMass = Textbook_3_3(
		iPropMassFraction=0.8,
		iPayloadMass=500,
		iDeltaVIdeal=5000,
		iSpecificImpulse=350
	)

	# Stage 1.
	Textbook_3_3(
		iPropMassFraction=0.9,
		iPayloadMass=500 + 200 + propMass + inertMass,
		iDeltaVIdeal=5000,
		iSpecificImpulse=270
	)

	### TEXTBOOK 3.4 ###
	print("\n")
	x = burnTimeForRocket(
		finalAltitude=20000,
		gravity=9.81,
		ISP=150,
		M0=100,
		MF=50
	)
	print(f"TEXTBOOK 3.4 : BURN TIME IS {x} SECONDS.")

	### HW6 ###
	x = machAtExit(10.4, 1583.1, 1.23)
	x = nozzleAreaRatio(2.8, 1.3)
	findMach(1.2, 2.65)
	x = isentropicPressureRatio(1.2, 2.3)
	x = forceCoefficient(1.2, 2.65, 2.3)
	x = mDot(5000000, 0.1, 3000, 15, 1.2)
	x = isentropicPressureRatio(1.2, 2.8)


