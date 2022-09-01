
import numpy as np
from numpy import array as npa
from numpy import linalg as la

# Constants.
NauticalMilesToFeet = 6076 # Feet.
MilesToFeet = 5280 # Feet.
R_E = 3963 # Miles.
Mu_E = 1.4076e16 # Feet cubed per second squared.
SecondsInADay = 86400 # Seconds.

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

print("\n")

ORBIT_ALT_1 = 85 # Miles
ORBIT_LAT_1 = np.radians(0.0)
ORBIT_ALT_2 = 90 # Miles
ORBIT_LAT_2 = np.radians(35.0)

Textbook_2_11(ORBIT_ALT_1, ORBIT_LAT_1)
Textbook_2_11(ORBIT_ALT_2, ORBIT_LAT_2)