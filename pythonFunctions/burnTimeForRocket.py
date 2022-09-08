import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

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

def chargerBlueRocket():

	"""

	GIVENS ALLOCATED IN MEMORY BELOW.
	FIND:
		A) BURNOUT VELOCITY
		B) BURNOUT ALTITUDE
		C) MAXIMUM ALTITUDE
		D) PLOT ACCELERATION AS A FUNCTION OF TIME.
		E) PLOT VELOCITY AS A FUNCTION OF TIME.
		F) PLOT HEIGHT AS A FUNCTION OF TIME.

	THIS IS THE WAY:

		1 = (lbf * s^2) / (32.2 * lbm * ft)

	"""

	print("SP03A")

	# CONSTANTS.
	GRAVITY = 32.2 # FEET PER SECOND SQUARED.
	DT = 1 # SECONDS.
	MAX_TIME = 400.0 # SECONDS.

	# ROCKET PARAMETERS.
	ISP = 242 # SECONDS. SPECIFIC IMPULSE.
	M0 = 79.4 # LBM. INITIAL ROCKET MASS.
	MP = 30.3 # LBM. PROPELLANT MASS.
	MF = M0 - MP # LBM. FINAL ROCKET MASS.
	TB = 4.33 # SECONDS. BURN TIME.

	# STATE.
	TOF = 0.0 # SECONDS. TIME OF FLIGHT.
	H0 = 0.0 # POSITION. FEET.
	V0 = 0.0 # VELOCITY. FEET PER SECOND.
	MASS = M0 # LBM. CURRENT ROCKET MASS.

	# DERIVATIVE.
	A0 = 0.0 # ACCELERATION. FEET PER SECOND SQUARED.

	# MOTOR MODEL.
	MDOT = (M0 - MF) / TB # LBM PER SECOND.
	CONSTANT_THRUST = (ISP * GRAVITY * MDOT) / GRAVITY # LBF.

	# DATA.
	DATA = {
		"TOF": [],
		"POS": [],
		"VEL": [],
		"ACC": []
	}

	# LOOP.
	LAST_TIME = 0.0
	GO = True
	while GO:

		# DERIVATIVE.
		if TOF < TB:
			A0 = ((CONSTANT_THRUST / MASS) * GRAVITY) - GRAVITY
		else:
			A0 = 0.0 - GRAVITY

		# INTEGRATE STATE USING EULER'S METHOD.
		TOF += DT
		H0 += V0 * DT
		V0 += A0 * DT

		# ADJUST MASS.
		FUEL_USED = MDOT * TOF
		MASS = (M0 - FUEL_USED)

		# STORE DATA.
		DATA["TOF"].append(TOF)
		DATA["POS"].append(H0)
		DATA["VEL"].append(V0)
		DATA["ACC"].append(A0)

		# CONSOLE REPORT.
		if round(TOF, 3).is_integer() and TOF != LAST_TIME:
			print(f"TOF {TOF:.2f}; HEIGHT {H0:.2f}; SPEED {V0:.2f}")
			LAST_TIME = TOF

		# END CHECK.
		if V0 < 0.0:
			print("FALLING")
			GO = False
		if H0 < 0.0:
			print("GROUND COLLISION.")
			GO = False
		if np.isnan(H0):
			print("NOT A NUMBER.")
			GO = False
		if TOF > MAX_TIME:
			print("MAX TIME EXCEEDED")
			GO = False

	# Figure.
	fig = plt.figure()
	
	# Data frame.
	df = pd.DataFrame(DATA)

	# Subplots.
	height = fig.add_subplot(131)
	speed = fig.add_subplot(132)
	acc = fig.add_subplot(133)

	# Plot.
	height.plot(df.iloc[:, 0], df.iloc[:, 1], color="red", label="ALTITUDE, FEET.")
	ALT_BURNOUT = linearInterpolation(TB, df.iloc[:, 0].to_list(), df.iloc[:, 1].to_list())
	height.scatter(TB, ALT_BURNOUT, color="red", marker="2", label="BURNOUT.", s=100)
	height.legend(fontsize="small")

	speed.plot(df.iloc[:, 0], df.iloc[:, 2], color="green", label="SPEED, FEET PER SECOND.")
	VEL_BURNOUT = linearInterpolation(TB, df.iloc[:, 0].to_list(), df.iloc[:, 2].to_list())
	speed.scatter(TB, VEL_BURNOUT, color="green", marker="2", label="BURNOUT.", s=100)
	speed.legend(fontsize="small")

	acc.plot(df.iloc[:, 0], df.iloc[:, 3], color="blue", label="ACC, FEET PER SECOND SQUARED.")
	ACC_BURNOUT = linearInterpolation(TB, df.iloc[:, 0].to_list(), df.iloc[:, 3].to_list())
	acc.scatter(TB, ACC_BURNOUT, color="blue", marker="2", label="BURNOUT.", s=100)
	acc.legend(fontsize="small")

	# Report.
	print(f"ALTITUDE AT BURNOUT IS {ALT_BURNOUT:.2f} FEET.")
	print(f"VELOCITY AT BURNOUT IS {VEL_BURNOUT:.2f} FEET PER SECOND.")
	print(f"MAXIMUM ALTITUDE IS {df.iloc[-1, 1]:.2f} FEET.")

	# Show.
	plt.show()



if __name__ == "__main__":

	# Textbook 3.4.
	print("\n")
	x = burnTimeForRocket(
		finalAltitude=20000,
		gravity=9.81,
		ISP=150,
		M0=100,
		MF=50
	)
	print(f"TEXTBOOK 3.4 : BURN TIME IS {x} SECONDS.\n\n")

	# Special problem SP03A.
	chargerBlueRocket()
