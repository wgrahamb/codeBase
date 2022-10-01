import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from interpolationGivenTwoVectors import linearInterpolation
from atm1976_metric import atm1976_metric
from matPlotLibColors import matPlotLibColors
import matplotlib
matplotlib.use("WebAgg")

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

	print("\n")
	print("SP03A")

	# CONSTANTS.
	STANDARD_GRAVITY = 32.2 # FEET PER SECOND SQUARED.
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
	CONSTANT_THRUST = (ISP * STANDARD_GRAVITY * MDOT) / STANDARD_GRAVITY # LBF.

	# DATA.
	DATA1 = {
		"TOF": [],
		"POS": [],
		"VEL": [],
		"ACC": []
	}

	# LOOP.
	GO = True
	while GO:

		# DERIVATIVE.
		if TOF < TB:
			A0 = ((CONSTANT_THRUST / MASS) * STANDARD_GRAVITY) - STANDARD_GRAVITY
			# ADJUST MASS.
			FUEL_USED = MDOT * TOF
			MASS = (M0 - FUEL_USED)
		else:
			A0 = 0.0 - STANDARD_GRAVITY
			# ADJUST MASS.
			MASS = MF

		# INTEGRATE STATE USING EULER'S METHOD.
		TOF += DT
		H0 += V0 * DT
		V0 += A0 * DT

		# STORE DATA.
		DATA1["TOF"].append(TOF)
		DATA1["POS"].append(H0)
		DATA1["VEL"].append(V0)
		DATA1["ACC"].append(A0)

		# # CONSOLE REPORT.
		# if round(TOF, 3).is_integer() and TOF != LAST_TIME:
		# 	print(f"TOF {TOF:.2f}; HEIGHT {H0:.2f}; SPEED {V0:.2f}")

		# END CHECK.
		if V0 < 0.0:
			print(f"TOF {TOF:.2f}; HEIGHT {H0:.2f}; SPEED {V0:.2f}")
			print("FALLING")
			GO = False
		if H0 < 0.0:
			print(f"TOF {TOF:.2f}; HEIGHT {H0:.2f}; SPEED {V0:.2f}")
			print("GROUND COLLISION.")
			GO = False
		if np.isnan(H0):
			print(f"TOF {TOF:.2f}; HEIGHT {H0:.2f}; SPEED {V0:.2f}")
			print("NOT A NUMBER.")
			GO = False
		if TOF > MAX_TIME:
			print(f"TOF {TOF:.2f}; HEIGHT {H0:.2f}; SPEED {V0:.2f}")
			print("MAX TIME EXCEEDED")
			GO = False

	"""

	GIVENS ALLOCATED IN MEMORY BELOW.
	FIND:
		A) FIX ANY PROBLEMS TO FIRST MODEL.
		B) PLOT DENSITY AS A FUNCTION OF ALTITUDE BETWEEN
		0 FEET AND 100000 FEET.
		C)
			1) SHOW EQNS FOR MASS, ACCELERATION, VELOCITY, AND
			ALTITUDE FOR THREE TIME STEPS. SHOW UNITS.
			2) PUT THE SOLUTION IN CODE.
		D) PLOT ACCELERATION FROM T_0 TO T_APOGEE OF
		FIRST MODEL VS SECOND MODEL AS A FUNCTION OF TIME.
		E) PLOT VELOCITY FROM T_0 TO T_APOGEE OF FIRST MODEL
		VS SECOND MODEL AS A FUNCTION OF TIME.
		F) PLOT HEIGHT FROM T_0 TO T_APOGEE OF FIRST MODEL
		VS SECOND MODEL AS A FUNCTION OF TIME.
		G) DATA TABLE.

	THIS IS THE WAY:

		1 = (lbf * s^2) / (32.2 * lbm * ft)

	"""

	print("\n")
	print("SP03B")

	# CONSTANTS.
	STANDARD_GRAVITY = 32.2 # FEET PER SECOND SQUARED.
	DT = None # SECONDS.
	MAX_TIME = 400.0 # SECONDS.
	PA_TO_PSF = 0.020885
	M_TO_FT = 3.28084
	KGPM3_TO_LBMPFT3 = 0.062428

	# ATMOSPHERE.
	ATMOS = atm1976_metric()
	ATMOS.update(0.0, 0.0)
	Q = ATMOS.q * PA_TO_PSF # PSF.
	G = ATMOS.g * M_TO_FT # Meters per second squared.

	# ROCKET PARAMETERS.
	ISP = 242 # SECONDS. SPECIFIC IMPULSE.
	M0 = 79.4 # LBM. INITIAL ROCKET MASS.
	MP = 30.3 # LBM. PROPELLANT MASS.
	MF = M0 - MP # LBM. FINAL ROCKET MASS.
	TB = 4.33 # SECONDS. BURN TIME.
	REFAREA = 0.2089819 # FEET SQUARED.
	CD = 0.3 # NON DIMENSIONAL.

	# STATE.
	TOF = 0.0 # SECONDS. TIME OF FLIGHT.
	H0 = 0.0 # POSITION. FEET.
	V0 = 0.0 # VELOCITY. FEET PER SECOND.
	MASS = M0 # LBM. CURRENT ROCKET MASS.

	# DERIVATIVE.
	A0 = 0.0 # ACCELERATION. FEET PER SECOND SQUARED.

	# MOTOR MODEL.
	MDOT = (M0 - MF) / TB # LBM PER SECOND.
	CONSTANT_THRUST = (ISP * STANDARD_GRAVITY * MDOT) / STANDARD_GRAVITY # LBF.

	# DATA.
	DATA2 = {
		"TOF": [],
		"POS": [],
		"VEL": [],
		"ACC": []
	}

	TOFS = np.linspace(0, MAX_TIME, int(MAX_TIME + 1))
	TOFS = TOFS.tolist()
	TOFS.append(4.33)
	TOFS.sort()

	# LOOP.
	for index, T in enumerate(TOFS):

		# ATMOSPHERE.
		ATMOS.update(H0 * (1.0 / M_TO_FT), V0 * (1.0 / M_TO_FT))
		RHO = ATMOS.rho * KGPM3_TO_LBMPFT3 # LBM PER FT^3.
		Q = ATMOS.q * PA_TO_PSF # Lbf per square foot.
		G = ATMOS.g * M_TO_FT # Feet per second squared.

		# DERIVATIVE.
		DRAG = None
		if V0 < 0:
			DRAG = 1.0 * (CD * REFAREA * Q / MASS) * STANDARD_GRAVITY
		else:
			DRAG = -1.0 * (CD * REFAREA * Q / MASS) * STANDARD_GRAVITY

		if TOF < TB:
			SPECIFIC_FORCE = ((CONSTANT_THRUST / MASS) * STANDARD_GRAVITY)
			G *= -1.0
			A0 =  SPECIFIC_FORCE + DRAG + G
		else:
			SPECIFIC_FORCE = 0.0
			G *= -1.0
			A0 =  SPECIFIC_FORCE + DRAG + G

		# INTEGRATE STATE USING EULER'S METHOD.
		DT = T - TOF
		TOF += DT
		H0 += V0 * DT
		V0 += A0 * DT

		if TOF < TB:
			FUEL_USED = MDOT * TOF
			MASS = (M0 - FUEL_USED)
		else:
			MASS = MF

		# STORE DATA.
		DATA2["TOF"].append(TOF)
		DATA2["POS"].append(H0)
		DATA2["VEL"].append(V0)
		DATA2["ACC"].append(A0)

		# # CONSOLE REPORT.
		# if round(TOF, 3).is_integer() and TOF != LAST_TIME:
		# 	print(f"TOF {TOF:.2f}; HEIGHT {H0:.2f}; SPEED {V0:.2f}")

		# END CHECK.
		if V0 < 0.0:
			print(f"TOF {TOF:.2f}; HEIGHT {H0:.2f}; SPEED {V0:.2f}")
			print("FALLING")
			break
		if H0 < 0.0:
			print(f"TOF {TOF:.2f}; HEIGHT {H0:.2f}; SPEED {V0:.2f}")
			print("GROUND COLLISION.")
			break
		if np.isnan(H0):
			print(f"TOF {TOF:.2f}; HEIGHT {H0:.2f}; SPEED {V0:.2f}")
			print("NOT A NUMBER.")
			break
		if TOF > MAX_TIME:
			print(f"TOF {TOF:.2f}; HEIGHT {H0:.2f}; SPEED {V0:.2f}")
			print("MAX TIME EXCEEDED")
			break

	# Plot density from 0.0 to 100000.0 feet.
	alts = np.linspace(0, 100000, 10000)
	rhoData = []
	for index, alt in enumerate(alts):
		ATMOS.update(alt * (1.0 / M_TO_FT), 0.0)
		RHO = ATMOS.rho * KGPM3_TO_LBMPFT3 # LBM PER FT^3.
		rhoData.append([alt, RHO])
	dfRho = pd.DataFrame(rhoData)

	# Figure.
	fig = plt.figure()
	colors = matPlotLibColors()

	# Data frame.
	df1 = pd.DataFrame(DATA1)

	# Data frame.
	df2 = pd.DataFrame(DATA2)

	# Subplots.
	density = fig.add_subplot(221)
	height = fig.add_subplot(222)
	height.set_title("ALTITUDE")
	height.set_xlabel("TOF (SECONDS)")
	speed = fig.add_subplot(223)
	speed.set_title("SPEED")
	speed.set_xlabel("TOF (SECONDS)")
	acc = fig.add_subplot(224)
	acc.set_title("ACCELERATION")
	acc.set_xlabel("TOF (SECONDS)")

	dfRocket = pd.DataFrame(
		columns=["MODEL1", "MODEL2"],
		index=["A0", "AB", "VB", "VMAX", "HB", "HMAX", "TMAX"]
	)

	# Plot first model.
	dfRocket["MODEL1"]["A0"] = df1.iloc[0, 3]
	dfRocket["MODEL1"]["AB"] = linearInterpolation(TB, df1.iloc[:, 0].to_list(), df1.iloc[:, 3].to_list())
	dfRocket["MODEL1"]["VB"] = linearInterpolation(TB, df1.iloc[:, 0].to_list(), df1.iloc[:, 2].to_list())
	dfRocket["MODEL1"]["VMAX"] = max(df1.iloc[:, 2].to_list())
	dfRocket["MODEL1"]["HB"] = linearInterpolation(TB, df1.iloc[:, 0].to_list(), df1.iloc[:, 1].to_list())
	dfRocket["MODEL1"]["HMAX"] = max(df1.iloc[:, 1].to_list())
	dfRocket["MODEL1"]["TMAX"] = max(df1.iloc[:, 0].to_list())

	height.plot(df1.iloc[:, 0], df1.iloc[:, 1], color=colors.pop(0), label="MODEL 1 ALTITUDE, FEET.")
	height.scatter(TB, dfRocket["MODEL1"]["HB"], color=colors.pop(0), marker="2", label="MODEL 1 BURNOUT.", s=100)
	speed.plot(df1.iloc[:, 0], df1.iloc[:, 2], color=colors.pop(0), label="MODEL 1 SPEED, FEET PER SECOND.")
	speed.scatter(TB, dfRocket["MODEL1"]["VB"], color=colors.pop(0), marker="2", label="MODEL 1 BURNOUT.", s=100)
	acc.plot(df1.iloc[:, 0], df1.iloc[:, 3], color=colors.pop(0), label="MODEL 1 ACC, FEET PER SECOND SQUARED.")
	acc.scatter(TB, dfRocket["MODEL1"]["AB"], color=colors.pop(0), marker="2", label="MODEL 1 BURNOUT.", s=100)

	# Plot second model.
	dfRocket["MODEL2"]["A0"] = df2.iloc[0, 3]
	dfRocket["MODEL2"]["AB"] = float(df2.loc[df2["TOF"] == 4.33]["ACC"])
	dfRocket["MODEL2"]["VB"] = linearInterpolation(TB, df2.iloc[:, 0].to_list(), df2.iloc[:, 2].to_list())
	dfRocket["MODEL2"]["VMAX"] = max(df2.iloc[:, 2].to_list())
	dfRocket["MODEL2"]["HB"] = linearInterpolation(TB, df2.iloc[:, 0].to_list(), df2.iloc[:, 1].to_list())
	dfRocket["MODEL2"]["HMAX"] = max(df2.iloc[:, 1].to_list())
	dfRocket["MODEL2"]["TMAX"] = max(df2.iloc[:, 0].to_list())

	height.plot(df2.iloc[:, 0], df2.iloc[:, 1], color=colors.pop(0), label="MODEL 2 ALTITUDE, FEET.")
	height.scatter(TB, dfRocket["MODEL2"]["HB"], color=colors.pop(0), marker="2", label="MODEL 2BURNOUT.", s=100)
	speed.plot(df2.iloc[:, 0], df2.iloc[:, 2], color=colors.pop(0), label="MODEL 2 SPEED, FEET PER SECOND.")
	speed.scatter(TB, dfRocket["MODEL2"]["VB"], color=colors.pop(0), marker="2", label="MODEL 2 BURNOUT.", s=100)
	acc.plot(df2.iloc[:, 0], df2.iloc[:, 3], color=colors.pop(0), label="MODEL 2 ACC, FEET PER SECOND SQUARED.")
	acc.scatter(TB, dfRocket["MODEL2"]["AB"], color=colors.pop(0), marker="2", label="MODEL 2 BURNOUT.", s=100)

	height.legend(fontsize="xx-small", loc="upper left")
	speed.legend(fontsize="xx-small", loc="upper right")
	acc.set_xlim([0, 10])
	acc.legend(fontsize="xx-small", loc="lower left")

	# Plot density.
	density.plot(dfRho.iloc[:, 1], dfRho.iloc[:, 0], color="blue")
	density.set_xlabel("RHO (LBM/FT^3)")
	density.set_ylabel("ALTITUDE (FEET)")

	print("\n")
	print(dfRocket)
	print("\n")

	# Show.
	fig.set_tight_layout(tight=True)
	plt.show()

if __name__ == "__main__":

	# Special problem SP03A and SP03B.
	chargerBlueRocket()