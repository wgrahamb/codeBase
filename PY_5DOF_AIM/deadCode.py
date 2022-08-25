# import pandas as pd
# import numpy as np
# from scipy.interpolate import interp1d
# from scipy.interpolate import interp2d
# from writePickle import writepickle
# from loadPickle import loadpickle

# def main():
# 	cdOff = pd.read_excel(r"AERO_CD_AIM_OFF.xlsx")
# 	cdOn = pd.read_excel(r"AERO_CD_AIM_ON.xlsx")
# 	cl = pd.read_excel(r"AERO_CL.xlsx")
# 	propulsion = pd.read_excel(r"PROP.xlsx")

# 	cdOffMachs = cdOff.columns.tolist()[1:len(cdOff.columns)]
# 	cdOffAlphas = cdOff.iloc[:, 0].tolist()
# 	cdOffData = cdOff.iloc[:, 1:].to_numpy()
# 	dragCoefficientMotorOffInterpolation = interp2d(cdOffMachs, cdOffAlphas, cdOffData)

# 	cdOnMachs = cdOn.columns.tolist()[1:len(cdOn.columns)]
# 	cdOnAlphas = cdOn.iloc[:, 0].tolist()
# 	cdOnData = cdOn.iloc[:, 1:].to_numpy()
# 	dragCoefficientMotorOnInterpolation = interp2d(cdOnMachs, cdOnAlphas, cdOnData)

# 	clMachs = cl.columns.tolist()[1:len(cl.columns)]
# 	clAlphas = cl.iloc[:, 0].tolist()
# 	clData = cl.iloc[:, 1:].to_numpy()
# 	liftCoefficientInterpolation = interp2d(clMachs, clAlphas, clData)

# 	motorThrustTimes = propulsion.iloc[:, 0].tolist()
# 	motorThrusts = propulsion.iloc[:]["THRUST"].tolist()
# 	motorThrustInterpolation = interp1d(x=motorThrustTimes, y=motorThrusts, kind="nearest", fill_value=motorThrusts[-1], bounds_error=False)

# 	missileMassTimes = propulsion.iloc[:, 0].tolist()
# 	missileMasses = propulsion.iloc[:]["MASS"].tolist()
# 	missileMassInterpolation = interp1d(x=missileMassTimes, y=missileMasses, kind="nearest", fill_value=missileMasses[-1], bounds_error=False)
	
# 	pickleDict = {
# 		"CD MOTOR OFF": dragCoefficientMotorOffInterpolation,
# 		"CD MOTOR ON": dragCoefficientMotorOnInterpolation,
# 		"CL": liftCoefficientInterpolation,
# 		"THRUST (NEWTONS)": motorThrustInterpolation,
# 		"MASS (KG)": missileMassInterpolation
# 	}

# 	writepickle(pickleDict, "lookUps.pickle")



# if __name__ == "__main__":
# 	# main()
# 	savedData = loadpickle("lookUps.pickle")
# 	print(savedData["MASS (KG)"](200))

# # NORMAL AND SIDE FORCE COEFFICIENT DERIVATIVES
# absAlphaDeg = np.abs(np.degrees(self.alpha))
# absBetaDeg = np.abs(np.degrees(self.beta))
# normalForceCoefficientDerivative = None
# sideForceCoefficientDerivative = None
# if absAlphaDeg < 10:
# 	normalForceCoefficientDerivative = np.degrees(0.123 + 0.013 * absAlphaDeg)
# else:
# 	normalForceCoefficientDerivative = np.degrees(0.06 * (absAlphaDeg ** 0.625))
# if absBetaDeg < 10:
# 	sideForceCoefficientDerivative = np.degrees(0.123 + 0.013 * absBetaDeg)
# else:
# 	sideForceCoefficientDerivative = np.degrees(0.06 * (absBetaDeg ** 0.625))

# # NORMAL AND SIDE FORCE COEFFICIENT DERIVATES CHECK AND BALANCE
# if self.alpha >= 0.0: # NOSE BELOW FREE STREAM, NORMAL FORCE PUSHES INTERCEPTOR TOWARD THE GROUND
# 	normalForceCoefficientDerivative = np.negative(np.abs(normalForceCoefficientDerivative))
# elif self.alpha < 0.0: # NOSE ABOVE FREE STREAM, NORMAL FORCE PUSHES INTERCEPTOR TOWARDS SPACE
# 	normalForceCoefficientDerivative = np.positive(np.abs(normalForceCoefficientDerivative))
# if self.beta >= 0.0: # NOSE LEFT OF FREE STREAM, SIDE FORCE PUSHES INTERCEPTOR LEFT
# 	sideForceCoefficientDerivative = np.negative(np.abs(sideForceCoefficientDerivative))
# elif self.beta < 0.0: # NOSE RIGHT OF FREE STREAM, SIDE FORCE PUSHES INTERCEPTOR RIGHT
# 	sideForceCoefficientDerivative = np.positive(np.abs(sideForceCoefficientDerivative))