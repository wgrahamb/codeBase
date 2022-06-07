import numpy as np

def AIM9M_FIN_COMMAND_PARAMS(
	weight,
	mach,
	rho,
	speed,
	command,
	centerOfGrav,
	transverseMomentOfInertia,
	gravity):

	inchesToMeters = 1 / 39.37

	dimensions = {
		"REFDIAM": 5 * inchesToMeters,
		"NOSELENGTH": 10.73 * inchesToMeters,
		"REFLENGTH": 70.715 * inchesToMeters,
		"WINGHALFSPANA1": 5.496 * inchesToMeters,
		"WINGTIPCHORDA1": 2.536 * inchesToMeters,
		"WINGROOTCHORDA1": 8.164 * inchesToMeters,
		"WINGHALFSPANA2": (8.164 - 5.496) * inchesToMeters,
		"WINGTIPCHORDA2": 0.2 * inchesToMeters,
		"WINGROOTCHORDA2": 2.536 * inchesToMeters,
		"TAILHALFSPAN": (24.775 - 5) * inchesToMeters,
		"TAILTIPCHORD": 15.915 * inchesToMeters,
		"TAILROOTCHORD": 24 * inchesToMeters,
		"DELTAXFROMNOSEBASETOWINGLE": 0.0 * inchesToMeters,
		"CENTEROFDEFL": (70.175 - 1.891 - 5) * inchesToMeters
	} # METERS
	
	tailArea = 0.5 * dimensions["TAILHALFSPAN"] * (dimensions["TAILTIPCHORD"] + dimensions["TAILROOTCHORD"])
	wingOneArea = 0.5 * dimensions["WINGHALFSPANA1"] * (dimensions["WINGTIPCHORDA1"] + dimensions["WINGROOTCHORDA1"])
	wingTwoArea = 0.5 * dimensions["WINGHALFSPANA2"] * (dimensions["WINGTIPCHORDA2"] + dimensions["WINGROOTCHORDA2"])
	wingArea = wingOneArea + wingTwoArea
	refArea = 3.14159 * (dimensions["REFDIAM"] ** 2) / 4
	noseArea = dimensions["NOSELENGTH"] * dimensions["REFDIAM"] * 0.667
	bodyArea = (dimensions["REFLENGTH"] - dimensions["NOSELENGTH"]) * dimensions["REFDIAM"]
	planformArea = bodyArea + noseArea
	beta = (mach ** 2 - 1)
	noseCenterOfPressure = 0.67 * dimensions["NOSELENGTH"]
	wingCenterOfPressure = dimensions["NOSELENGTH"] + dimensions["DELTAXFROMNOSEBASETOWINGLE"] + 0.7 * dimensions["WINGROOTCHORDA1"] - 0.2 * dimensions["WINGROOTCHORDA2"]
	bodyCenterOfPressure = (0.67 * noseArea * dimensions["NOSELENGTH"] + bodyArea * (dimensions["NOSELENGTH"] + 0.5 * (dimensions["REFLENGTH"] - dimensions["NOSELENGTH"]))) / (noseArea + bodyArea)
	dynamicPressure = 0.5 * rho * speed * speed
	accelCommandGs = command / gravity
	refDiameter = dimensions["REFDIAM"]
	TEMP1 = (centerOfGrav - wingCenterOfPressure) / refDiameter
	TEMP2 = (centerOfGrav - dimensions["CENTEROFDEFL"]) / refDiameter
	TEMP3 = (centerOfGrav - bodyCenterOfPressure) / refDiameter
	TEMP4 = (centerOfGrav - noseCenterOfPressure) / refDiameter
	CNTRIM = weight * accelCommandGs / (dynamicPressure * refArea)
	Y1 = 2 + 8 * wingArea / (beta * refArea) + 8 * tailArea / (beta * refArea)
	Y2 = 1.5 * planformArea / refArea
	Y3 = 8 * tailArea / (beta * refArea)
	Y4 = 2 * TEMP4 + 8 * wingArea * TEMP1 / (beta * refArea) + 8 * tailArea * TEMP2 / (beta * refArea)
	Y5 = 1.5 * planformArea * TEMP3 / refArea
	Y6 = 8 * tailArea * TEMP2 / (beta * refArea)
	P2 = Y2 - (Y3 * Y5) / Y6
	P3 = Y1 - (Y3 * Y4) / Y6
	alphaTrim = (-1 * P3 + np.sqrt(P3 * P3 + 4 * P2 * CNTRIM)) / (2 * P2)
	CNA = 2 + 1.5 * planformArea * alphaTrim / refArea + 8 * wingArea / (beta * refArea) + 8 * tailArea / (beta * refArea)
	CND = 8 * tailArea / (beta * refArea)
	ZA = -1 * gravity * dynamicPressure * refArea * CNA / (weight * speed)
	ZD = -1 * gravity * dynamicPressure * refArea * CND / (weight * speed)
	CMAP = 2 * TEMP4 + 1.5 * planformArea * alphaTrim * TEMP3 / refArea + 8 * wingArea * TEMP1 / (beta * refArea)
	CMA = CMAP + 8 * tailArea * TEMP2 / (beta * refArea)
	CMD = 8 * tailArea * TEMP2 / (beta * refArea)
	MA = dynamicPressure * refArea * refDiameter * CMA / transverseMomentOfInertia
	MD = dynamicPressure * refArea * refDiameter * CMD / transverseMomentOfInertia
	omegaZ = np.sqrt((MA * ZD - MD * ZA) / ZD)
	if MA < 0:
		omegaAF = np.sqrt(-1 * MA)
	else:
		omegaAF = np.sqrt(MA)
	zetaAF = ZA * omegaAF / (2 * MA)
	KR = 0.1
	K1 = -1 * speed * ((MA * ZD - ZA * MD) / (1845 * MA))
	TA = MD / (MA * ZD - MD * ZA)
	K3 = 1845 * K1 / speed
	KDC = (1 - KR * K3) / (K1 * KR)
	return K3, TA, KR, KDC, omegaAF, zetaAF, K1, omegaZ