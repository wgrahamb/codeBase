# pitchZA = -1 * gravity * dynamicPressure * self.interceptorRefArea * pitchCNA / (self.refAeroData["weight"] * self.interceptorSpeed)

# pitchZD = -1 * gravity * dynamicPressure * self.interceptorRefArea * pitchCND / (self.refAeroData["weight"] * self.interceptorSpeed)

# pitchMA = dynamicPressure * self.interceptorRefArea * self.interceptorRefDiam * pitchCMA / self.refAeroData["transverseMomentOfInertia"]

# pitchMD = dynamicPressure * self.interceptorRefArea * self.interceptorRefDiam * pitchCMD / self.refAeroData["transverseMomentOfInertia"]

# omegaZ = np.sqrt((pitchMA * pitchZD - pitchZA * pitchMD) / pitchZD)
# omegaAF = np.sqrt(-1 * pitchMA)
# zetaAF = pitchZA * omegaAF / (2 * pitchMA)
# KR = 0.0000000001
# speedFeetPerSec = self.interceptorSpeed * 3.28084
# K1 = -1 * speedFeetPerSec * ((pitchMA * pitchZD - pitchZA * pitchMD) / 1845 * pitchMA)
# TA = pitchMD / (pitchMA * pitchZD - pitchZA * pitchMD)
# K3 = 1845 * K1 / speedFeetPerSec
# KDC = (1 - KR * K3) / (K1 * KR)

# eOld = self.pitchE
# eDotOld = self.pitchE
# thetaDot = K3 * (eOld + TA * eDotOld)
# normCommFeetPerS2 = self.normalAccelCommandBody * 3.28084
# normCommGs = self.normalAccelCommandBody / gravity
# deflection = KR * (KDC * (self.normalAccelCommandBody + gravity) + self.interceptorRate[1])
# if np.abs(deflection) > self.maxDeflection:
# 	deflection = np.sign(deflection) * self.maxDeflection
# print(deflection)
# self.pitchControlFinDeflection = np.radians(deflection)
# eDotDot = (omegaAF ** 2) * (deflection - eOld - 2 * zetaAF * eDotOld / omegaAF)
# eNew = eOld + self.timeStep * eDotOld
# eDotNew = eDotOld + self.timeStep * eDotDot
# self.pitchE = (eNew + eOld + self.timeStep * eDotNew) / 2
# self.pitchEDot = (eDotNew + eDotOld + self.timeStep * eDotDot) / 2

# zrate = DNA / self.interceptorSpeed - DMA * DND / (self.interceptorSpeed * DMD)
# AA = DNA / self.interceptorSpeed - DMQ
# BB = -1 * DMA - DMQ * DNA / self.interceptorSpeed
# TEMP1 = (AA - 2 * 0.6 * 0.6 * zrate)
# TEMP2 = (AA * AA - 4 * 0.6 * 0.6 * BB)
# RADIX = TEMP1 * TEMP1 - TEMP2
# GRATE = (-1 * TEMP1 + np.sqrt(RADIX)) / (-1 * DMD)
# print(GRATE, self.interceptorRate[1])
# defl = GRATE * self.interceptorRate[1]
# if defl == 0.0:
# 	defl = 0.01
# elif np.abs(defl) > np.radians(28.0):
# 	defl = np.sign(defl) * np.radians(28.0)
# self.pitchControlFinDeflection = defl