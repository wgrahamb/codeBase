# INCLUDED WITH PYTHON
import time
from enum import Enum

# PIP INSTALLED LIBRARIES
import numpy as np
from numpy import array as npa
from numpy import linalg as la

# UTILITY
from utility.coordinateTransformations import FLIGHTPATH_TO_LOCAL_TM
from utility.coordinateTransformations import ORIENTATION_TO_LOCAL_TM
from utility.loadPickle import loadpickle as lp
from utility.unitVector import unitvector
from utility.returnAzAndElevation import returnAzAndElevation
from utility.projection import projection
from utility.SecondOrderActuator import SecondOrderActuator
from utility.ATM1976 import ATM1976
from utility import loggingFxns as lf

RAD_TO_DEG = 57.3

class endChecks(Enum):
     intercept = 1
     flying = 0
     groundCollision = -1
     pointOfClosestApproachPassed = -2
     notANumber = -3
     maxTimeExceeded = -4
     forcedSimTermination = -5

class sixDofSim:

     def __init__(self):

          ############################################################################
          #
          # AUTHOR - WILSON GRAHAM BEECH
          # REFERENCE - MODELING AND SIMULATION OF AEROSPACE
          # VEHICLE DYNAMICS SECOND EDITON - PETER H. ZIPFEL
          #
          # EAST, NORTH, UP COORDINATE SYSTEM
          #
          # INTERCEPTOR LOCAL ORIENTATION
          # ARRAY 0 >>> LOOKING DOWN THE NOZZLE OF THE INTERCEPTOR
          # ARRAY 1 >>> LOOKING DOWN THE NOZZLE OF THE INTERCEPTOR,
          # THIS POINTS OUT THE LEFT HAND SIDE
          # ARRAY 2 >>> LOOKING DOWN THE NOZZLE OF THE INTERCEPTOR,
          # THIS POINTS OUT THE TOP SIDE OF THE INTERCEPTOR
          #
          #                           POSITIVE NORMAL
          #                                         |
          #                                         |
          #                                         |
          #  POSITIVE SIDE -----------O----------- NEGATIVE SIDE
          #                                         |
          #                                         |
          #                                         |
          #                          NEGATIVE NORMAL
          #
          # NEGATIVE AXIS IS COMING OUT OF THE SCREEN STRAIGHT AT YOU
          # POSITIVE AXIS IS POINTING INTO THE SCREEN DIRECTLY AWAY FROM YOU
          #
          # POSITIVE ALPHA INDICATES NOSE BELOW FREE STREAM VELOCITY
          # POSITIVE BETA INDICATES NOSE LEFT FREE STREAM VELOCITY
          # POSITIVE ROLL INDICATES NORMAL AXIS CLOCKWISELY
          # ROTATED FROM TWELVE O'CLOCK
          #
          # FIN ORIENTATION
          # LOOKING DOWN THE NOZZLE OF THE MISSILE
          #
          #                              FIN 4       FIN 1
          #                                         X
          #                              FIN 3       FIN 2
          #
          ############################################################################

          # SIM CONTROL.
          self.wallClockStart = time.time()
          self.go = True
          self.timeStep = 0.005 # SECONDS
          self.maxTime = 400 # SECONDS

          # TARGET.
          self.tgtPos = npa([5000.0, 5000.0, 5000.0])
          self.tgtVel = np.zeros(3)

          # Input.
          # RADIANS >>> ONLY POSITIVE NUMBERS 0-360,
          # MEASURED COUNTER CLOCKWISE FROM TRUE EAST
          mslAz = np.radians(20) 
          mslEl = np.radians(35) # RADIANS

          ### MSL STATE ###
          self.mslTof = 0.0 # SECONDS

          # FRAME
          self.mslENUtoFLU = FLIGHTPATH_TO_LOCAL_TM(mslAz, -mslEl) # ND
          self.mslPosEnu = np.zeros(3)# METERS
          self.mslVelEnu = self.mslENUtoFLU[0] # METERS PER SECOND
          self.mslAccEnu = np.zeros(3) # METERS PER SECOND^2
          self.mslEulerEnu = npa([0.0, mslEl, mslAz]) # RADIANS
          self.mslEulerDotEnu = np.zeros(3) # RADIANS PER SECOND

          # BODY.
          self.mslVelB = self.mslENUtoFLU @ self.mslVelEnu # METERS PER SECOND
          self.mslSpecificForce = self.mslENUtoFLU @ self.mslAccEnu # METERS PER SECOND^2
          self.mslSpeed = la.norm(self.mslVelB) # METERS PER SECOND
          self.mslMach = 0.0 # ND
          self.mslRange = 0.0 # METERS
          self.mslAlpha = 0.0 # RADIANS
          self.mslBeta = 0.0 # RADIANS
          self.mslRate = np.zeros(3) # RADIANS PER SECOND
          self.mslRateDot = np.zeros(3) # RADIANS PER SECOND^2

          # MISSILE CONSTANTS.
          self.mslRefArea = 0.01824 # METERS^2
          self.mslRefDiam = 0.1524 # METERS
          self.mslExitArea = 0.0125 # METERS^2
          self.mslBurnOut = 2.421 # SECONDS

          ###### FUNCTION VARIABLES ######
          # ATMOSPHERE.
          self.ATMOS = ATM1976()
          self.ATMOS.update(self.mslPosEnu[2], la.norm(self.mslVelEnu))
          self.RHO = self.ATMOS.rho # Kilograms per meter cubed.
          self.Q = self.ATMOS.q # Pascals.
          self.P = self.ATMOS.p # Pascals.
          self.A = self.ATMOS.a # Meters per second.
          self.G = self.ATMOS.g # Meters per second squared.
          self.mslMach = self.ATMOS.mach # Non dimensional.
          self.gravBodyVec = np.zeros(3) # METERS PER SECOND^2

          # SEEKER >>> INITIALIZE BY POINTING THE SEEKER DIRECTLY AT THE TARGET
          relPosU = unitvector(self.tgtPos - self.mslPosEnu) # ND
          mslToInterceptU = self.mslENUtoFLU @ relPosU # ND
          mslToInterceptAz, mslToInterceptEl = returnAzAndElevation(mslToInterceptU) # RADIANS
          self.seekerPitch = mslToInterceptEl # RADIANS
          self.seekerYaw = mslToInterceptAz # RADIANS
          self.seekerLocalOrient = np.zeros((3, 3)) # ND
          self.seekPitchErr = 0.0 # ND
          self.seekYawErr = 0.0 # ND
          self.gk = 10 # KALMAN FILTER GAIN >>> ONE PER SECOND
          self.zetak = 0.9 # KALMAN FILTER DAMPING
          self.wnk = 60 # KALMAN FILTER NATURAL FREQUENCY >>> RADIANS PER SECOND
          self.wlr = self.seekerYaw # RADIANS PER SECOND >>> POINTING YAW RATE
          self.wlrd = 0.0 # RADIANS PER SECOND^2 >>> DERIVATIVE OF POINTING YAW RATE
          self.wlr1 = 0.0 # RADIANS PER SECOND >>> YAW SIGHT LINE SPIN RATE
          self.wlr1d = 0.0 # RADIANS PER SECOND^2 >>>
          # DERIVATIVE OF YAW SIGHT LINE SPIN RATE
          self.wlr2 = 0.0 # RADIANS PER SECOND^2 >>>
          # SECOND STATE VARIABLE IN KALMAN FILTER, YAW
          self.wlr2d = 0.0 # RADIANS PER SECOND^3 >>>
          # DERIVATIVE OF SECOND STATE VARIABLE IN KALMAN FILTER, YAW
          self.wlq = self.seekerPitch # RADIANS PER SECOND >>> POINTING PITCH RATE
          self.wlqd = 0.0 # RADIANS PER SECOND^2 >>>
          # DERIVATIVE OF POINTING PITCH RATE
          self.wlq1 = 0.0 # RADIANS PER SECOND >>> PITCH SIGHT LINE SPIN RATE
          self.wlq1d = 0.0 # RADIANS PER SECOND^2 >>>
          # DERIVATIVE OF PITCH SIGHT LINE SPIN RATE
          self.wlq2 = 0.0 # RADIANS PER SECOND^2 >>>
          # SECOND STATE VARIABLE IN KALMAN FILTER, PITCH
          self.wlq2d = 0.0 # RADIANS PER SECOND^3 >>>
          # DERIVATIVE OF SECOND STATE VARIABLE IN KALMAN FILTER, PITCH

          # GUIDANCE
          self.forwardLeftUpMslToInterceptRelPos = np.zeros(3) # METERS
          self.loft = 0.4 # ND >>> THIS IS THE ELEVATION PARAMETER OF THE LINE OF
          # ATTACK, ASSUMING LINE OF SIGHT FOR THE AZIMUTH PARAMETER
          self.K = 1 # ND
          self.proNavGain = 3 # ND
          self.normCommand = 0.0 # METERS PER SECOND^2
          self.sideCommand = 0.0 # METERS PER SECOND^2
          self.maxAccelAllow = 500.0 # METERS PER SECOND^2 >>> ROUGHLY FIFTY Gs
          self.maxAccel = self.maxAccelAllow

          # CONTROL
          self.zetlagr = 0.6 # ND >>> DAMPING OF CLOSED RATE LOOP
          self.wrcl = 20 # RADIANS PER SECOND >>>
          # FREQUENCY OF ROLL CLOSED LOOP COMPLEX POLE
          self.zrcl = 0.9 # ND >> DAMPING OF ROLL CLOSED LOOP POLE
          self.yy = 0.0 # METERS PER SECOND >>> YAW FEED FORWARD INTEGRATION
          self.yyd = 0.0 # METERS PER SECOND >>> YAW FEED FORWARD DERIVATIVE
          self.zz = 0.0 # METERS PER SECOND >>> PITCH FEED FORWARD INTEGRATION
          self.zzd = 0.0 # METER PER SECOND >>> PITCH FEED FORWARD DERIVATIVE
          self.maxDefl = 28.0 # DEGREES 
          self.pitchFinComm = 0.0 # RADIANS
          self.yawFinComm = 0.0 # RADIANS
          self.rollFinComm = 0.0 # RADIANS
          self.PHI_ANGLE_COMMAND = 0.0 # RADIANS

          # ACTUATORS
          self.DEL1C = 0.0
          self.DEL2C = 0.0
          self.DEL3C = 0.0
          self.DEL4C = 0.0
          self.FIN1 = SecondOrderActuator("FIN1")
          self.FIN2 = SecondOrderActuator("FIN2")
          self.FIN3 = SecondOrderActuator("FIN3")
          self.FIN4 = SecondOrderActuator("FIN4")
          self.DEL1 = 0.0 # RADIANS >>> FIN POSITION
          self.DEL2 = 0.0 # RADIANS
          self.DEL3 = 0.0 # RADIANS
          self.DEL4 = 0.0 # RADIANS
          self.pitchFinDefl = 0.0 # RADIANS
          self.yawFinDefl = 0.0 # RADIANS
          self.rollFinDefl = 0.0 # RADIANS

          # ANGLES
          self.alphaPrimeDeg = 0.0 # DEGREES
          self.sinPhiPrime = 0.0 # ND
          self.cosPhiPrime = 0.0 # ND
          self.pitchDeflAeroDeg = 0.0 # DEGREES
          self.yawDeflAeroDeg = 0.0 # DEGREES
          self.rollDeflDeg = 0.0 # DEGREES
          self.totalFinDeflDeg = 0.0 # DEGREES
          self.pitchRateAeroDeg = 0.0 # DEGREES PER SECOND
          self.yawRateAeroDeg = 0.0 # DEGREES PER SECOND
          self.rollRateDeg = 0.0 # DEGREES PER SECOND
          self.sinOfFourTimesPhiPrime = 0.0 # ND
          self.squaredSinOfTwoTimesPhiPrime = 0.0 # ND

          # DATA LOOK UP
          self.lookUpValues = lp("PY_6DOF_SRAAM/lookUpTables.pickle")
          self.CA0 = 0.0 # ND
          self.CAA = 0.0 # PER DEGREE
          self.CAD = 0.0 # PER DEGREE^2
          self.CAOFF = 0.0 # ND
          self.CYP = 0.0 # ND
          self.CYDR = 0.0 # PER DEGREE
          self.CN0 = 0.0 # ND
          self.CNP = 0.0 # ND
          self.CNDQ = 0.0 # PER DEGREE
          self.CLLAP = 0.0 # PER DEGREE^2
          self.CLLP = 0.0 # PER DEGREE
          self.CLLDP = 0.0 # PER DEGREE
          self.CLM0 = 0.0 # ND
          self.CLMP = 0.0 # ND
          self.CLMQ = 0.0 # PER DEGREE
          self.CLMDQ = 0.0 # PER DEGREE
          self.CLNP = 0.0 # ND
          self.mass = 0.0 # KILOGRAMS
          self.unAdjThrust = 0.0 # NEWTONS
          self.tmoi = 0.0 # KILOGRAMS * METERS^2
          self.amoi = 0.0 # KILOGRAMS * METERS^2
          self.cgFromNose = 0.0 # METERS
          self.alphaPrimeMax = 40.0 # DEGREES

          # PROPULSION
          self.seaLevelPress = 101325
          self.thrust = 0.0

          # AERO DYNAMIC COEFFICIENTS
          self.launchCg = self.lookUpValues["CENTER OF GRAVITY"](0.0) # METERS
          self.CX = 0 # AXIAL FORCE COEFFICIENT
          self.CY = 0 # SIDE FORCE COEFFICIENT
          self.CZ = 0 # NORMAL FORCE COEFFICIENT
          self.CL = 0 # ROLL MOMENT COEFFICIENT
          self.CM = 0 # PITCHING MOMENT COEFFICIENT
          self.CN = 0 # YAWING MOMENT COEFFICIENT

          # AERO DYNAMIC REFERENCE DATA
          self.refAeroData = {}
          self.staticMargin = 0

          # DATA
          self.logFile = open("PY_6DOF_SRAAM/log.txt", "w")
          self.STATE = self.populateState()
          lf.writeHeader(self.STATE, self.logFile)
          lf.writeData(self.STATE, self.logFile)

          # END CHECK
          self.missDistance = 0.0 # METERS
          self.lethality = endChecks.flying # ND

     def populateState(self):
          STATE = {
               "tof": self.mslTof,
               "posE": self.mslPosEnu[0],
               "posN": self.mslPosEnu[1],
               "posU": self.mslPosEnu[2],
               "tgtE": self.tgtPos[0],
               "tgtN": self.tgtPos[1],
               "tgtU": self.tgtPos[2],
               "normComm": self.normCommand / self.G,
               "normAch": self.mslSpecificForce[2] / self.G,
               "sideComm": self.sideCommand / self.G,
               "sideAch": self.mslSpecificForce[1] / self.G,
               "p": self.mslRate[0],
               "q": self.mslRate[1],
               "r": self.mslRate[2],
               "phi": self.mslEulerEnu[0],
               "alpha": self.mslAlpha,
               "beta": self.mslBeta,
               "fin1c": self.DEL1C,
               "fin2c": self.DEL2C,
               "fin3c": self.DEL3C,
               "fin4c": self.DEL4C,
               "fin1d": self.DEL1,
               "fin2d": self.DEL2,
               "fin3d": self.DEL3,
               "fin4d": self.DEL4,
          }
          return STATE

     def trapIntegrate(self, dy_new, dy, y, step):
          return y + (dy_new + dy) * step / 2

     def target(self):

          # # MANEUVERING TARGET.
          # target = npa([10.0, 10.0, 0.0])
          # relPos = target - self.tgtPos
          # relPosU = unitvector(relPos)
          # closingVel = -1 * self.tgtVel # METERS PER SECOND
          # closingVelMag = la.norm(closingVel) # METERS PER SECOND
          # TEMP1 = np.cross(relPos, closingVel)
          # TEMP2 = np.dot(relPos, relPos)
          # lineOfSightRate = TEMP1 / TEMP2 # RADIANS PER SECOND
          # command = \
          # np.cross(-1 * self.proNavGain * closingVelMag * relPosU, lineOfSightRate)

          deltaVel = np.zeros(3) * self.timeStep
          self.tgtVel += deltaVel
          deltaPos = self.tgtVel * self.timeStep
          self.tgtPos += deltaPos

     def atmosphere(self):

          self.ATMOS = ATM1976()
          self.ATMOS.update(self.mslPosEnu[2], la.norm(self.mslVelEnu))
          self.RHO = self.ATMOS.rho # Kilograms per meter cubed.
          self.Q = self.ATMOS.q # Pascals.
          self.P = self.ATMOS.p # Pascals.
          self.A = self.ATMOS.a # Meters per second.
          self.G = self.ATMOS.g # Meters per second squared.
          self.mslMach = self.ATMOS.mach # Non dimensional.

          gravLocalVec = npa([0.0, 0.0, -self.G]) # METERS PER SECOND^2
          self.gravBodyVec = self.mslENUtoFLU @ gravLocalVec # METERS PER SECOND^2

     def seeker(self):

          wsq = self.wnk ** 2 # ND
          gg = self.gk * wsq # ND

          # YAW CHANNEL
          wlr1d_new = self.wlr2 # RADIANS PER SECOND^2
          self.wlr1 = \
          self.trapIntegrate(wlr1d_new, self.wlr1d, self.wlr1, self.timeStep) # RADS PER SEC
          self.wlr1d = wlr1d_new # RADIANS PER SECOND^2
          wlr2d_new = \
          gg * self.seekYawErr - \
          2 * self.zetak * self.wnk * self.wlr1d - \
          wsq * self.wlr1 # RADIANS PER SECOND^3
          self.wlr2 = \
          self.trapIntegrate(wlr2d_new, self.wlr2d, self.wlr2, self.timeStep) # RADS PER SEC^2
          self.wlr2d = wlr2d_new # RADIANS PER SECOND^3

          # PITCH CHANNEL
          wlq1d_new = self.wlq2 # RADIANS PER SECOND^2
          self.wlq1 = \
          self.trapIntegrate(wlq1d_new, self.wlq1d, self.wlq1, self.timeStep) # RADS PER SEC
          self.wlq1d = wlq1d_new # RADIANS PER SECOND^2
          wlq2d_new = \
          gg * self.seekPitchErr - \
          2 * self.zetak * self.wnk * self.wlq1d - \
          wsq * self.wlq1 # RADIANS PER SECOND^3
          self.wlq2 = \
          self.trapIntegrate(wlq2d_new, self.wlq2d, self.wlq2, self.timeStep) # RADS PER SEC^2
          self.wlq2d = wlq2d_new # RADIANS PER SECOND^3

          # YAW CONTROL
          wlrd_new = self.wlr1 - self.mslEulerDotEnu[2] # RADIANS PER SECOND^2
          self.wlr = \
          self.trapIntegrate(wlrd_new, self.wlrd, self.wlr, self.timeStep) # RADS PER SECOND
          self.wlrd = wlrd_new # RADIANS PER SECOND^2
          self.seekerYaw = self.wlr # RADIANS

          # PITCH CONTROL
          wlqd_new = self.wlq1 - self.mslEulerDotEnu[1] # RADIANS PER SECOND^2
          self.wlq = \
          self.trapIntegrate(wlqd_new, self.wlqd, self.wlq, self.timeStep) # RADS PER SECOND
          self.wlqd = wlqd_new # RADIANS PER SECOND^2
          self.seekerPitch = self.wlq # RADIANS

          # TRANSFORM.
          localRelPos = self.tgtPos - self.mslPosEnu # METERS
          seekerAttitudeToLocalTM = \
          ORIENTATION_TO_LOCAL_TM(0, -self.seekerPitch, self.seekerYaw) # ND
          self.seekerLocalOrient = seekerAttitudeToLocalTM @ self.mslENUtoFLU # ND
          seekerToInterceptRelPos = (self.seekerLocalOrient @ localRelPos) \
          * npa([1.0, 0.5, 0.2]) # METERS >>> ARRAY AT THE END SIMULATES ERROR
          self.seekYawErr, self.seekPitchErr = \
          returnAzAndElevation(seekerToInterceptRelPos) # RADIANS
          self.forwardLeftUpMslToInterceptRelPos = \
          (seekerToInterceptRelPos @ seekerAttitudeToLocalTM) # METERS

     def guidance(self):

          # INPUT FROM SEEKER.
          forwardLeftUpMslToInterceptRelPosU = \
          unitvector(self.forwardLeftUpMslToInterceptRelPos) # ND
          forwardLeftUpMslToInterceptLineOfSightVel = \
          projection(forwardLeftUpMslToInterceptRelPosU, self.mslVelB) # M/S

          # TIME TO GO.
          timeToGo = la.norm(self.forwardLeftUpMslToInterceptRelPos) / \
          la.norm(forwardLeftUpMslToInterceptLineOfSightVel) # SECONDS

          # PROPORTIONAL GUIDANCE.
          if timeToGo < 3:
               closingVel = -1 * self.mslVelB # METERS PER SECOND
               closingVelMag = la.norm(closingVel) # METERS PER SECOND
               TEMP1 = np.cross(self.forwardLeftUpMslToInterceptRelPos, closingVel)
               TEMP2 = np.dot(
                    self.forwardLeftUpMslToInterceptRelPos,
                    self.forwardLeftUpMslToInterceptRelPos
               )
               lineOfSightRate = TEMP1 / TEMP2 # RADIANS PER SECOND
               command = np.cross(
                    -1 * self.proNavGain * closingVelMag * forwardLeftUpMslToInterceptRelPosU,
                    lineOfSightRate
               ) # METERS PER SECOND^2
               self.normCommand = command[2] # METERS PER SECOND^2
               self.sideCommand = command[1] # METERS PER SECOND^2
          
          # LINE OF ATTACK.
          else:
               lineOfAttack = npa(
                    [
                         forwardLeftUpMslToInterceptRelPosU[0],
                         forwardLeftUpMslToInterceptRelPosU[1],
                         self.loft
                    ]
               ) # ND
               forwardLeftUpMslToInterceptLineOfAttackVel = \
               projection(lineOfAttack, self.mslVelB) # METERS PER SECOND
               G = 1 - np.exp(-1 * la.norm(self.forwardLeftUpMslToInterceptRelPos) / 10000) # ND
               command = npa(
                    [
                         0.0,
                         self.K * (forwardLeftUpMslToInterceptLineOfSightVel[1] + \
                         G * forwardLeftUpMslToInterceptLineOfAttackVel[1]),
                         self.K * (forwardLeftUpMslToInterceptLineOfSightVel[2] + \
                         G * forwardLeftUpMslToInterceptLineOfAttackVel[2]),
                    ]
               ) # METERS PER SECOND^2
               self.normCommand = command[2] # METERS PER SECOND^2
               self.sideCommand = command[1] # METERS PER SECOND^2
               
          # LIMIT ACCELERATION.
          accMag = la.norm(npa([self.sideCommand, self.normCommand]))
          trigonometricRatio = np.arctan2(self.normCommand, self.sideCommand) # ND
          if accMag > self.maxAccel:
               accMag = self.maxAccel # METERS PER SECOND^2
          self.sideCommand = accMag * np.cos(trigonometricRatio) # METERS PER SECOND^2
          self.normCommand = accMag * np.sin(trigonometricRatio) # METERS PER SECOND^2

     def control(self):

          # MANEUVERING
          if len(self.refAeroData) > 0 and self.mslMach > 0.6:
                    
               CNA = self.refAeroData["CNA"] * RAD_TO_DEG # ND
               CMA = self.refAeroData["CMA"] * RAD_TO_DEG # ND
               CMD = self.refAeroData["CMD"] * RAD_TO_DEG # ND
               CMQ = self.refAeroData["CMQ"] * RAD_TO_DEG # ND
               CLP = self.refAeroData["CLP"] * RAD_TO_DEG # ND
               CLD = self.refAeroData["CLD"] * RAD_TO_DEG # ND
               mass = self.mass # KILOGRAMS
               tMoi = self.tmoi # KILOGRAMS * METERS^2
               aMoi = self.amoi # KILOGRAMS * METERS^2

               DNA = CNA * (self.Q * self.mslRefArea / mass) # METERS PER SECOND^2
               DMA = CMA * (self.Q * self.mslRefArea * self.mslRefDiam / tMoi) # PER SEC^2
               DMD = CMD * (self.Q * self.mslRefArea * self.mslRefDiam / tMoi) # PER SEC^2
               DMQ = CMQ * (self.mslRefDiam / (2 * self.mslSpeed)) * \
               (self.Q * self.mslRefArea * self.mslRefDiam / tMoi) # PER SECOND
               DLP = CLP * (self.mslRefDiam / (2 * self.mslSpeed)) * \
               (self.Q * self.mslRefArea * self.mslRefDiam / aMoi) # PER SECOND
               DLD = CLD * (self.Q * self.mslRefArea * self.mslRefDiam / aMoi) # PER SEC^2

               WACL = 0.013 * np.sqrt(self.Q) + 7.1 # PER SECOND
               ZACL = 0.000559 * np.sqrt(self.Q) + 0.232 # ND
               PACL = 14 # ND

               # FEEDBACK GAINS
               GAINFB3 = WACL * WACL * PACL / (DNA * DMD) # ND
               GAINFB2 = (
                    2 * ZACL * WACL +
                    PACL +
                    DMQ -
                    DNA / self.mslSpeed
               ) / DMD # ND
               GAINFB1 = (
                    WACL ** 2 +
                    2 * ZACL * WACL * PACL +
                    DMA +
                    DMQ * DNA / self.mslSpeed -
                    GAINFB2 * DMD * DNA / self.mslSpeed
               ) / (DNA * DMD) # ND
               
               # ROLL
               GKP = (2 * self.wrcl * self.zrcl + DLP) / DLD # ND
               GKPHI = self.wrcl * self.wrcl / DLD # ND
               EPHI = GKPHI * (self.PHI_ANGLE_COMMAND - self.mslEulerEnu[0]) # RADIANS
               self.rollFinComm = EPHI - GKP * self.mslRate[0] # RADIANS

               # PITCH
               zzdNew = self.normCommand - self.mslSpecificForce[2] # METERS PER SECOND
               self.zz = \
               self.trapIntegrate(zzdNew, self.zzd, self.zz, self.timeStep) # METERS PER SECOND
               self.zzd = zzdNew # METERS PER SECOND
               deflPitch = \
               -1 * GAINFB1 * self.mslSpecificForce[2] - \
               GAINFB2 * self.mslRate[1] + \
               GAINFB3 * self.zz # DEGREES
               if np.abs(deflPitch) > self.maxDefl:
                    deflPitch = np.sign(deflPitch) * self.maxDefl # DEGREES
               self.pitchFinComm = np.radians(deflPitch) # RADIANS

               # YAW
               yydNew = self.mslSpecificForce[1] - self.sideCommand # METERS PER SECOND
               self.yy = \
               self.trapIntegrate(yydNew, self.yyd, self.yy, self.timeStep) # METERS PER SECOND
               self.yyd = yydNew # METERS PER SECOND
               deflYaw = \
               -1 * GAINFB1 * -1 * self.mslSpecificForce[1] - \
               GAINFB2 * self.mslRate[2] + \
               GAINFB3 * self.yy # DEGREES
               if np.abs(deflYaw) > self.maxDefl: 
                    deflYaw = np.sign(deflYaw) * self.maxDefl # DEGREES
               self.yawFinComm = np.radians(deflYaw) # RADIANS

          # THIS KEEPS THE MISSILE ON LINE BEFORE IT
          # GAINS ENOUGH SPEED TO PROPERLY MANEUVER
          elif len(self.refAeroData) > 0:

               CNA = self.refAeroData["CNA"] * RAD_TO_DEG # ND
               CMA = self.refAeroData["CMA"] * RAD_TO_DEG # ND
               CMD = self.refAeroData["CMD"] * RAD_TO_DEG # ND
               CMQ = self.refAeroData["CMQ"] * RAD_TO_DEG # ND
               CLP = self.refAeroData["CLP"] * RAD_TO_DEG # ND
               CLD = self.refAeroData["CLD"] * RAD_TO_DEG # ND
               CND = self.refAeroData["CND"] * RAD_TO_DEG #ND
               mass = self.mass # KILOGRAMS
               tMoi = self.tmoi # KILOGRAMS * METERS^2
               aMoi = self.amoi # KILOGRAMS * METERS^2

               DNA = CNA * (self.Q * self.mslRefArea / mass) # METERS PER SECOND^2
               DND = CND * (self.Q * self.mslRefArea / mass) # METERS PER SECOND^2
               DMA = CMA * (self.Q * self.mslRefArea * self.mslRefDiam / tMoi) # PER SEC^2
               DMD = CMD * (self.Q * self.mslRefArea * self.mslRefDiam / tMoi) # PER SEC^2
               DMQ = CMQ * (self.mslRefDiam / (2 * self.mslSpeed)) * \
               (self.Q * self.mslRefArea * self.mslRefDiam / tMoi) # PER SECOND
               DLP = CLP * (self.mslRefDiam / (2 * self.mslSpeed)) * \
               (self.Q * self.mslRefArea * self.mslRefDiam / aMoi) # PER SECOND
               DLD = CLD * (self.Q * self.mslRefArea * self.mslRefDiam / aMoi) # PER SECOND^2

               # ROLL
               GKP = (2 * self.wrcl * self.zrcl + DLP) / DLD # ND
               GKPHI = self.wrcl * self.wrcl / DLD # ND
               EPHI = GKPHI * (self.PHI_ANGLE_COMMAND - self.mslEulerEnu[0]) # RADIANS
               self.rollFinComm = EPHI - GKP * self.mslEulerDotEnu[0] # RADIANS

               # RATE CONTROL
               ZRATE = DNA / self.mslSpeed - DMA * DND / (self.mslSpeed * DMD) # ND
               AA = DNA / self.mslSpeed - DMQ # ND
               BB = -1 * DMA - DMQ * DNA / self.mslSpeed # ND
               TEMP1 = AA - 2 * self.zetlagr * self.zetlagr * ZRATE # ND
               TEMP2 = AA * AA - 4 * self.zetlagr * self.zetlagr * BB # ND
               RADIX = TEMP1 ** 2 - TEMP2 # ND
               GRATE = (-1 * TEMP1 + np.sqrt(RADIX)) / (-1 * DMD) # ND

               # PITCH
               self.pitchFinComm = GRATE * self.mslEulerDotEnu[1] # RADIANS

               # YAW
               self.yawFinComm = GRATE * self.mslEulerDotEnu[2] # RADIANS

          else:
               self.PHI_ANGLE_COMMAND = 0.0
               self.pitchFinComm = 0.0
               self.yawFinComm = 0.0

     def actuators(self):

          self.DEL1C = -self.rollFinComm + self.pitchFinComm - self.yawFinComm
          self.DEL2C = -self.rollFinComm + self.pitchFinComm + self.yawFinComm
          self.DEL3C = self.rollFinComm + self.pitchFinComm - self.yawFinComm
          self.DEL4C = self.rollFinComm + self.pitchFinComm + self.yawFinComm

          self.FIN1.update(self.DEL1C, self.timeStep)
          self.FIN2.update(self.DEL2C, self.timeStep)
          self.FIN3.update(self.DEL3C, self.timeStep)
          self.FIN4.update(self.DEL4C, self.timeStep)

          self.DEL1 = self.FIN1.DEFLECTION
          self.DEL2 = self.FIN2.DEFLECTION
          self.DEL3 = self.FIN3.DEFLECTION
          self.DEL4 = self.FIN4.DEFLECTION

          self.rollFinDefl = (-self.DEL1 - self.DEL2 + self.DEL3 + self.DEL4) / 4
          self.pitchFinDefl = (self.DEL1 + self.DEL2 + self.DEL3 + self.DEL4) / 4
          self.yawFinDefl = (-self.DEL1 + self.DEL2 - self.DEL3 + self.DEL4) / 4

     def angles(self):
          alphaPrime = np.arccos(np.cos(self.mslAlpha) * np.cos(self.mslBeta)) # RADIANS
          self.alphaPrimeDeg = np.degrees(alphaPrime) # DEGREES
          phiPrime = np.arctan2(
               np.tan(self.mslBeta),
               np.sin(self.mslAlpha)
          ) # RADIANS
          self.sinPhiPrime = np.sin(phiPrime) # ND
          self.cosPhiPrime = np.cos(phiPrime) # ND

          pitchDeflAeroFrame = \
          self.pitchFinDefl * self.cosPhiPrime - \
          self.yawFinDefl * self.sinPhiPrime # RADIANS

          self.pitchDeflAeroDeg = np.degrees(pitchDeflAeroFrame) # DEGREES

          yawDeflAeroFrame = \
          self.pitchFinDefl * self.sinPhiPrime + \
          self.yawFinDefl * self.cosPhiPrime # RADIANS

          self.yawDeflAeroDeg = np.degrees(yawDeflAeroFrame) # DEGREES
          self.rollDeflDeg = np.degrees(self.rollFinDefl)

          self.totalFinDeflDeg = \
          (np.abs(self.pitchDeflAeroDeg) + np.abs(self.yawDeflAeroDeg)) / 2 # DEGREES

          pitchRateAeroFrame = \
          self.mslEulerDotEnu[1] * self.cosPhiPrime - \
          self.mslEulerDotEnu[2] * self.sinPhiPrime # RADIANS PER SECOND
          
          self.pitchRateAeroDeg = np.degrees(pitchRateAeroFrame) # DEGREES PER SECOND
          
          yawRateAeroFrame = \
          self.mslEulerDotEnu[1] * self.sinPhiPrime + \
          self.mslEulerDotEnu[2] * self.cosPhiPrime # RADIANS PER SECOND

          self.yawRateAeroDeg = np.degrees(yawRateAeroFrame) # DEGREES PER SECOND
          self.rollRateDeg = np.degrees(self.mslEulerDotEnu[0]) # DEGREES PER SECOND
          self.sinOfFourTimesPhiPrime = np.sin(4 * phiPrime) # DIMENSIONLESS
          self.squaredSinOfTwoTimesPhiPrime = (np.sin(2 * phiPrime)) ** 2 # DIMENSIONLESS

     def dataLookUp(self):
          self.CA0 = self.lookUpValues["CA0"](self.mslMach) # ND
          self.CAA = self.lookUpValues["CAA"](self.mslMach) # PER DEGREE
          self.CAD = self.lookUpValues["CAD"](self.mslMach) # PER DEGREE^2
          if self.mslTof <= self.mslBurnOut:
               self.CAOFF = 0.0 # ND
          else:
               self.CAOFF = self.lookUpValues["CAOFF"](self.mslMach) # ND
          self.CYP = self.lookUpValues["CYP"](self.mslMach, self.alphaPrimeDeg)[0] # ND

          self.CYDR = \
          self.lookUpValues["CYDR"](self.mslMach, self.alphaPrimeDeg)[0] # PER DEGREE

          self.CN0 = self.lookUpValues["CN0"](self.mslMach, self.alphaPrimeDeg)[0] # ND
          self.CNP = self.lookUpValues["CNP"](self.mslMach, self.alphaPrimeDeg)[0] # ND

          self.CNDQ = \
          self.lookUpValues["CNDQ"](self.mslMach, self.alphaPrimeDeg)[0] # PER DEGREE

          self.CLLAP = \
          self.lookUpValues["CLLAP"](self.mslMach, self.alphaPrimeDeg)[0] # PER DEGREE^2

          self.CLLP = \
          self.lookUpValues["CLLP"](self.mslMach, self.alphaPrimeDeg)[0] # PER DEGREE

          self.CLLDP = \
          self.lookUpValues["CLLDP"](self.mslMach, self.alphaPrimeDeg)[0] # PER DEGREE

          self.CLM0 = self.lookUpValues["CLM0"](self.mslMach, self.alphaPrimeDeg)[0] # ND
          self.CLMP = self.lookUpValues["CLMP"](self.mslMach, self.alphaPrimeDeg)[0] # ND
          self.CLMQ = self.lookUpValues["CLMQ"](self.mslMach)  # PER DEGREE

          self.CLMDQ = \
          self.lookUpValues["CLMDQ"](self.mslMach, self.alphaPrimeDeg)[0] # PER DEGREE

          self.CLNP = self.lookUpValues["CLNP"](self.mslMach, self.alphaPrimeDeg)[0] # ND
          self.mass = self.lookUpValues["MASS"](self.mslTof) # KILOGRAMS
          self.unAdjThrust = self.lookUpValues["THRUST"](self.mslTof) # NEWTONS
          self.tmoi = self.lookUpValues["TRANSVERSE MOI"](self.mslTof) # KG * METERS^2
          self.amoi = self.lookUpValues["AXIAL MOI"](self.mslTof) # KG * METERS ^ 2
          self.cgFromNose = self.lookUpValues["CENTER OF GRAVITY"](self.mslTof) # METERS

     def maneuveringLimit(self):
          CN0MAX = self.lookUpValues["CN0"](
               self.mslMach, self.alphaPrimeMax
          )[0] # ND
          MAXACCEL = CN0MAX * self.Q * self.mslRefArea / self.mass
          CURRENTACCEL = self.CN0 * self.Q * self.mslRefArea / self.mass
          AVAILACCEL = MAXACCEL - CURRENTACCEL # METERS PER SECOND^2
          if AVAILACCEL < 0:
               self.maxAccel = 1 # METERS PER SECOND^2
          elif AVAILACCEL > self.maxAccelAllow:
               self.maxAccel = self.maxAccelAllow # METERS PER SECOND^2
          else:
               self.maxAccel = AVAILACCEL # METERS PER SECOND^2

     def propulsion(self):
          if self.mslTof > self.mslBurnOut:
               self.thrust = 0
          else:
               self.thrust = self.unAdjThrust + \
               (self.seaLevelPress - self.P) * self.mslExitArea # NEWTONS

     def aerodynamics(self):

          CYAERO = \
          self.CYP * self.sinOfFourTimesPhiPrime + \
          self.CYDR * self.yawDeflAeroDeg # ND

          CZAERO = \
          self.CN0 + \
          self.CNP * self.squaredSinOfTwoTimesPhiPrime + \
          self.CNDQ * self.pitchDeflAeroDeg # ND

          CNAEROREF = \
          self.CLNP * self.sinOfFourTimesPhiPrime + \
          self.CLMQ * self.yawRateAeroDeg * self.mslRefDiam / (2 * self.mslSpeed) + \
          self.CLMDQ * self.yawDeflAeroDeg # ND

          CNAERO = \
          CNAEROREF - \
          CYAERO * (self.launchCg - self.cgFromNose) / self.mslRefDiam # ND

          CMAEROREF = \
          self.CLM0 + \
          self.CLMP * self.squaredSinOfTwoTimesPhiPrime + \
          self.CLMQ * self.pitchRateAeroDeg * self.mslRefDiam / (2 * self.mslSpeed) + \
          self.CLMDQ * self.pitchDeflAeroDeg # ND

          CMAERO = \
          CMAEROREF - \
          CZAERO * (self.launchCg - self.cgFromNose) / self.mslRefDiam # ND

          self.CX = \
          self.CA0 + \
          self.CAA * self.alphaPrimeDeg + \
          self.CAD * (self.totalFinDeflDeg ** 2) + \
          self.CAOFF # ND

          self.CL = \
          self.CLLAP * (self.alphaPrimeDeg ** 2) * self.sinOfFourTimesPhiPrime + \
          self.CLLP * self.rollRateDeg * self.mslRefDiam / (2 * self.mslSpeed) + \
          self.CLLDP * self.rollDeflDeg # ND

          self.CY = CYAERO * self.cosPhiPrime - CZAERO * self.sinPhiPrime # ND
          self.CZ = CYAERO * self.sinPhiPrime + CZAERO * self.cosPhiPrime # ND
          self.CN = CMAERO * self.sinPhiPrime + CNAERO * self.cosPhiPrime # ND
          self.CM = CMAERO * self.cosPhiPrime + CNAERO * self.sinPhiPrime # ND

     def aerodynamicDerivatives(self):

          alphaPrimeDegLookUp = None
          if self.alphaPrimeDeg > self.alphaPrimeMax - 3:
                    alphaPrimeDegLookUp = self.alphaPrimeMax - 3
          else:
                    alphaPrimeDegLookUp = self.alphaPrimeDeg
          alphaPrimeDegMinusThree = alphaPrimeDegLookUp - 3 # DEGREES
          alphaPrimeDegPlusThree = alphaPrimeDegLookUp + 3 # DEGREES

          CN0MIN = self.lookUpValues["CN0"](self.mslMach, alphaPrimeDegMinusThree)[0] # ND
          CN0MAX = self.lookUpValues["CN0"](self.mslMach, alphaPrimeDegPlusThree)[0] # ND
          CLM0MIN = self.lookUpValues["CLM0"](self.mslMach, alphaPrimeDegMinusThree)[0] # ND
          CLM0MAX = self.lookUpValues["CLM0"](self.mslMach, alphaPrimeDegPlusThree)[0] # ND

          self.refAeroData["CNA"] = \
          (CN0MAX - CN0MIN) / \
          (alphaPrimeDegPlusThree - alphaPrimeDegMinusThree) # PER DEGREE

          self.refAeroData["CMA"] = \
          (CLM0MAX - CLM0MIN) / \
          (alphaPrimeDegPlusThree - alphaPrimeDegMinusThree) - \
          self.refAeroData["CNA"] * \
          (self.launchCg - self.cgFromNose) / self.mslRefDiam # PER DEGREE

          self.refAeroData["CND"] = self.CNDQ # PER DEGREE
          self.refAeroData["CMD"] = self.CLMDQ # PER DEGREE
          self.refAeroData["CMQ"] = self.CLMQ # PER DEGREE
          self.refAeroData["CLP"] = self.CLLP # PER DEGREE
          self.refAeroData["CLD"] = self.CLLDP # PER DEGREE
          self.staticMargin = -1 * self.refAeroData["CMA"] / self.refAeroData["CNA"]

     def missileMotion(self):

          # DERIVATIVES.
          axialForce = \
          self.thrust - \
          self.CX * self.Q * self.mslRefArea + \
          (self.gravBodyVec[0] * self.mass) # NEWTONS

          sideForce = \
          self.CY * self.Q * self.mslRefArea + \
          (self.gravBodyVec[1] * self.mass) # NEWTONS

          normalForce = \
          self.CZ * self.Q * self.mslRefArea + \
          (self.gravBodyVec[2] * self.mass) # NEWTONS

          rollMoment = self.CL * self.Q * self.mslRefArea * self.mslRefDiam # NEWTON * M
          pitchMoment = self.CM * self.Q * self.mslRefArea * self.mslRefDiam # NEWTON * M
          yawMoment = self.CN * self.Q * self.mslRefArea * self.mslRefDiam # NEWTON * M

          self.mslSpecificForce[0] = \
          axialForce / self.mass - \
          (self.mslRate[1] * self.mslVelB[2] - self.mslRate[2] * self.mslVelB[1]) # M/S^2

          self.mslSpecificForce[1] = \
          sideForce / self.mass - \
          (self.mslRate[2] * self.mslVelB[0] - self.mslRate[0] * self.mslVelB[2]) # M/S^2

          self.mslSpecificForce[2] = \
          normalForce / self.mass - \
          (self.mslRate[0] * self.mslVelB[1] - self.mslRate[1] * self.mslVelB[0]) # M/S^2

          newRollRateDot = rollMoment / self.amoi # RADIANS PER SECOND^2

          newPitchRateDot = \
          (1 / self.tmoi) * \
          ((self.tmoi - self.amoi) * self.mslRate[0] * \
          self.mslRate[2] + pitchMoment) # RADIAN PER SECOND^2

          newYawRateDot = \
          (1 / self.tmoi) * \
          ((self.amoi - self.tmoi) * self.mslRate[0] * \
          self.mslRate[1] + yawMoment) # RADIAN PER SECOND^2

          newPhiDot = \
          self.mslRate[0] + \
          (self.mslRate[1] * np.sin(self.mslEulerEnu[0]) + \
          self.mslRate[2] * np.cos(self.mslEulerEnu[0])) * np.tan(self.mslEulerEnu[1])

          newThetaDot = \
          self.mslRate[1] * np.cos(self.mslEulerEnu[0]) - \
          self.mslRate[2] * np.sin(self.mslEulerEnu[0])

          newPsiDot = \
          -1 * (self.mslRate[1] * np.sin(self.mslEulerEnu[0]) + \
          self.mslRate[2] * np.cos(self.mslEulerEnu[0])) / np.cos(self.mslEulerEnu[1])

          self.mslEulerDotEnu = npa([newPhiDot, newThetaDot, newPsiDot])
          self.mslRateDot = npa([newRollRateDot, newPitchRateDot, newYawRateDot])
          self.mslAccEnu = self.mslSpecificForce @ self.mslENUtoFLU # METERS PER SECOND^2

          # UPDATE TIME OF FLIGHT.
          self.mslTof += self.timeStep # SECONDS

          # STATE.
          deltaEuler = self.mslEulerDotEnu * self.timeStep
          deltaOmega = self.mslRateDot * self.timeStep
          deltaPos = self.mslVelEnu * self.timeStep # METERS
          deltaVel = self.mslAccEnu * self.timeStep # METERS PER SECOND
          self.mslEulerEnu += deltaEuler
          self.mslRate += deltaOmega
          self.mslPosEnu += deltaPos # METERS
          self.mslVelEnu += deltaVel # METERS PER SECOND
          self.mslRange += la.norm(deltaPos) # METERS

          # ATTITUDE.
          self.mslENUtoFLU = ORIENTATION_TO_LOCAL_TM(
               self.mslEulerEnu[0],
               -self.mslEulerEnu[1],
               self.mslEulerEnu[2]
          )
          self.mslVelB = self.mslENUtoFLU @ self.mslVelEnu # METERS PER SECOND
          self.mslSpeed = la.norm(self.mslVelB) # METERS PER SECOND
          self.mslAlpha = -1 * np.arctan2(self.mslVelB[2], self.mslVelB[0]) # RADIANS
          self.mslBeta = np.arctan2(self.mslVelB[1], self.mslVelB[0])

     def logData(self):
          self.STATE = self.populateState()
          lf.writeData(self.STATE, self.logFile)

     def endCheck(self):

          self.missDistance = la.norm(self.forwardLeftUpMslToInterceptRelPos)

          if self.mslPosEnu[2] < 0.0:
               self.lethality = endChecks.groundCollision
               self.go = False
          elif self.missDistance < 5.0:
               self.lethality = endChecks.intercept
               self.go = False
          elif self.forwardLeftUpMslToInterceptRelPos[0] < 0.0:
               self.lethality = endChecks.pointOfClosestApproachPassed
               self.go = False
          elif np.isnan(np.sum(self.mslPosEnu)):
               self.lethality = endChecks.notANumber
               self.go = False
          elif self.mslTof > self.maxTime:
               self.lethality = endChecks.maxTimeExceeded
               self.go = False
          elif self.lethality == endChecks.forcedSimTermination:
               self.go = False

     def fly(self):
          self.target()
          self.atmosphere()
          self.seeker()
          self.guidance()
          self.control()
          self.actuators()
          self.angles()
          self.dataLookUp()
          self.maneuveringLimit()
          self.propulsion()
          self.aerodynamics()
          self.aerodynamicDerivatives()
          self.missileMotion()
          self.logData()
          self.endCheck()

     def main(self):
          while self.go:
               self.fly()
               if round(self.mslTof, 3).is_integer():
                    print(f"TIME {self.mslTof:.3f} : ENU {self.mslPosEnu}")
          wallClockEnd = time.time()
          print(f"TIME {self.mslTof:.3f} : ENU {self.mslPosEnu}")
          print(f"SIMULATION RESULT : {self.lethality.name}")
          print(f"MISS DISTANCE : {self.missDistance:.4f}")
          print(f"SIMULATION RUN TIME : {wallClockEnd - self.wallClockStart} SECONDS")



if __name__ == "__main__":
     np.set_printoptions(suppress=True, precision=4)
     x = sixDofSim()
     x.main()