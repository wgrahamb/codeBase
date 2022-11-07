import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import random as rd
import copy

matplotlib.use("WebAgg")
rd.seed(101001)

"""

reference link: https://www.kalmanfilter.net/default.aspx 

"""

###

"""

Example One.

Use an alpha filter on measurements of mass
of a gold bar to get an estimate of its 
true weight.

"""

def exampleOne():

    # measuring a gold bar (static dynamic system) [fun oxymoron!]
    trueMass           = 1010
    initMassGuess      = 1000 # grams
    filter             = initMassGuess
    loopCount          = 0
    data               = {"INDEX":[], "MEASUREMENT":[], "TRUTH": [], "FILTER": []}

    while True:

        # iterate loop
        loopCount   += 1

        # this is a static dynamic system, next state is equal to current state
        estimate    = copy.deepcopy(filter)

        # take a measurement based on truth and a sigma of ten
        measurement = rd.gauss(trueMass, 10)

        # calculate alpha
        alpha       = 1 / loopCount

        # state update equation
        filter      = estimate + alpha * (measurement - estimate)

        # report
        print(f"Loop          {loopCount}")
        print(f"Estimate      {estimate:.2f}")
        print(f"Measurement   {measurement:.2f}")
        print(f"Alpha         {alpha:.2f}")
        print(f"Filter        {filter:.2f}")
        print(f"Truth         {trueMass:.2f}")
        print("\n")

        # data
        data["INDEX"].append(loopCount)
        data["MEASUREMENT"].append(measurement)
        data["TRUTH"].append(trueMass)
        data["FILTER"].append(filter)

        # break criteria
        if loopCount == 50:
            break

    plt.plot(data["INDEX"], data["TRUTH"], label="TRUTH")
    plt.plot(data["INDEX"], data["MEASUREMENT"], label="MEASUREMENT")
    plt.plot(data["INDEX"], data["FILTER"], label="FILTER")
    plt.legend(fontsize="xx-small")
    plt.show()

"""

Example Two.

Use an alpha-beta filter on measurements of 
range to an aircraft with a constant speed
to estimate its true range and true speed.
The measurements simulate a very simple
radar.

"""

def exampleTwo():

    # constants
    alpha        = 0.2   # nd
    beta         = 0.1   # nd

    # sim
    time         = 0.0   # seconds
    timeStep     = 5     # seconds
    maxTime      = 50    # seconds

    # first prediction (n=1)
    filteredRng  = 30000 # m
    filteredSpd  = 40    # m/s

    # set truth
    trueRng      = 30000 # meters
    trueSpd      = 40    # meters per second

    # data
    data = {
        "TIME":     [],
        "MEAS_RNG": [],
        "FILT_RNG": [],
        "FILT_SPD": [],
        "TRUE_RNG": [],
        "TRUE_SPD": []
    }

    while True:

        # update truth
        trueRng += trueSpd * timeStep
        trueSpd = trueSpd

        # catch up to trueh
        estimatedRng = filteredRng + filteredSpd * timeStep
        estimatedSpd = filteredSpd

        # make a range measurement with a sigma == 10 meters
        measuredRng = trueRng + rd.gauss(0, 100)

        # filter
        filteredRng = estimatedRng + alpha * (measuredRng - estimatedRng)
        filteredSpd = estimatedSpd + beta * \
            ((measuredRng - estimatedRng) / timeStep)

        # data
        data["TIME"].append(time)
        data["MEAS_RNG"].append(measuredRng)
        data["FILT_RNG"].append(filteredRng)
        data["FILT_SPD"].append(filteredSpd)
        data["TRUE_RNG"].append(trueRng)
        data["TRUE_SPD"].append(trueSpd)

        # update time
        time += timeStep

        if time > maxTime:
            break

    plt.plot(data["TIME"], data["MEAS_RNG"], label="MEAS_RNG")
    plt.plot(data["TIME"], data["FILT_RNG"], label="FILT_RNG")
    plt.plot(data["TIME"], data["TRUE_RNG"], label="TRUE_RNG")
    plt.legend(fontsize="xx-small")
    plt.show()

"""

Example Three.

Use an alpha-beta filter on measurements of 
range to an aircraft with a constant speed
of 50 m/s for 15 seconds and then accelerating
constantly at 8 m/s^2 for 35 more seconds
to estimate its true range and true speed.
The measurements simulate a very simple
radar.

"""

def exampleThree():

    # constants
    alpha        = 0.2 # nd
    beta         = 0.1 # nd

    # sim
    timeStep     = 5  # seconds
    time         = 0  # seconds
    maxTime      = 50 # seconds

    # set truth
    maneuver     = 15    # seconds
    acc1         = 0.0   # m/s^2
    acc2         = 8.0   # m/s^2
    trueRng      = 30000 # m
    trueSpd      = 50    # m/s
    trueAcc      = None  # m/s^2

    # set filter
    filteredRng  = 30000 # m
    filteredSpd  = 50    # m/s
    filteredAcc  = 0.0   # m/s^2

    # data
    data = {
        "TIME":     [],
        "MEAS_RNG": [],
        "FILT_RNG": [],
        "FILT_SPD": [],
        "FILT_ACC": [],
        "TRUE_RNG": [],
        "TRUE_SPD": [],
        "TRUE_ACC": []
    }

    while True:

        # update truth
        if time < maneuver:
            trueAcc  = acc1
            trueRng  += trueSpd * timeStep
            trueSpd  += trueAcc * timeStep
        else:
            trueAcc  = acc2
            trueRng  += trueSpd * timeStep
            trueSpd  += trueAcc * timeStep

        # catch up to truth
        predictedRng = filteredRng + filteredSpd * timeStep
        predictedSpd = filteredSpd + filteredAcc * timeStep
        predictedAcc = filteredAcc

        # make a range measurement with a sigma == 100 meters
        measuredRng  = trueRng + rd.gauss(0, 1000)

        # filter
        filteredRng  = predictedRng + alpha * (measuredRng - predictedRng)
        filteredSpd  = predictedSpd + beta  * \
            ((measuredRng - predictedRng) / timeStep)
        filteredAcc  = predictedAcc

        # update time
        time         += timeStep

        # data
        data["TIME"].append(time)
        data["MEAS_RNG"].append(measuredRng)
        data["FILT_RNG"].append(predictedRng)
        data["FILT_SPD"].append(predictedSpd)
        data["FILT_ACC"].append(predictedAcc)
        data["TRUE_RNG"].append(trueRng)
        data["TRUE_SPD"].append(trueSpd)
        data["TRUE_ACC"].append(trueAcc)

        # end check
        if time > maxTime:
            break

    fig = plt.figure()
    pos = fig.add_subplot(131)
    pos.plot(data["TIME"], data["MEAS_RNG"], label="MEAS_RNG")
    pos.plot(data["TIME"], data["FILT_RNG"], label="FILT_RNG")
    pos.plot(data["TIME"], data["TRUE_RNG"], label="TRUE_RNG")
    pos.legend(fontsize="xx-small")
    vel = fig.add_subplot(132)
    vel.plot(data["TIME"], data["FILT_SPD"], label="FILT_SPD")
    vel.plot(data["TIME"], data["TRUE_SPD"], label="TRUE_SPD")
    vel.legend(fontsize="xx-small")
    acc = fig.add_subplot(133)
    acc.plot(data["TIME"], data["FILT_ACC"], label="FILT_ACC")
    acc.plot(data["TIME"], data["TRUE_ACC"], label="TRUE_ACC")
    acc.legend(fontsize="xx-small")
    plt.show()

"""

Example Four.

Use an alpha-beta-gamma filter on measurements
of range to an aircraft with a constant speed
of 50 m/s for 15 seconds and then accelerating
constantly at 8 m/s^2 for 35 more seconds
to estimate its true range and true speed.
The measurements simulate a very simple
radar.

"""

def exampleFour():

    # constants
    alpha        = 0.5 # nd
    beta         = 0.4 # nd
    gamma        = 0.1 # nd

    # sim
    timeStep     = 5  # seconds
    time         = 0  # seconds
    maxTime      = 50 # seconds

    # set truth
    maneuver     = 15    # seconds
    acc1         = 0.0   # m/s^2
    acc2         = 8.0   # m/s^2
    trueRng      = 30000 # m
    trueSpd      = 50    # m/s
    trueAcc      = None  # m/s^2

    # set filter
    filteredRng  = 30000 # m
    filteredSpd  = 50    # m/s
    filteredAcc  = 0.0   # m/s^2

    # data
    data = {
        "TIME":     [],
        "MEAS_RNG": [],
        "FILT_RNG": [],
        "FILT_SPD": [],
        "FILT_ACC": [],
        "TRUE_RNG": [],
        "TRUE_SPD": [],
        "TRUE_ACC": []
    }

    while True:

        # update truth
        if time < maneuver:
            trueAcc  = acc1
            trueRng  += trueSpd * timeStep
            trueSpd  += trueAcc * timeStep
        else:
            trueAcc  = acc2
            trueRng  += trueSpd * timeStep
            trueSpd  += trueAcc * timeStep

        # catch up to truth
        predictedRng = filteredRng + filteredSpd * timeStep
        predictedSpd = filteredSpd + filteredAcc * timeStep
        predictedAcc = filteredAcc

        # make a range measurement with a sigma == 100 meters
        measuredRng  = trueRng + rd.gauss(0, 100)

        # filter
        filteredRng  = predictedRng + alpha * (measuredRng - predictedRng)
        filteredSpd  = predictedSpd + beta  * \
            ((measuredRng - predictedRng) / timeStep)
        filteredAcc  = predictedAcc + gamma * \
            ((measuredRng - predictedRng) / (0.5 * timeStep * timeStep))

        # update time
        time         += timeStep

        # data
        data    ["TIME"].append(time)
        data["MEAS_RNG"].append(measuredRng)
        data["FILT_RNG"].append(predictedRng)
        data["FILT_SPD"].append(predictedSpd)
        data["FILT_ACC"].append(predictedAcc)
        data["TRUE_RNG"].append(trueRng)
        data["TRUE_SPD"].append(trueSpd)
        data["TRUE_ACC"].append(trueAcc)

        # end check
        if time > maxTime:
            break

    fig = plt.figure()
    pos = fig.add_subplot(131)
    vel = fig.add_subplot(132)
    acc = fig.add_subplot(133)

    pos.plot(data["TIME"], data["MEAS_RNG"], label="MEAS_RNG")
    pos.plot(data["TIME"], data["FILT_RNG"], label="FILT_RNG")
    pos.plot(data["TIME"], data["TRUE_RNG"], label="TRUE_RNG")
    pos.legend(fontsize="xx-small")

    vel.plot(data["TIME"], data["FILT_SPD"], label="FILT_SPD")
    vel.plot(data["TIME"], data["TRUE_SPD"], label="TRUE_SPD")
    vel.legend(fontsize="xx-small")

    acc.plot(data["TIME"], data["FILT_ACC"], label="FILT_ACC")
    acc.plot(data["TIME"], data["TRUE_ACC"], label="TRUE_ACC")
    acc.legend(fontsize="xx-small")
    plt.show()

"""

Example Five.

Use a one dimensional kalman filter on measurements
of a building with constant height == 50 m.

"""

def exampleFive():
    trueHgt    = 50 # m, true height of the building
    sigmaMeas  = 5  # m, standard deviation in measurements
    listMeas   = [  # m, set list of measurements rather than draws
        48.54,
        47.11,
        55.01,
        55.15,
        49.89,
        40.85,
        46.72,
        50.05,
        51.27,
        49.95
    ]
    filtHgt    = 60      # m, initial filtered height of the building
    filtUncert = 15 ** 2 # m, initial filtered uncertainty
    data       = {
        "INDEX":    [],
        "MEAS_HGT": [],
        "TRUE_HGT": [],
        "FILT_HGT": []
    }
    for index, m in enumerate(listMeas):
        predHgt     = filtHgt
        predUncert  = filtUncert
        kalmanGain  = predUncert / (predUncert + sigmaMeas ** 2)
        filtHgt     = predHgt + kalmanGain * (m - predHgt)
        filtUncert  = (1 - kalmanGain) * predUncert
        data["INDEX"]   .append(index)
        data["MEAS_HGT"].append(m)
        data["TRUE_HGT"].append(trueHgt)
        data["FILT_HGT"].append(filtHgt)
    plt.plot(data["INDEX"], data["TRUE_HGT"], label="TRUE")
    plt.plot(data["INDEX"], data["MEAS_HGT"], label="MEAS")
    plt.plot(data["INDEX"], data["FILT_HGT"], label="FILT")
    plt.legend(fontsize="xx-small")
    plt.show()



if __name__ == "__main__":
    # exampleOne()
    # exampleTwo()
    # exampleThree()
    # exampleFour()
    exampleFive()
























































































