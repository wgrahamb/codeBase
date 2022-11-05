import numpy as np
import pandas as pd
import matplotlib
import matplotlib.pyplot as plt
import random as rd
import copy

matplotlib.use("WebAgg")
rd.seed(101001)

def staticDynamicSystem():

    # measuring a gold bar (static dynamic system) [fun oxymoron!]
    trueWeight = 1010
    initialGuessWeight = 1000 # grams
    filter = initialGuessWeight
    loopCount = 0
    data = {"INDEX":[], "TRUTH": [], "ESTIMATE": [], "FILTER": []}
    while True:

        # iterate loop
        loopCount += 1

        # this is a static dynamic system, next state is equal to current state
        estimate = copy.deepcopy(filter)

        # take a measurement based on truth and a sigma of ten
        measurement = rd.gauss(trueWeight, 10)

        # calculate alpha
        alpha = 1 / loopCount

        # state update equation
        filter = estimate + alpha * \
            (measurement - estimate)

        # report
        print(f"Next Weight Estimate {estimate}")
        print(f"Alpha {alpha}")
        print(f"Filtered Weight Estimate {filter}")
        print(f"True Weight {trueWeight}")
        print("\n")

        # data
        data["INDEX"].append(loopCount)
        data["TRUTH"].append(trueWeight)
        data["ESTIMATE"].append(estimate)
        data["FILTER"].append(filter)

        # break criteria
        if loopCount == 10:
            break

    plt.plot(data["INDEX"], data["TRUTH"], label="TRUTH")
    plt.plot(data["INDEX"], data["ESTIMATE"], label="ESTIMATE")
    plt.plot(data["INDEX"], data["FILTER"], label="FILTER")
    plt.legend(fontsize="xx-small")
    plt.show()



if __name__ == "__main__":
    staticDynamicSystem()
























































































