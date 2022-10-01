import os
import pickle

def loadpickle(filePath):
    ret = pickle.load(open(r"{}".format(filePath), "rb"))
    return ret