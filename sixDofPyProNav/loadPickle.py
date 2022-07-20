import os
import pickle

def loadpickle(filePath):
    dirname = os.getcwd()
    ret = pickle.load(open(r"{}".format(filePath), "rb"))
    return ret