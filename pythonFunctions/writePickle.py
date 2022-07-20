import os
import pickle

def writepickle(d, filePath):
    dirname = os.getcwd()
    with open(r"{}".format(filePath), "wb") as f:
        pickle.dump(d, f)