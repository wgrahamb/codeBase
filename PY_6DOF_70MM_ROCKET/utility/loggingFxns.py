# Input.
# STATE = Dictionary.
# LOGFILE = open(file)

def writeHeader(STATE, LOGFILE):

	DATAPOINT = ""
	COUNT = len(STATE.keys())
	for index, key in enumerate(STATE.keys()):
		if index == COUNT - 1:
			DATAPOINT += f"{key}\n"
		else:
			DATAPOINT += f"{key} "
	LOGFILE.write(DATAPOINT)

def writeData(STATE, LOGFILE):

	DATAPOINT = ""
	COUNT = len(STATE.keys())
	for index, key in enumerate(STATE.keys()):
		VALUE = STATE[f"{key}"]
		if index == COUNT - 1:
			DATAPOINT += f"{VALUE:.4f}\n"
		else:
			DATAPOINT += f"{VALUE:.4f} "
	LOGFILE.write(DATAPOINT)