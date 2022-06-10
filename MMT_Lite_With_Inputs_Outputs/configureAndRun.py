import subprocess
import shutil
import os

### RUN THIS SCRIPT AT THE TOP LEVEL OF THE PROJECT TO SIMULATE ONE RUN

inPutValues = {

	### EULER ANGLES RELEVANT TO LINE OF SIGHT TO TARGET I.E. IF PSI == 0 THEN THE MISSILE IS POINTED STRAIGHT AT THE TARGET, AT LEAST ON THE HORIZONTAL PLANE ###
	"PHI": "0.0", # EULER ANGLE >>> ROLL >>> DEGREES
	"THETA": "45.0", # EULER ANGLE >>> ELEVATION >>> DEGREES
	"PSI": "-2.0", # EULER ANGLE >>> AZIMUTH >>> DEGREES

	### GEODETIC ORIGIN COORDINATES AND LAUNCH POSITION ###
	"LAT_ORIGIN": "38.8719", # DEGREES
	"LON_ORIGIN": "77.0563", # DEGREES
	"ALT_ORIGIN": "0.0", # METERS

	### WAYPOINT PARAMETERS ###
	"TGT_RANGE": "5000.0", # METERS
	"TGT_AZIMUTH": "0.0", # MEASURED FROM TRUE NORTH >>> DEGREES
	"TGT_ALTITUDE": "2000.0", # METERS

	### LINE OF ATTACK ###
	"GAMMAY": "10.0", # SIDEWAYS TRAJECTORY SHAPING >>> DEGREES >>> MEASURED FROM LINE OF SIGHT >>> POSITIVE MEANS INCOMING FROM THE RIGHT
	"GAMMAP": "-15.0", # LOFT TRAJECTORY SHAPING >>> DEGREES >>> MEASURED FROM HORIZON >>> NEGATIVE MEANS THE HIGH GROUND

	### AERODYNAMICS ###
	"AERO_TYPE": "-1", # DETERMINES THE USE OF BASIC OR COMPLEX AERODYNAMIC DATA, 1 == COMPLEX, -1 == BASIC

	### ACTUATORS ###
	"ACTUATOR_TYPE": "-1" # 1 IS THE HIGH FIDELITY ELECTRO CAS MODEL, -1 IS SECOND ORDER ACTUATORS

}

simFile = r"input/sim.dat"
navFile = r"input/navproc.dat"
guidanceFile = r"input/guidelaw.dat"
aeroFile = r"input/aero.dat"
actuatorFile = r"input/actuators.dat"

### SIM FILE
with open(simFile, "r") as file:
	data = file.readlines()

for lineIndex, line in enumerate(data):
	lineContent = line.split()
	newLine = None
	for wordIndex, word in enumerate(lineContent):
		if word == "phi0":
			newLine = ""
			lineContent[2] = inPutValues["PHI"]
			for n in lineContent:
				newLine += (n + " ")
			newLine += "\n"
		if word == "tht0":
			newLine = ""
			lineContent[2] = inPutValues["THETA"]
			for n in lineContent:
				newLine += (n + " ")
			newLine += "\n"
		if word == "psi0":
			newLine = ""
			lineContent[2] = inPutValues["PSI"]
			for n in lineContent:
				newLine += (n + " ")
			newLine += "\n"
	if newLine != None:
		data[lineIndex] = newLine

with open(simFile, "w") as file:
	file.writelines(data)

### NAV FILE
with open(navFile, "r") as file:
	data = file.readlines()

for lineIndex, line in enumerate(data):
	lineContent = line.split()
	newLine = None
	for wordIndex, word in enumerate(lineContent):
		if word == "orig_Lat":
			newLine = ""
			lineContent[2] = inPutValues["LAT_ORIGIN"]
			for n in lineContent:
				newLine += (n + " ")
			newLine += "\n"
		if word == "orig_Lon":
			newLine = ""
			lineContent[2] = inPutValues["LON_ORIGIN"]
			for n in lineContent:
				newLine += (n + " ")
			newLine += "\n"
		if word == "orig_Altg":
			newLine = ""
			lineContent[2] = inPutValues["ALT_ORIGIN"]
			for n in lineContent:
				newLine += (n + " ")
			newLine += "\n"
		if word == "tgt_Rng":
			newLine = ""
			lineContent[2] = inPutValues["TGT_RANGE"]
			for n in lineContent:
				newLine += (n + " ")
			newLine += "\n"
		if word == "tgt_Az":
			newLine = ""
			lineContent[2] = inPutValues["TGT_AZIMUTH"]
			for n in lineContent:
				newLine += (n + " ")
			newLine += "\n"
		if word == "tgt_Hgt":
			newLine = ""
			lineContent[2] = inPutValues["TGT_ALTITUDE"]
			for n in lineContent:
				newLine += (n + " ")
			newLine += "\n"
	if newLine != None:
		data[lineIndex] = newLine

with open(navFile, "w") as file:
	file.writelines(data)

### GUIDANCE FILE
with open(guidanceFile, "r") as file:
	data = file.readlines()

for lineIndex, line in enumerate(data):
	lineContent = line.split()
	newLine = None
	for wordIndex, word in enumerate(lineContent):
		if word == "gamY_T":
			newLine = ""
			lineContent[2] = inPutValues["GAMMAY"]
			for n in lineContent:
				newLine += (n + " ")
			newLine += "\n"
		if word == "gamP_T":
			newLine = ""
			lineContent[2] = inPutValues["GAMMAP"]
			for n in lineContent:
				newLine += (n + " ")
			newLine += "\n"
	if newLine != None:
		data[lineIndex] = newLine

with open(guidanceFile, "w") as file:
	file.writelines(data)

### AERO FILE
with open(aeroFile, "r") as file:
	data = file.readlines()

for lineIndex, line in enumerate(data):
	lineContent = line.split()
	newLine = None
	for wordIndex, word in enumerate(lineContent):
		if word == "aero_Flag":
			newLine = ""
			lineContent[2] = inPutValues["AERO_TYPE"]
			for n in lineContent:
				newLine += (n + " ")
			newLine += "\n"
	if newLine != None:
		data[lineIndex] = newLine

with open(aeroFile, "w") as file:
	file.writelines(data)

### ACTUATORS FILE
with open(actuatorFile, "r") as file:
	data = file.readlines()

for lineIndex, line in enumerate(data):
	lineContent = line.split()
	newLine = None
	for wordIndex, word in enumerate(lineContent):
		if word == "fidelityFlag":
			newLine = ""
			lineContent[2] = inPutValues["ACTUATOR_TYPE"]
			for n in lineContent:
				newLine += (n + " ")
			newLine += "\n"
	if newLine != None:
		data[lineIndex] = newLine

with open(actuatorFile, "w") as file:
	file.writelines(data)

process = subprocess.Popen(["./MMT_Lite"])
process.wait()

# SPECIFIC TO GRAHAM'S COMPUTER >>> YOU NEED TO INSERT YOUR OWN PATHS.
os.remove("/mnt/c/Users/graha/Documents/pythonRepo/pythonRepo/sixDofVisual/MMT_Lite.txt")
shutil.copy("/home/graham/mmt-service-box/MMT_Lite_With_Inputs_Outputs/output/MMT_Lite.dat", "/mnt/c/Users/graha/Documents/pythonRepo/pythonRepo/sixDofVisual/MMT_Lite.txt")