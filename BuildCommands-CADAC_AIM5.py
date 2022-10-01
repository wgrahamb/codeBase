import os
import shutil

try:
	shutil.rmtree("CADAC_SIMULATIONS/AIM5/build")
except:
	print("No existing build directory. Creating.")
os.mkdir("CADAC_SIMULATIONS/AIM5/build")
os.chdir("CADAC_SIMULATIONS/AIM5/build")
os.system("cmake ../")
os.system("make")