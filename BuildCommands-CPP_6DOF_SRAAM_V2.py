import os
import shutil

try:
	shutil.rmtree("CPP_6DOF_SRAAM_V2/build")
except:
	print("No existing build directory. Creating.")
os.mkdir("CPP_6DOF_SRAAM_V2/build")
os.chdir("CPP_6DOF_SRAAM_V2/build")
os.system("cmake ../")
os.system("make")