import os
import shutil

try:
	shutil.rmtree("cppFunctions/build")
except:
	print("No existing build directory. Creating.")
os.mkdir("cppFunctions/build")
os.chdir("cppFunctions/build")
os.system("cmake ../")
os.system("make")