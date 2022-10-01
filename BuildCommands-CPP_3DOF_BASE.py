import os
import shutil

try:
	shutil.rmtree("3DOFS/CPP_3DOF_BASE/build")
except:
	print("No existing build directory. Creating.")
os.mkdir("3DOFS/CPP_3DOF_BASE/build")
os.chdir("3DOFS/CPP_3DOF_BASE/build")
os.system("cmake ../")
os.system("make")