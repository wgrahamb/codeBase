import subprocess
import shutil
import os

for index, f in enumerate(os.listdir("output")):
	os.remove(f"output/{f}")

process = subprocess.Popen(["./build/NOVICE"])
process.wait()

# for index, f in enumerate(os.listdir("/mnt/c/Users/graha/iCloudDrive/windowsCode/NOVICE_Visuals/output")):
# 	os.remove(f"/mnt/c/Users/graha/iCloudDrive/windowsCode/NOVICE_Visuals/output/{f}")
# shutil.rmtree("/mnt/c/Users/graha/iCloudDrive/windowsCode/NOVICE_Visuals/output")
# shutil.copytree(
# 	"/home/graham/codeBase/NOVICE/output",
# 	"/mnt/c/Users/graha/iCloudDrive/windowsCode/NOVICE_Visuals/output"
# )