import subprocess
import shutil
import os

for index, f in enumerate(os.listdir("output")):
	os.remove(f"output/{f}")

process = subprocess.Popen(["./build/NOVICE"])
process.wait()

moveToPath = "/mnt/c/Users/graha/Desktop/NOVICE_Visuals/output"

for index, f in enumerate(os.listdir(moveToPath)):
	os.remove(f"{moveToPath}/{f}")
shutil.rmtree(f"{moveToPath}")
shutil.copytree(
	"/home/graham/codeBase/NOVICE/output",
	moveToPath
)