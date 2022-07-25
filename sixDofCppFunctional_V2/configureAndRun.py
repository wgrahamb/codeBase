import subprocess
import shutil
import os

process = subprocess.Popen(["./build/missileModel"])
process.wait()