import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from utility.matPlotLibColors import matPlotLibColors
import os

directory = "MockHellfire_HighFidelityThreeDOF/output"
dfs = []

for f in os.listdir(directory):
	path = f"{directory}/{f}"
	df = pd.read_csv(open(r"{}".format(path)), delimiter= " ")
	df.name = f
	dfs.append(df)

for index, header in enumerate(df.columns):
	print(index, header)

colors = matPlotLibColors()

fig = plt.figure(figsize=(20, 20))
startIndex = 0
stopIndex = -1

# Trajectory.
trajectory = fig.add_subplot(231)
trajectory.set_title("Trajectory")
trajectory.set_xlabel("RANGE (M)")
trajectory.set_ylabel("ALT (M)")
for index, df in enumerate(dfs):
	trajectory.plot(df.iloc[startIndex:stopIndex]["RNG"], df.iloc[startIndex:stopIndex]["ALT"], label=df.name, color=colors[index], alpha = 0.5)
trajectory.legend(fontsize="xx-small")

# Theta.
theta = fig.add_subplot(232)
theta.set_title("THETA DEGREES")
theta.set_xlabel("TIME OF FLIGHT (S)")
for index, df in enumerate(dfs):
	theta.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["THT_DEG"], label=df.name, color=colors[index], alpha = 0.5)
theta.legend(fontsize="xx-small")

# Theta dot.
thetaDot = fig.add_subplot(233)
thetaDot.set_title("THETA DOT DEGREES")
thetaDot.set_xlabel("TIME OF FLIGHT (S)")
for index, df in enumerate(dfs):
	thetaDot.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["THT_DOT_DEG"], label=df.name, color=colors[index], alpha = 0.5)
thetaDot.legend(fontsize="xx-small")

# Normal accel.
normalAccel = fig.add_subplot(234)
normalAccel.set_title("NORMAL ACCEL M/S^2")
normalAccel.set_xlabel("TIME OF FLIGHT (S)")
for index, df in enumerate(dfs):
	normalAccel.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["W_DOT"], label=df.name, color=colors[index], alpha = 0.5)
normalAccel.legend(fontsize="xx-small")

# Alpha.
alpha = fig.add_subplot(235)
alpha.set_title("ALPHA DEGREES")
alpha.set_xlabel("TIME OF FLIGHT (S)")
for index, df in enumerate(dfs):
	alpha.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["AOA_DEG"], label=df.name, color=colors[index], alpha = 0.5)
alpha.legend(fontsize="xx-small")

# Alpha dot.
alphaDot = fig.add_subplot(236)
alphaDot.set_title("ALPHA DOT DEGREES")
alphaDot.set_xlabel("TIME OF FLIGHT (S)")
for index, df in enumerate(dfs):
	alphaDot.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["AOA_DOT_DEG"], label=df.name, color=colors[index], alpha = 0.5)
alphaDot.legend(fontsize="xx-small")

# Show plot.
plt.show()















