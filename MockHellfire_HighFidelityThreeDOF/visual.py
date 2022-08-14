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
trajectory = fig.add_subplot(121, projection="3d")
trajectory.set_title("Trajectory")
trajectory.set_xlabel("RANGE (M)")
trajectory.set_zlabel("ALT (M)")

xs = []
zs = []

for index, df in enumerate(dfs):
	xs += list(df.iloc[startIndex:stopIndex]["RNG"])
	zs += list(df.iloc[startIndex:stopIndex]["ALT"])
	trajectory.plot(df.iloc[startIndex:stopIndex]["RNG"], np.zeros(len(df.iloc[startIndex:stopIndex]["RNG"])), df.iloc[startIndex:stopIndex]["ALT"], label=df.name, color=colors.pop(0), alpha = 0.5)

xMin = min(xs)
xMax = max(xs)
zMin = min(zs)
zMax = max(zs)
trajectory.set_box_aspect(
	(
		np.ptp([xMin - 1000, xMax + 1000]), 
		np.ptp([0 - 1000, 0 + 1000]), 
		np.ptp([zMin, zMax + 1000]),
	)
)
trajectory.set_xlim([xMin - 1000, xMax + 1000])
trajectory.set_ylim([0 - 1000, 0 + 1000])
trajectory.set_zlim([zMin, zMax + 1000])

trajectory.legend(fontsize="xx-small")

# Performance.
performance = fig.add_subplot(122)
performance.set_title("PITCH")
performance.set_xlabel("TIME OF FLIGHT (S)")
performance.set_ylim([-100, 100])
for index, df in enumerate(dfs):
	performance.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["THT_DEG"], label=df.name + "_THT_DEG", color=colors.pop(0), alpha = 0.5)
for index, df in enumerate(dfs):
	performance.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["THT_DOT_DEG"], label=df.name + "_THT_DOT_DEG", color=colors.pop(0), alpha = 0.5)
for index, df in enumerate(dfs):
	performance.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["AOA_DEG"], label=df.name + "_AOA_DEG", color=colors.pop(0), alpha = 0.5)
for index, df in enumerate(dfs):
	performance.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["W_DOT"], label=df.name + "_NORM_ACC_M/S^2", color=colors.pop(0), alpha = 0.5)
performance.legend(fontsize="xx-small")

# Show plot.
plt.show()



