import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
from matPlotLibColors import matPlotLibColors

directory = "3DOFS/CPP_3DOF_PIP_SELECTION/output"
dfs = []

for f in os.listdir(directory):
	path = f"{directory}/{f}"
	df = pd.read_csv(open(r"{}".format(path)), delimiter= " ")
	df.name = f
	dfs.append(df)

startIndex = 0
stopIndex = -1

xs = []
ys = []
zs = []

for index, df in enumerate(dfs):
	xs += list(df.iloc[startIndex:stopIndex]["posE"])
	ys += list(df.iloc[startIndex:stopIndex]["posN"])
	zs += list(df.iloc[startIndex:stopIndex]["posU"])
	xs += list(df.iloc[startIndex:stopIndex]["tgtE"])
	ys += list(df.iloc[startIndex:stopIndex]["tgtN"])
	zs += list(df.iloc[startIndex:stopIndex]["tgtU"])

xMin = min(xs)
xMax = max(xs)
yMin = min(ys)
yMax = max(ys)
zMin = min(zs)
zMax = max(zs)

fig = plt.figure()
trajectories = fig.add_subplot(111, projection="3d")
trajectories.set_box_aspect(
	(
		np.ptp([xMin - 1000, xMax + 1000]), 
		np.ptp([yMin - 1000, yMax + 1000]), 
		np.ptp([zMin, zMax + 1000]),
	)
)
trajectories.set_xlim([xMin - 1000, xMax + 1000])
trajectories.set_ylim([yMin - 1000, yMax + 1000])
trajectories.set_zlim([zMin, zMax + 1000])

colors = matPlotLibColors()
for index, df in enumerate(dfs):
	trajectories.plot(
		df.iloc[startIndex:stopIndex]["posE"],
		df.iloc[startIndex:stopIndex]["posN"],
		df.iloc[startIndex:stopIndex]["posU"],
		color=colors[index]
	)
	trajectories.scatter(
		df.iloc[stopIndex]["tgtE"],
		df.iloc[stopIndex]["tgtN"],
		df.iloc[stopIndex]["tgtU"],
		color=colors[index],
		marker="2",
		label=df.name,
		s=80
	)

plt.legend(fontsize = "x-small")
plt.show()