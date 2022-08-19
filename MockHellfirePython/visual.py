import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from utility.matPlotLibColors import matPlotLibColors

RAD_TO_DEG = 57.2957795130823

# Setup.
f1 = r"MockHellfirePython/data.txt"
viewFile = f1
df = pd.read_csv(open(f"{viewFile}"), delimiter=" ")
for index, header in enumerate(df.columns):
	print(index, header)
startIndex = 1
stopIndex = -1
colors = matPlotLibColors()
fig = plt.figure(figsize=(20, 20))

# Trajectory
scale = True
trajectory = fig.add_subplot(121, projection="3d")
trajectory.set_title("Trajectory")
trajectory.set_xlabel("East")
trajectory.set_ylabel("North")
trajectory.set_zlabel("Up")
if scale:
	xMin = min(list(df.iloc[startIndex:stopIndex]["POS_0X"]))
	xMax = max(list(df.iloc[startIndex:stopIndex]["POS_0X"]))
	yMin = 0
	yMax = 0
	zMin = min(list(df.iloc[startIndex:stopIndex]["POS_0Y"]))
	zMax = max(list(df.iloc[startIndex:stopIndex]["POS_0Y"]))
	trajectory.set_box_aspect(
		(
			np.ptp([xMin - 1000, xMax + 1000]), 
			np.ptp([yMin - 1000, yMax + 1000]), 
			np.ptp([zMin, zMax + 1000]),
		)
	)
	trajectory.set_xlim([xMin - 1000, xMax + 1000])
	trajectory.set_ylim([yMin - 1000, yMax + 1000])
	trajectory.set_zlim([zMin, zMax + 1000])
trajectory.plot(df.iloc[startIndex:stopIndex]["POS_0X"], np.linspace(0, 0.000000000001, len(df.iloc[startIndex:stopIndex]["POS_0X"])), df.iloc[startIndex:stopIndex]["POS_0Y"], color=colors.pop(0))

# Pitch.
pitch = fig.add_subplot(122)
pitch.set_title("Pitch")
pitch.set_xlabel("TOF")
pitch.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["ALPHA"] * RAD_TO_DEG, label="ALPHA - DEG", color=colors.pop(0))
pitch.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["T_0"] * RAD_TO_DEG, label="THETA - DEG", color=colors.pop(0))
pitch.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["TDOT_0"] * RAD_TO_DEG, label="THETA DOT - DEG", color=colors.pop(0))
pitch.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["WDOT_0"], label="NORMAL ACC - M/S^2", color=colors.pop(0))
pitch.legend(fontsize="small")

# Show.
plt.show()











