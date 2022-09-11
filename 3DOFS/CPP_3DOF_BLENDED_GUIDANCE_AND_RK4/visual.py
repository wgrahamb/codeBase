import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

f1 = r"output.txt"

viewFile = f1

df = pd.read_csv(open(f"{viewFile}"), delimiter=" ")

for index, header in enumerate(df.columns):
	print(index, header)

startIndex = 1
stopIndex = -1

fig = plt.figure(figsize=(20, 20))

# Trajectory
trajectory = fig.add_subplot(111, projection="3d")
trajectory.set_title("Trajectory")
trajectory.set_xlabel("East")
trajectory.set_ylabel("North")
trajectory.set_zlabel("Up")
xMin = min(list(df.iloc[startIndex:stopIndex]["EAST"]) + list(df.iloc[startIndex:stopIndex]["tgtE"]))
xMax = max(list(df.iloc[startIndex:stopIndex]["EAST"]) + list(df.iloc[startIndex:stopIndex]["tgtE"]))
yMin = min(list(df.iloc[startIndex:stopIndex]["NORTH"]) + list(df.iloc[startIndex:stopIndex]["tgtN"]))
yMax = max(list(df.iloc[startIndex:stopIndex]["NORTH"]) + list(df.iloc[startIndex:stopIndex]["tgtN"]))
zMin = min(list(df.iloc[startIndex:stopIndex]["UP"]) + list(df.iloc[startIndex:stopIndex]["tgtU"]))
zMax = max(list(df.iloc[startIndex:stopIndex]["UP"]) + list(df.iloc[startIndex:stopIndex]["tgtU"]))
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
trajectory.plot(df.iloc[startIndex:stopIndex]["EAST"], df.iloc[startIndex:stopIndex]["NORTH"], df.iloc[startIndex:stopIndex]["UP"], color="b")
trajectory.scatter(df.iloc[stopIndex]["tgtE"], df.iloc[stopIndex]["tgtN"], df.iloc[stopIndex]["tgtU"], color="r")

plt.show()