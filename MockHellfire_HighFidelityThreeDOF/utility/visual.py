import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

f1 = r"MockHellfireAirframeLinearization/threeDofHellfire.txt"

viewFile = f1

df = pd.read_csv(open(f"{viewFile}"), delimiter=" ")

for index, header in enumerate(df.columns):
	print(index, header)

startIndex = 1
stopIndex = -1

fig = plt.figure(figsize=(20, 20))

# Trajectory
scale = True
trajectory = fig.add_subplot(121, projection="3d")
trajectory.set_title("Trajectory")
trajectory.set_xlabel("East")
trajectory.set_ylabel("North")
trajectory.set_zlabel("Up")
if scale:
	xMin = min(list(df.iloc[startIndex:stopIndex]["posE"]) + list(df.iloc[startIndex:stopIndex]["tgtE"]))
	xMax = max(list(df.iloc[startIndex:stopIndex]["posE"]) + list(df.iloc[startIndex:stopIndex]["tgtE"]))
	yMin = min(list(df.iloc[startIndex:stopIndex]["posN"]) + list(df.iloc[startIndex:stopIndex]["tgtN"]))
	yMax = max(list(df.iloc[startIndex:stopIndex]["posN"]) + list(df.iloc[startIndex:stopIndex]["tgtN"]))
	zMin = min(list(df.iloc[startIndex:stopIndex]["posU"]) + list(df.iloc[startIndex:stopIndex]["tgtU"]))
	zMax = max(list(df.iloc[startIndex:stopIndex]["posU"]) + list(df.iloc[startIndex:stopIndex]["tgtU"]))
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
trajectory.plot(df.iloc[startIndex:stopIndex]["posE"], df.iloc[startIndex:stopIndex]["posN"], df.iloc[startIndex:stopIndex]["posU"], color="b")
if scale:
	trajectory.scatter(df.iloc[stopIndex]["tgtE"], df.iloc[stopIndex]["tgtN"], df.iloc[stopIndex]["tgtU"], color="r")

# Mach, Axial Acceleration, Speed.
performance = fig.add_subplot(122)
performance.set_title("Performance")
performance.set_xlabel("Time of Flight")
performance.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["mach"], color="b", label="Mach Speed")
performance.plot(df.iloc[startIndex:stopIndex]["tof"], 0.01 * df.iloc[startIndex:stopIndex]["specificForceX"], color="r", label="1% Axial Acceleration M/S^2")
performance.plot(df.iloc[startIndex:stopIndex]["tof"], 0.01 * df.iloc[startIndex:stopIndex]["speed"], color="g", label="1% Speed M/S")
performance.legend(fontsize="xx-small")

fig.subplots_adjust(top=0.9, bottom=0.1, left=0.1, hspace=0.4)
plt.show()
