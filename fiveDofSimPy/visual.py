import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

df = pd.read_csv("log.txt", delimiter=" ")

startIndex = 1
stopIndex = -1

fig = plt.figure()

trajectory = fig.add_subplot(131, projection="3d")
trajectory.set_xlabel("EAST")
trajectory.set_ylabel("NORTH")
trajectory.set_zlabel("UP")
eMin = min(df.iloc[startIndex:stopIndex]["posEast(m)"])
eMax = max(df.iloc[startIndex:stopIndex]["posEast(m)"])
nMin = min(df.iloc[startIndex:stopIndex]["posNorth(m)"])
nMax = max(df.iloc[startIndex:stopIndex]["posNorth(m)"])
uMin = min(df.iloc[startIndex:stopIndex]["posUp(m)"])
uMax = max(df.iloc[startIndex:stopIndex]["posUp(m)"])
trajectory.set_box_aspect(
	(
		np.ptp([eMin, eMax]), 
		np.ptp([nMin, nMax]), 
		np.ptp([uMin, uMax]),
	)
)
trajectory.plot(
	df.iloc[startIndex:stopIndex]["posEast(m)"],
	df.iloc[startIndex:stopIndex]["posNorth(m)"],
	df.iloc[startIndex:stopIndex]["posUp(m)"],
	color="b", label="TRAJECTORY"
)
trajectory.scatter(
	df.iloc[stopIndex]["targetEast(m)"],
	df.iloc[stopIndex]["targetNorth(m)"],
	df.iloc[stopIndex]["targetUp(m)"], color="r", marker="x", label="TARGET"
)
trajectory.legend()

acc = fig.add_subplot(132)
acc.set_xlabel("TIME OF FLIGHT (SECONDS)")
acc.set_ylabel("METERS PER S^2")
acc.plot(df.iloc[startIndex:stopIndex]["timeOfFlight(s)"], df.iloc[startIndex:stopIndex]["normalAccCommand(m/s^2)"], color="r", label="NORMAL COMMAND")
acc.plot(df.iloc[startIndex:stopIndex]["timeOfFlight(s)"], df.iloc[startIndex:stopIndex]["normalAccAchieved(m/s^2)"], color="r", linestyle="dotted", label="NORMAL ACHIEVED")
acc.plot(df.iloc[startIndex:stopIndex]["timeOfFlight(s)"], df.iloc[startIndex:stopIndex]["sideAccCommand(m/s^2)"], color="b", label="SIDE COMMAND")
acc.plot(df.iloc[startIndex:stopIndex]["timeOfFlight(s)"], df.iloc[startIndex:stopIndex]["sideAccAchieved(m/s^2)"], color="b", linestyle="dotted", label="SIDE ACHIEVED")
acc.legend()

aeroAngles = fig.add_subplot(133)
aeroAngles.set_xlabel("TIME OF FLIGHT (SECONDS)")
aeroAngles.set_ylabel("RADIANS")
aeroAngles.plot(df.iloc[startIndex:stopIndex]["timeOfFlight(s)"], df.iloc[startIndex:stopIndex]["alpha(rads)"], color="r", label="ALPHA")
aeroAngles.plot(df.iloc[startIndex:stopIndex]["timeOfFlight(s)"], df.iloc[startIndex:stopIndex]["beta(rads)"], color="b", label="BETA")
aeroAngles.legend()

plt.get_current_fig_manager().full_screen_toggle()
plt.show()