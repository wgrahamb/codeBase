import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('WebAgg')
import numpy as np

df = pd.read_csv("PY_5DOF_AIM/log.txt", delimiter=" ")
startIndex = 1
stopIndex = -1
fig = plt.figure()

trajectory = fig.add_subplot(121, projection="3d")
trajectory.set_xlabel("EAST")
trajectory.set_ylabel("NORTH")
trajectory.set_zlabel("UP")
eMin = min(list(df.iloc[:stopIndex]["posE"]) + list(df.iloc[:stopIndex]["tgtE"]))
eMax = max(list(df.iloc[:stopIndex]["posE"]) + list(df.iloc[:stopIndex]["tgtE"]))
nMin = min(list(df.iloc[:stopIndex]["posN"]) + list(df.iloc[:stopIndex]["tgtN"]))
nMax = max(list(df.iloc[:stopIndex]["posN"]) + list(df.iloc[:stopIndex]["tgtN"]))
uMin = min(list(df.iloc[:stopIndex]["posU"]) + list(df.iloc[:stopIndex]["tgtU"]))
uMax = max(list(df.iloc[:stopIndex]["posU"]) + list(df.iloc[:stopIndex]["tgtU"]))
trajectory.set_box_aspect(
	(
		np.ptp([eMin, eMax]), 
		np.ptp([nMin, nMax]), 
		np.ptp([uMin, uMax]),
	)
)
trajectory.plot(
	df.iloc[:stopIndex]["posE"],
	df.iloc[:stopIndex]["posN"],
	df.iloc[:stopIndex]["posU"],
	color="b", label="INTERCEPTOR"
)
if df.iloc[startIndex]["tgtE"] == df.iloc[stopIndex]["tgtE"]:
	trajectory.scatter(
		df.iloc[stopIndex]["tgtE"],
		df.iloc[stopIndex]["tgtN"],
		df.iloc[stopIndex]["tgtU"],
		color="r", label="TARGET"
	)
else:
	trajectory.plot(
		df.iloc[:stopIndex]["tgtE"],
		df.iloc[:stopIndex]["tgtN"],
		df.iloc[:stopIndex]["tgtU"],
		color="r", label="TARGET"
	)
trajectory.legend()

acc = fig.add_subplot(122)
acc.set_xlabel("TIME OF FLIGHT (SECONDS)")
acc.set_ylabel("METERS PER S^2")
acc.plot(
	df.iloc[:stopIndex]["tof"],
	df.iloc[:stopIndex]["normComm"],
	color="r",
	label="NORMAL COMMAND"
)
acc.plot(
	df.iloc[:stopIndex]["tof"],
	df.iloc[:stopIndex]["normAch"],
	color="r",
	linestyle="dotted",
	label="NORMAL ACHIEVED"
)
acc.plot(
	df.iloc[:stopIndex]["tof"],
	df.iloc[:stopIndex]["sideComm"],
	color="b",
	label="SIDE COMMAND"
)
acc.plot(
	df.iloc[:stopIndex]["tof"],
	df.iloc[:stopIndex]["sideAch"],
	color="b",
	linestyle="dotted",
	label="SIDE ACHIEVED"
)
acc.legend()

plt.get_current_fig_manager().full_screen_toggle()
plt.show()