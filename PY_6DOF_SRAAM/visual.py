import pandas as pd
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('WebAgg')
import numpy as np
from utility import matPlotLibColors as mc

f1 = "PY_6DOF_SRAAM/log.txt"

df = pd.read_csv(open(f"{f1}"), delimiter=" ", skipfooter=3, engine="python")
startIndex = 1
stopIndex = -1
fig = plt.figure()
colors = mc.matPlotLibColors()

# TRAJECTORY
trajectory = fig.add_subplot(231, projection="3d")
trajectory.set_title("MISS")
trajectory.set_xlabel("EAST")
trajectory.set_ylabel("NORTH")
trajectory.set_zlabel("UP")
eMin = min(list(df.iloc[startIndex:stopIndex]["posE"]) + list(df.iloc[startIndex:stopIndex]["tgtE"]))
eMax = max(list(df.iloc[startIndex:stopIndex]["posE"]) + list(df.iloc[startIndex:stopIndex]["tgtE"]))
nMin = min(list(df.iloc[startIndex:stopIndex]["posN"]) + list(df.iloc[startIndex:stopIndex]["tgtN"]))
nMax = max(list(df.iloc[startIndex:stopIndex]["posN"]) + list(df.iloc[startIndex:stopIndex]["tgtN"]))
uMin = min(list(df.iloc[startIndex:stopIndex]["posU"]) + list(df.iloc[startIndex:stopIndex]["tgtU"]))
uMax = max(list(df.iloc[startIndex:stopIndex]["posU"]) + list(df.iloc[startIndex:stopIndex]["tgtU"]))
trajectory.set_box_aspect(
	(
		np.ptp([eMin - 1000, eMax + 1000]), 
		np.ptp([nMin - 1000, nMax + 1000]), 
		np.ptp([uMin, uMax + 1000]),
	)
)
trajectory.set_xlim([eMin - 1000, eMax + 1000])
trajectory.set_ylim([nMin - 1000, nMax + 1000])
trajectory.set_zlim([uMin, uMax + 1000])
trajectory.plot(df.iloc[startIndex:stopIndex]["posE"], df.iloc[startIndex:stopIndex]["posN"], df.iloc[startIndex:stopIndex]["posU"], color=colors.pop(0))
if df.iloc[stopIndex]["tgtE"] == df.iloc[startIndex]["tgtE"]:
	trajectory.scatter(df.iloc[stopIndex]["tgtE"], df.iloc[stopIndex]["tgtN"], df.iloc[stopIndex]["tgtU"], color=colors.pop(0))
else:
	trajectory.plot(df.iloc[:stopIndex]["tgtE"], df.iloc[:stopIndex]["tgtN"], df.iloc[:stopIndex]["tgtU"], color=colors.pop(0))

# ACCELERATIONS.
accelerations = fig.add_subplot(222)
accelerations.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["normComm"], label="NORMAL ACC COMMAND", color=colors.pop(0))
accelerations.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["normAch"], label="NORMAL ACC ACHIEVED", color=colors.pop(0))
accelerations.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["sideComm"], label="SIDE ACC COMMAND", color=colors.pop(0))
accelerations.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["sideAch"], label="SIDE ACC ACHIEVED", color=colors.pop(0))
accelerations.set_xlabel("TOF (Seconds)")
accelerations.set_ylabel("Gs")
accelerations.legend(fontsize="small")

# RATES.
rates = fig.add_subplot(223)
rates.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["p"], label="ROLL RATE RADS PER SEC", color=colors.pop(0))
rates.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["phi"], label="PHI RADS", color=colors.pop(0))
rates.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["q"], label="PITCH RATE RADS PER SEC", color=colors.pop())
rates.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["alpha"], label="ALPHA RADS")
rates.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["r"], label="YAW RATE RADS PER SEC", color=colors.pop())
rates.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["beta"], label="BETA RADS")
rates.set_xlabel("TOF (Seconds)")
rates.legend(fontsize="small")

# FINS.
fins = fig.add_subplot(224)
fins.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["fin1c"], label="FIN 1 COMM", color=colors.pop(0))
fins.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["fin2c"], label="FIN 2 COMM", color=colors.pop(0))
fins.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["fin3c"], label="FIN 3 COMM", color=colors.pop(0))
fins.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["fin4c"], label="FIN 4 COMM", color=colors.pop(0))
fins.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["fin1d"], label="FIN 1 DEFL", color=colors.pop(0))
fins.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["fin2d"], label="FIN 2 DEFL", color=colors.pop(0))
fins.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["fin3d"], label="FIN 3 DEFL", color=colors.pop(0))
fins.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["fin4d"], label="FIN 4 DEFL", color=colors.pop(0))
fins.set_xlabel("TOF (Seconds)")
fins.set_ylabel("Radians")
fins.legend(fontsize="small")

fig.subplots_adjust(top=0.9, bottom=0.1, left=0.1, hspace=0.25)
plt.get_current_fig_manager().full_screen_toggle()
plt.show()