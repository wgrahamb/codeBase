import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

f1 = "SIMDOF_SRAAM.txt"
f2 = "MMT_Lite.txt"

viewFile = f2

df = pd.read_csv(open(f"{viewFile}"), delimiter=" ", skipfooter=3, engine="python")

startIndex = 1
stopIndex = -10
fig = plt.figure()

# WINDOW ONE
trajectory = fig.add_subplot(221, projection="3d")
trajectory.set_title("TRAJECTORY")
trajectory.set_xlabel("X")
trajectory.set_ylabel("Y")
trajectory.set_zlabel("Z")
xMin = min(list(df.iloc[startIndex:stopIndex]["posX"]) + list(df.iloc[startIndex:stopIndex]["tgtX"]))
xMax = max(list(df.iloc[startIndex:stopIndex]["posX"]) + list(df.iloc[startIndex:stopIndex]["tgtX"]))
yMin = min(list(df.iloc[startIndex:stopIndex]["posY"]) + list(df.iloc[startIndex:stopIndex]["tgtY"]))
yMax = max(list(df.iloc[startIndex:stopIndex]["posY"]) + list(df.iloc[startIndex:stopIndex]["tgtY"]))
zMin = min(list(df.iloc[startIndex:stopIndex]["posZ"]) + list(df.iloc[startIndex:stopIndex]["tgtZ"]))
zMax = max(list(df.iloc[startIndex:stopIndex]["posZ"]) + list(df.iloc[startIndex:stopIndex]["tgtZ"]))
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
trajectory.plot(df.iloc[startIndex:stopIndex]["posX"], df.iloc[startIndex:stopIndex]["posY"], df.iloc[startIndex:stopIndex]["posZ"], color="b")
trajectory.scatter(df.iloc[stopIndex]["tgtX"], df.iloc[stopIndex]["tgtY"], df.iloc[stopIndex]["tgtZ"], color="r")

# WINDOW 2
pitch = fig.add_subplot(222)
pitch.set_title("PITCH")
pitch.set_xlabel("TIME OF FLIGHT (SECONDS)")
pitch.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["normAch"] * (1/10), label="NORMAL ACC Gs * (1/10)", color="b")
pitch.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["pitchRate"], label="PITCH RATE RADS", color="r")
pitch.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["alpha"], label="ALPHA RADS", color="k")
pitch.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["theta"], label="THETA RADS", color="cyan")
pitch.legend(fontsize="x-small")

# WINDOW 3
yaw = fig.add_subplot(223)
yaw.set_title("YAW")
yaw.set_xlabel("TIME OF FLIGHT (SECONDS)")
yaw.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["sideAch"] * (1/10), label="SIDE ACC Gs * (1/10)", color="b")
yaw.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["yawRate"], label="YAW RATE RADS", color="r")
yaw.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["beta"], label="BETA RADS", color="k")
yaw.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["psi"], label="PSI RADS", color="cyan")
yaw.legend(fontsize="x-small")

# WINDOW FOUR
roll = fig.add_subplot(224)
roll.set_title("ROLL")
roll.set_xlabel("TIME OF FLIGHT (SECONDS)")
roll.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["rollRate"], label="ROLL RATE RADS", color="r")
roll.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["phi"] * 10, label="PHI RADS * 10", color="k")
roll.legend(fontsize="x-small")

fig.subplots_adjust(top=0.9, bottom=0.1, left=0.1, hspace=0.25)
# plt.get_current_fig_manager().full_screen_toggle()
plt.show()