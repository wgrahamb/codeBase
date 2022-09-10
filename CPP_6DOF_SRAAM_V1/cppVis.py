import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

f1 = "CPP_6DOF_SRAAM_V1/log.txt"

df = pd.read_csv(open(f"{f1}"), delimiter=" ", skipfooter=3, engine="python")

startIndex = 1
stopIndex = -1
fig = plt.figure()

# WINDOW ONE
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
trajectory.plot(df.iloc[startIndex:stopIndex]["posE"], df.iloc[startIndex:stopIndex]["posN"], df.iloc[startIndex:stopIndex]["posU"], color="b")
trajectory.scatter(df.iloc[stopIndex]["tgtE"], df.iloc[stopIndex]["tgtN"], df.iloc[stopIndex]["tgtU"], color="r")

# WINDOW 2
pitch = fig.add_subplot(232)
pitch.set_title("PITCH")
pitch.set_xlabel("TIME OF FLIGHT (SECONDS)")
pitch.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["pitchDefl"] * 50, label="PITCH FIN DEFLECTION DEG * 50", color="k")
pitch.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["normComm"], label="NORMAL ACC COMMAND Gs", color="b", alpha=0.5)
pitch.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["normAch"], label="NORMAL ACC ACHIEVED Gs", color="r")
pitch.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["pitchRate"], label="PITCH RATE DEG PER SEC", color="g")
pitch.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["thetaRate"], label="THETA RATE DEG PER SEC", color="cyan")
pitch.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["alpha"] * 50, label="ALPHA DEG * 50", color="purple", alpha=0.5)
pitch.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["theta"], label="THETA DEG", color="orange")
pitch.legend(fontsize="x-small")


# WINDOW 3
yaw = fig.add_subplot(233)
yaw.set_title("YAW")
yaw.set_xlabel("TIME OF FLIGHT (SECONDS)")
yaw.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["yawDefl"] * 50, label="YAW FIN DEFLECTION DEG * 50", color="k")
yaw.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["sideComm"], label="SIDE ACC COMMAND Gs", color="b", alpha=0.5)
yaw.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["sideAch"], label="SIDE ACC ACHIEVED Gs", color="r")
yaw.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["yawRate"], label="YAW RATE DEG PER SEC", color="g")
yaw.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["psiRate"], label="PSI RATE DEG PER SEC", color="cyan")
yaw.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["beta"] * 50, label="BETA DEG * 50", color="purple", alpha=0.5)
yaw.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["psi"], label="PSI DEG", color="orange")
yaw.legend(fontsize="x-small")

# WINDOW FOUR
roll = fig.add_subplot(234)
roll.set_title("ROLL")
roll.set_xlabel("TIME OF FLIGHT (SECONDS)")
roll.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["rollDefl"] * 50, label="ROLL FIN DEFLECTION DEG * 50", color="k")
roll.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["rollComm"], label="ROLL COMMAND Gs", color="b")
roll.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["phiRate"], label="PHI RATE DEG PER SEC", color="r")
roll.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["rollRate"], label="ROLL RATE DEG PER SEC", color="g")
roll.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["roll"], label="ROLL DEG", color="orange")
roll.legend()

# WINDOW FIVE
seeker = fig.add_subplot(235)
seeker.set_title("SEEKER ERROR")
seeker.set_xlabel("TIME OF FLIGHT (SECONDS)")
seeker.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["seekPitchErr"], label="PITCH ERROR", color="b")
seeker.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["seekYawErr"], label="YAW ERROR", color="r")
seeker.legend()

# WINDOW SIX
staticMarginAndMach = fig.add_subplot(236)
staticMarginAndMach.set_title("STATIC MARGIN AND MACH")
staticMarginAndMach.set_xlabel("TIME OF FLIGHT (SECONDS)")
staticMarginAndMach.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["staticMargin"], color="b", label="STATIC MARGIN")
staticMarginAndMach.plot(df.iloc[:stopIndex]["tof"], df.iloc[:stopIndex]["mach"], color="r", label="MACH")
staticMarginAndMach.legend()

fig.subplots_adjust(top=0.9, bottom=0.1, left=0.1, hspace=0.25)
plt.get_current_fig_manager().full_screen_toggle()
plt.show()