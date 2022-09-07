import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from utility.matPlotLibColors import matPlotLibColors

RAD_TO_DEG = 57.2957795130823

# Setup.
f1 = r"PY_5DOF_MOCK_HELLFIRE_DYNAMICS/output/MOCK_HELLFIRE5DOF.txt"
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
trajectory = fig.add_subplot(221, projection="3d")
trajectory.view_init(elev=30, azim=135)
trajectory.set_title("Trajectory")
trajectory.set_xlabel("East")
trajectory.set_ylabel("North")
trajectory.set_zlabel("Up")
if scale:
	xMin = min(list(df.iloc[startIndex:stopIndex]["ENUPOSX"]))
	xMax = max(list(df.iloc[startIndex:stopIndex]["ENUPOSX"]))
	yMin = min(list(df.iloc[startIndex:stopIndex]["ENUPOSY"]))
	yMax = max(list(df.iloc[startIndex:stopIndex]["ENUPOSY"]))
	zMin = min(list(df.iloc[startIndex:stopIndex]["ENUPOSZ"]))
	zMax = max(list(df.iloc[startIndex:stopIndex]["ENUPOSZ"]))
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
trajectory.plot(
	df.iloc[startIndex:stopIndex]["ENUPOSX"],
	df.iloc[startIndex:stopIndex]["ENUPOSY"],
	df.iloc[startIndex:stopIndex]["ENUPOSZ"],
	color=colors.pop(0)
)

# Pitch.
pitch = fig.add_subplot(222)
pitch.set_title("Pitch")
pitch.set_xlabel("TOF")
pitch.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["ALPHA"] * RAD_TO_DEG, label="ALPHA - DEG", color=colors.pop(0))
pitch.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["ENUTHT"] * RAD_TO_DEG, label="THETA - DEG", color=colors.pop(0))
pitch.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["QRATE"] * RAD_TO_DEG, label="THETA DOT - DEG", color=colors.pop(0))
pitch.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["WDOT_0"], label="NORMAL ACC - M/S^2", color=colors.pop(0))
pitch.legend(fontsize="small")

# Yaw.
yaw = fig.add_subplot(223)
yaw.set_title("Yaw")
yaw.set_xlabel("TOF")
yaw.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["SIDESLIP"] * RAD_TO_DEG, label="SIDESLIP - DEG", color=colors.pop(0))
yaw.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["ENUPSI"] * RAD_TO_DEG, label="PSI - DEG", color=colors.pop(0))
yaw.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["RRATE"] * RAD_TO_DEG, label="PSI DOT - DEG", color=colors.pop(0))
yaw.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["VDOT_0"], label="SIDE ACC - M/S^2", color=colors.pop(0))
yaw.legend(fontsize="small")

# Fins
fins = fig.add_subplot(224)
fins.set_title("Fins")
fins.set_xlabel("TOF")
fins.set_ylabel("Degrees")
pitchFinsDF = pd.read_csv(open(f"PY_5DOF_MOCK_HELLFIRE_DYNAMICS/output/PITCH_DEFL.txt"), delimiter=" ")
yawFinsDF = pd.read_csv(open(f"PY_5DOF_MOCK_HELLFIRE_DYNAMICS/output/YAW_DEFL.txt"), delimiter=" ")
fins.plot(pitchFinsDF.iloc[startIndex:stopIndex]["TIME"], pitchFinsDF.iloc[startIndex:stopIndex]["COMMAND"], label="Pitch Command", color=colors.pop(0))
fins.plot(pitchFinsDF.iloc[startIndex:stopIndex]["TIME"], pitchFinsDF.iloc[startIndex:stopIndex]["DEFL"], label="Pitch Deflection", color=colors.pop(0))
fins.plot(yawFinsDF.iloc[startIndex:stopIndex]["TIME"], yawFinsDF.iloc[startIndex:stopIndex]["COMMAND"], label="Yaw Command", color=colors.pop(0))
fins.plot(yawFinsDF.iloc[startIndex:stopIndex]["TIME"], yawFinsDF.iloc[startIndex:stopIndex]["DEFL"], label="Yaw Deflection", color=colors.pop(0))
fins.legend(fontsize="small")

# Show.
fig.subplots_adjust(top=0.9, bottom=0.1, left=0.1, hspace=0.4)
plt.show()
