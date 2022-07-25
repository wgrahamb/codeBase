import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

f1 = r"output/missile_6DOF.txt"

viewFile = f1

df = pd.read_csv(open(f"{viewFile}"), delimiter=" ")

for index, header in enumerate(df.columns):
	print(index, header)

startIndex = 1
stopIndex = -1

fig = plt.figure(figsize=(20, 20))

# Trajectory
scale = True
trajectory = fig.add_subplot(331, projection="3d")
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

# Roll
roll = fig.add_subplot(332)
roll.set_title("Roll")
roll.set_xlabel("Time of Flight")
roll.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["phi"], color="b", label="Phi Radians")
roll.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["phiDot"], color="r", label="Phi Dot Radians per Second")
roll.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["p"], color="g", label="Roll Rate Radians per Second")
roll.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["pdot"], color="cyan", label="Roll Rate Dot Radians per Second^2")
roll.legend(fontsize="xx-small")

# Pitch
pitch = fig.add_subplot(333)
pitch.set_title("Pitch")
pitch.set_xlabel("Time of Flight")
pitch.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["theta"], color="b", label="Theta Radians")
pitch.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["alphaDegrees"], color="fuchsia", label="Alpha Degrees")
pitch.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["thetaDot"], color="r", label="Theta Dot Radians per Second")
pitch.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["q"], color="g", label="Pitch Rate Radians per Second")
pitch.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["qdot"], color="cyan", label="Pitch Rate Dot Radians per Second^2")
pitch.legend(fontsize="xx-small")

# Yaw
yaw = fig.add_subplot(334)
yaw.set_title("Yaw")
yaw.set_xlabel("Time of Flight")
yaw.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["psi"], color="b", label="Psi Radians")
yaw.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["betaDegrees"], color="fuchsia", label="Beta Degrees")
yaw.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["psiDot"], color="r", label="Psi Dot Radians per Second")
yaw.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["r"], color="g", label="Yaw Rate Radians per Second")
yaw.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["rdot"], color="cyan", label="Yaw Rate Dot Radians per Second^2")
yaw.legend(fontsize="xx-small")

# Fins
fins = fig.add_subplot(335)
fins.set_title("Fins")
fins.set_xlabel("Time of Flight")
fins.set_ylabel("Radians")
fins.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["rollFinCommand"], color="b", label="Roll Fin Command")
fins.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["rollFinDeflection"], color="r", label="Roll Fin Deflection")
fins.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["pitchFinCommand"], color="g", label="Pitch Fin Command")
fins.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["pitchFinDeflection"], color="yellow", label="Pitch Fin Deflection")
fins.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["yawFinCommand"], color="pink", label="Yaw Fin Command")
fins.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["yawFinDeflection"], color="orange", label="Yaw Fin Deflection")
fins.legend(fontsize="xx-small")

# Acceleration
acceleration = fig.add_subplot(336)
acceleration.set_title("Acceleration")
acceleration.set_xlabel("Time of Flight")
acceleration.set_ylabel("Meters per Second^2")
acceleration.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["vdot"], color="b", label="Achieved Side Acceleration")
acceleration.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["guidanceSideCommand"], color="r", label="Commanded Side Acceleration")
acceleration.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["wdot"], color="g", label="Achieved Normal Acceleration")
acceleration.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["guidanceNormalCommand"], color="orange", label="Commanded Normal Acceleration")
acceleration.legend(fontsize="xx-small")

# Mach, Axial Acceleration, Drag Coefficient, Acceleration Limit, Static Margin, and Angle of Attack.
performance = fig.add_subplot(337)
performance.set_title("Performance")
performance.set_xlabel("Time of Flight")
performance.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["mach"], color="b", label="Mach Speed")
performance.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["CX"], color="g", label="Axial Drag Coefficient")
performance.plot(df.iloc[startIndex:stopIndex]["tof"], 0.01 * df.iloc[startIndex:stopIndex]["accelerationLimit"], color="r", label="1% Maneuvering Limit M/S^2")
performance.plot(df.iloc[startIndex:stopIndex]["tof"], 0.01 * df.iloc[startIndex:stopIndex]["udot"], color="orange", label="1% Axial Acceleration M/S^2")
performance.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["staticMargin"], color="cyan", label="Static Margin")
performance.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["alphaPrimeDegrees"], color="fuchsia", label="Angle of Attack Degrees")
performance.legend(fontsize="xx-small")

# Seeker
seeker = fig.add_subplot(338)
seeker.set_title("Seeker")
seeker.set_xlabel("Time of Flight")
seeker.set_ylabel("Radians")
seeker.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["seekerPitch"], color="b", label="Seeker Pitch Off Missile Boresight")
seeker.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["seekerYaw"], color="r", label="Seeker Yaw Off Missile Boresight")
seeker.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["seekerPitchError"], color="g", label="Seeker Boresight Vertical Offset from Target")
seeker.plot(df.iloc[startIndex:stopIndex]["tof"], df.iloc[startIndex:stopIndex]["seekerYawError"], color="cyan", label="Seeker Boresight Horizontal Offset from Target")
seeker.legend(fontsize="xx-small")

fig.subplots_adjust(top=0.9, bottom=0.1, left=0.1, hspace=0.4)
plt.show()
# %%
