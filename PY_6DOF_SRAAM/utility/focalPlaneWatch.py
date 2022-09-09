import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

df = pd.read_csv(open("PY_6DOF_SRAAM/log.txt"), delimiter=" ")

fig = plt.figure()
focalPlane = fig.add_subplot()
focalPlane.set_xlim([-0.1744, 0.1744])
focalPlane.set_ylim([-0.1744, 0.1744])

seekErr = focalPlane.scatter([], [], color="r", label="ERROR", marker="2")
focalPlane.scatter(0, 0, color="g")

def update(frames, data):
	seekErr.set_offsets(
		[data.iloc[frames]["seekYawErr"],
		data.iloc[frames]["seekPitchErr"]]
	)
	rets = [seekErr]
	return rets

animation = FuncAnimation(
	fig=fig,
	func=update,
	frames=len(df),
	fargs=([df]),
	repeat=True,
	repeat_delay=1500,
	blit=True,
	interval=0.0001
)

plt.get_current_fig_manager().full_screen_toggle()
plt.show()