import enum
from matplotlib import projections
import numpy as np
import matplotlib.pyplot as plt

def plotBodyFrames(frames):
	fig = plt.figure()
	ax = fig.add_subplot(projection="3d")

	ax.set_xlim([-1, 1])
	ax.set_ylim([-1, 1])
	ax.set_zlim([-1, 1])

	ax.set_box_aspect(
		(
			np.ptp([-1, 1]), 
			np.ptp([-1, 1]), 
			np.ptp([-1, 1]), 
		)
	)

	zero = "AXIAL "
	one = "SIDE "
	two = "NORMAL "

	colors = ["r", "b", "g", "k"]
	lineStyles = ["solid", "dotted", "dashed"]

	for iteration, frame in enumerate(frames):
		label=frame[0]
		for index, unitVector in enumerate(frame[1]):
			plotLabel = None
			if index == 0:
				plotLabel = zero + label
			elif index == 1:
				plotLabel = one + label
			elif index == 2:
				plotLabel = two + label
			ax.plot((0, unitVector[0]), (0, unitVector[1]), (0, unitVector[2]), color=colors[iteration], label=plotLabel, linestyle=lineStyles[index])
	
	plt.legend()
	plt.show()