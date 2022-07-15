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
	
	scale = 1.5
	ax.plot((0, 0.57735 * scale), (0, 0.57735 * scale), (0, 0.57735 * scale), color="g", alpha=0.5, label="LINE OF SIGHT TO TARGET")

	plt.legend()
	plt.show()



if __name__ == "__main__":
	plotBodyFrames(
		[
			[
				"NO DEVIATION",
				[
					[0.57735, 0.57735, 0.57735],
					[-0.70711, 0.70711, 0.0],
					[-0.40825, -0.40825, 0.81650]
				]
			],
			[
				"WITH DEVIATED ANGLES",
				[
					[0.47776, 0.58432, 0.65598],
					[-0.81175, 0.57911, 0.07535],
					[-0.33586, -0.56849, 0.75101]
				]
			],
		]
	)