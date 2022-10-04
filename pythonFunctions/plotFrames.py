import enum
from matplotlib import projections
import numpy as np
from numpy import array as npa
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('WebAgg')

# Input: Phi, theta, and psi in radians.
def ATTITUDE_TO_LOCAL_TM(PHI, THETA, PSI):
	O_TO_L_TM = npa(
		[
			[
				np.cos(PSI) * np.cos(THETA),
				np.sin(PSI) * np.cos(THETA),
				-np.sin(THETA)
			],
			[
				np.cos(PSI) * np.sin(THETA) * np.sin(PHI) - np.sin(PSI) * np.cos(PHI),
				np.sin(PSI) * np.sin(THETA) * np.sin(PHI) + np.cos(PSI) * np.cos(PHI),
				np.cos(THETA) * np.sin(PHI)
			],
			[
				np.cos(PSI) * np.sin(THETA) * np.cos(PHI) + np.sin(PSI) * np.sin(PHI),
				np.sin(PSI) * np.sin(THETA) * np.cos(PHI) - np.cos(PSI) * np.sin(PHI),
				np.cos(THETA) * np.cos(PHI)
			]
		]
	)
	return O_TO_L_TM

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
			ax.plot((0, unitVector[0]), (0, unitVector[1]), (0, unitVector[2]), \
				color=colors[iteration], label=plotLabel, \
				linestyle=lineStyles[index])
	plt.legend()
	plt.show()

if __name__ == "__main__":

	euler1 = [0.0, np.radians(45.0), 0.0]
	euler2 = [0.0, np.radians(35.0), 0.0]

	x = ATTITUDE_TO_LOCAL_TM(euler1[0], euler1[1] * -1.0, euler1[2])
	y = ATTITUDE_TO_LOCAL_TM(euler2[0], euler2[1] * -1.0, euler2[2])

	frame1 = ["one", x]
	frame2 = ["two", y]

	frames = [frame1, frame2]

	plotBodyFrames(frames)

