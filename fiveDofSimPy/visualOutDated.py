import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

df = pd.read_csv(open("log.txt"), delimiter=" ")
cols = df.columns
timeCol = cols[0]
stopPlotIndex = len(df)
groups = [
	[cols[1], cols[2], cols[3]],
	[cols[4], cols[5], cols[6]],
	[cols[7], cols[8], cols[9]],
	[cols[10], cols[11], cols[12], cols[13], cols[14], cols[15]]
]
for index, group in enumerate(groups):
	if index == 0:
		fig = plt.figure()
		ax1 = fig.add_subplot(221)
		ax1.set_xlabel("TIME OF FLIGHT")
		ax1.set_ylabel(f"{group[0]}")
		ax1.plot(df.iloc[:stopPlotIndex][f"{timeCol}"], df.iloc[:stopPlotIndex][f"{group[0]}"])
		ax1.scatter(
			df.iloc[-1][f"{timeCol}"],
			df.iloc[-1]["targetEast(m)"],
			marker="o", color="red", s=1, label="DESIRED INTERCEPT POINT")
		ax1.legend()
		ax2 = fig.add_subplot(222)
		ax2.set_xlabel("TIME OF FLIGHT")
		ax2.set_ylabel(f"{group[1]}")
		ax2.plot(df.iloc[:stopPlotIndex][f"{timeCol}"], df.iloc[:stopPlotIndex][f"{group[1]}"])
		ax2.scatter(
			df.iloc[-1][f"{timeCol}"],
			df.iloc[-1]["targetNorth(m)"],
			marker="o", color="red", s=1, label="DESIRED INTERCEPT POINT")
		ax2.legend()
		ax3 = fig.add_subplot(223)
		ax3.set_xlabel("TIME OF FLIGHT")
		ax3.set_ylabel(f"{group[2]}")
		ax3.plot(df.iloc[:stopPlotIndex][f"{timeCol}"], df.iloc[:stopPlotIndex][f"{group[2]}"])
		ax3.scatter(
			df.iloc[-1][f"{timeCol}"],
			df.iloc[-1]["targetUp(m)"],
			marker="o", color="red", s=1, label="DESIRED INTERCEPT POINT")
		ax3.legend()
		ax4 = fig.add_subplot(224, projection="3d")
		ax4.set_xlabel("EAST")
		ax4.set_ylabel("NORTH")
		ax4.set_zlabel("UP")
		ax4.scatter(
			df.iloc[-1]["targetEast(m)"],
			df.iloc[-1]["targetNorth(m)"],
			df.iloc[-1]["targetUp(m)"],
			marker="o", color="red", s=1, label="DESIRED INTERCEPT POINT")
		ax4.legend()
		ax4.plot(df.iloc[:stopPlotIndex][f"{group[0]}"], df.iloc[:stopPlotIndex][f"{group[1]}"], df.iloc[:stopPlotIndex][f"{group[2]}"])
	elif len(group) == 6:
		fig = plt.figure()
		ax1 = fig.add_subplot(221)
		ax1.set_xlabel("TIME OF FLIGHT")
		ax1.plot(df.iloc[:stopPlotIndex][f"{timeCol}"], df.iloc[:stopPlotIndex][f"{group[2]}"], label=group[2], color="blue")
		ax1.plot(df.iloc[:stopPlotIndex][f"{timeCol}"], df.iloc[:stopPlotIndex][f"{group[3]}"], label=group[3], color="red")
		ax1.legend()
		ax2 = fig.add_subplot(222)
		ax2.set_xlabel("TIME OF FLIGHT")
		ax2.plot(df.iloc[:stopPlotIndex][f"{timeCol}"], df.iloc[:stopPlotIndex][f"{group[4]}"], label=group[4], color="blue")
		ax2.plot(df.iloc[:stopPlotIndex][f"{timeCol}"], df.iloc[:stopPlotIndex][f"{group[5]}"], label=group[5], color="red")
		ax2.legend()
		ax3 = fig.add_subplot(223)
		ax3.set_xlabel("TIME OF FLIGHT")
		ax3.set_ylabel("ALPHA")
		ax3.plot(df.iloc[:stopPlotIndex][f"{timeCol}"], df.iloc[:stopPlotIndex][f"{group[0]}"], color="red")
		ax4 = fig.add_subplot(224)
		ax4.set_xlabel("TIME OF FLIGHT")
		ax4.set_ylabel("BETA")
		ax4.plot(df.iloc[:stopPlotIndex][f"{timeCol}"], df.iloc[:stopPlotIndex][f"{group[1]}"], color="red")
	else:
		fig = plt.figure()
		ax1 = fig.add_subplot(131)
		ax1.set_xlabel("TIME OF FLIGHT")
		ax1.set_ylabel(f"{group[0]}")
		ax1.plot(df.iloc[:stopPlotIndex][f"{timeCol}"], df.iloc[:stopPlotIndex][f"{group[0]}"])
		ax2 = fig.add_subplot(132)
		ax2.set_xlabel("TIME OF FLIGHT")
		ax2.set_ylabel(f"{group[1]}")
		ax2.plot(df.iloc[:stopPlotIndex][f"{timeCol}"], df.iloc[:stopPlotIndex][f"{group[1]}"])
		ax3 = fig.add_subplot(133)
		ax3.set_xlabel("TIME OF FLIGHT")
		ax3.set_ylabel(f"{group[2]}")
		ax3.plot(df.iloc[:stopPlotIndex][f"{timeCol}"], df.iloc[:stopPlotIndex][f"{group[2]}"])

plt.show()