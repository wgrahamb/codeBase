import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import pandas as pd
import numpy as np
import math
import warnings

# to ignore runtime warnings about dividing
# by zero. handled by the fxn.
warnings.filterwarnings("ignore")

def compareTwoFiles(reportRelPath, refFileRelPath, newFileRelPath, xAxisKey):

	# Console report.
	print(reportRelPath)

	# Open pdf to write to.
	pdfFile = PdfPages(reportRelPath)

	# Create matplotlib fig.
	fig = plt.figure(figsize=(20,20))

	# Fxn to calculate percent difference.
	def percentDiff(a, b):
		ret = None
		t1 = np.abs(a - b)
		t2 = (np.abs(a) + np.abs(b)) / 2.0
		ret = (t1 / t2) * 100.0
		# results from dividing by zero,
		# and if dividing by zero, the diff iz zero
		if math.isnan(ret) or math.isinf(ret): 
			ret = 0.0
		return ret

	# Fxn to plot two lines on one subplot.
	def plotAndWrite(xs, ys, labels, header):
		ax = fig.add_subplot(111)
		colors = ["r", "b", "g", "cyan"]
		for index, x in enumerate(xs):
			ax.plot(x , ys[index], label=labels[index], color=colors[index])
		plt.xlabel("X") 
		plt.ylabel("Y") 
		plt.title(header)
		plt.legend()
		pdfFile.savefig(fig)
		plt.clf()

	# Fxn to calculate diffs between two y sets and plot histogram of diffs.
	def plotHistAndWrite(ys, header):
		diffs = []
		for index, y in enumerate(ys[0]):
			x = percentDiff(y, ys[1][index])
			diffs.append(x)
		ax = fig.add_subplot(111)
		ax.hist(diffs, bins=5, color="b")
		plt.title(f"{header} percent diffs histogram")
		pdfFile.savefig(fig)
		plt.clf()

	# Read in input comparison files to data frames
	dfs = []
	files = [refFileRelPath, newFileRelPath]
	for f in files:
		df = pd.read_csv(open(r"{}".format(f)), delimiter= " ")
		df.name = f
		dfs.append(df)

	# Get a list of the headers for the two files (should be identical)
	headers = []
	for dfIndex, df in enumerate(dfs):
		if dfIndex == 0:
			for headerIndex, header in enumerate(df.columns):
				headers.append(header)
			break

	# Write report.
	startIndex = 0
	stopIndex = -1
	for headerIndex, header in enumerate(headers):
		listOfStrings = header.split()
		doIt = True
		for index, string in enumerate(listOfStrings):
			if string == "Unnamed:": # normally only at the end of the header
				doIt = False
				break # normally only at the end of the header
		if doIt == True:
			print(headerIndex, header)
			xs = []
			ys = []
			labels = []
			for dfIndex, df in enumerate(dfs):
				if xAxisKey == "INDEX":
					temp = list(df.index)
					temp.pop(-1)
					xs.append(temp)
				else:
					xs.append(list(df.iloc[startIndex:stopIndex][f"{xAxisKey}"]))
				ys.append(list(df.iloc[startIndex:stopIndex][f"{header}"]))
				labels.append(df.name)
			plotAndWrite(xs, ys, labels, header)
			plotHistAndWrite(ys, header)
		# break
	print("\n")

	# Close pdf.
	pdfFile.close()

if __name__ == "__main__":

	print("WRITING REPORTS")

	compareTwoFiles(
		reportRelPath="PY_6DOF_70MM_ROCKET/report.pdf",
		refFileRelPath="PY_6DOF_70MM_ROCKET/data/log.txt",
		newFileRelPath="PY_6DOF_70MM_ROCKET/data/log.txt",
		xAxisKey="TOF"
	)














