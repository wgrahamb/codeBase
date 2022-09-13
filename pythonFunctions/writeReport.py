import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import pandas as pd
import os

def compareTwoFiles(reportPath, f1, f2, key):

	print(reportPath)

	pdfFile = PdfPages(reportPath)
	fig = plt.figure(figsize=(20,20))

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

	dfs = []
	files = [f1, f2]
	for f in files:
		df = pd.read_csv(open(r"{}".format(f)), delimiter= " ")
		df.name = f
		dfs.append(df)

	headers = []
	for dfIndex, df in enumerate(dfs):
		if dfIndex == 0:
			for headerIndex, header in enumerate(df.columns):
				headers.append(header)
			break

	startIndex = 0
	stopIndex = -1
	for index, header in enumerate(headers):
		print(index, header)
		listOfStrings = header.split()
		doIt = True
		for index, string in enumerate(listOfStrings):
			if string == "Unnamed:":
				doIt = False
				break
		if doIt == True:
			xs = []
			ys = []
			labels = []
			for dfIndex, df in enumerate(dfs):
				xs.append(list(df.iloc[startIndex:stopIndex][f"{key}"]))
				ys.append(list(df.iloc[startIndex:stopIndex][f"{header}"]))
				labels.append(df.name)
			plotAndWrite(xs, ys, labels, header)
	print("\n")

	pdfFile.close()

if __name__ == "__main__":

	print("WRITING REPORTS")
