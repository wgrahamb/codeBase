import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from arrayFxns import DISTANCE
import random
import matplotlib
matplotlib.use('WebAgg')

# INPUT SHOULD BE A LIST COMPRISED OF NUMPY ARRAYS
def sortListByNearestPoint(inputList):
	# CHECK IF SAMPLE ARRAY IS A LIST
	if type(inputList) != list:
		# CONVERT IF IT IS NOT
		inputList = list(inputList)
	# INITIALIZE THE ARRAY TO BE SORTED
	sortedList = []
	# INITIALIZE THE FIRST POINT OF THE ARRAY
	startingPoint = [None, None, None]
	# LOOP THROUGH THE PROVIDED ARRAY
	for index, point in enumerate(inputList):
		# BECAUSE STARTING POINT IS EMPTY
		if index == 0:
			# CALCULATE THE MAGNITUDE OF THE POINT
			magnitude = np.linalg.norm(point)
			# POPULATE STARTING POINT
			startingPoint[0], startingPoint[1], startingPoint[2] = index, magnitude, point
		# NOW THAT STARTING POINT IS CALCULATED
		else:
			# CALCULATE THE MAGNITUDE OF THE POINT
			magnitude = np.linalg.norm(point)
			# IF THE MAGNITUDE OF THIS POINT IS CLOSER TO ZERO THAN THE CURRENT STARTING POINT
			if magnitude < startingPoint[1]:
				# UPDATE STARTING POINT
				startingPoint[0], startingPoint[1], startingPoint[2] = index, magnitude, point
	# APPEND STARTING POINT TO THE SORTED ARRAY
	sortedList.append(startingPoint[2])
	# REMOVE THE INDEX OF THE STARTING POINT FROM THE SAMPLE ARRAY
	inputList.pop(startingPoint[0])
	# WHILE COPIED ARRAY STILL HAS POINTS
	while len(inputList) > 0:
		# INITIALIZE THE NEXT POINT OF THE ARRAY
		nextPoint = [None, None, None]
		# LOOP THROUGH  
		for iteration, dataPoint in enumerate(inputList):
			# BECAUSE NEXT POINT IS EMPTY
			if iteration == 0:
				# CALCULATE THE DISTANCE FROM THE LAST POINT IN THE SORTED ARRAY
				distance = DISTANCE(sortedList[-1], dataPoint)
				# POPULATE NEXT POINT
				nextPoint[0], nextPoint[1], nextPoint[2] = iteration, distance, dataPoint
			# NOW THAT NEXT POINT IS POPULATED
			else:
				# CALCULATE THE DISTANCE FROM THE LAST POINT
				distance = DISTANCE(sortedList[-1], dataPoint)
				# IF THIS DISTANCE IS LESS THAN THE LAST DISTANCE CALCULATED
				if distance < nextPoint[1]:
					# UPDATE NEXT POINT
					nextPoint[0], nextPoint[1], nextPoint[2] = iteration, distance, dataPoint
		# APPEND NEXT POINT TO THE SORTED ARRAY
		sortedList.append(nextPoint[2])
		# REMOVE THE INDEX OF THE NEXT POINT FROM THE SAMPLE ARRAY
		inputList.pop(nextPoint[0])
	# RETURN THE LIST
	return sortedList



if __name__ == "__main__":
	x = []
	for i in range(0, 100, 1):
		y = i * i + 25
		x.append(np.array([i, y]))
	random.shuffle(x)
	returnList = sortListByNearestPoint(x)
	df = pd.DataFrame(returnList)
	plt.plot(df.iloc[:, 0],df.iloc[:, 1])
	plt.show()