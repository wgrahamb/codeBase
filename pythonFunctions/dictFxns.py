

# Function to sum all values in a dictionary.
def sumValuesInADictionary(DICT):
	ret = 0
	for index, key in enumerate(DICT.keys()):
		ret += DICT[f"{key}"]
	return ret

# Function to print all values in a dictionary.
def printValuesInADictionary(DICT):
	for index, key in enumerate(DICT.keys()):
		print(key, DICT[f"{key}"])
	print("\n")