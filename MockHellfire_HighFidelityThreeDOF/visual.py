import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

f1 = r"MockHellfire_HighFidelityThreeDOF/data.txt"
viewFile = f1
df = pd.read_csv(open(viewFile), delimiter=" ")

for index, header in enumerate(df.columns):
	print(index, header)

fig = plt.figure(figsize=(20, 20))
startIndex = 0
stopIndex = -1

# Trajectory.
trajectory = fig.add_subplot(121)
trajectory.set_title("Trajectory")
trajectory.set_xlabel("RANGE (M)")
trajectory.set_ylabel("ALT (M)")
trajectory.plot(df.iloc[startIndex:stopIndex]["RNG"], df.iloc[startIndex:stopIndex]["ALT"])

# Maneuver.
maneuver = fig.add_subplot(122)
maneuver.set_title("MANEUVER")
maneuver.set_xlabel("TIME OF FLIGHT (S)")
maneuver.plot()


plt.show()















