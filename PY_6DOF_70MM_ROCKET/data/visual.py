import matplotlib.pyplot as plt
import pandas as pd
import matPlotLibColors as mc

f = open("PY_6DOF_70MM_ROCKET\data\log.txt")
df = pd.read_csv(f, delim_whitespace=True)

fig = plt.figure()

startIndex = 0
stopIndex = -1

colors = mc.matPlotLibColors()

ax1 = fig.add_subplot(221)
ax1.plot(df.iloc[startIndex:stopIndex]["RNG"], df.iloc[startIndex:stopIndex]["ALT"], \
	color=colors.pop(0))
ax1.set_xlabel("TOF")
ax1.set_title("POSITION FEET")

ax2 = fig.add_subplot(222)
ax2.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["RATE"], \
	color=colors.pop(0))
ax2.set_xlabel("TOF")
ax2.set_title("RATE RADS")

ax3 = fig.add_subplot(223)
ax3.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["ALPHA"], \
	color=colors.pop(0))
ax3.set_xlabel("TOF")
ax3.set_title("ALPHA RADS")

ax4 = fig.add_subplot(224)
ax4.plot(df.iloc[startIndex:stopIndex]["TOF"], df.iloc[startIndex:stopIndex]["WDOT"], \
	color=colors.pop(0))
ax4.set_xlabel("TOF")
ax4.set_title("WDOT FT PER S^2")

plt.show()



















