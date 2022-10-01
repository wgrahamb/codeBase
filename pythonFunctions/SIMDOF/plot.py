import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib
matplotlib.use('WebAgg')

df1 = pd.read_csv(open("pythonFunctions/SIMDOF/output/ASSET_ONE.txt"), delimiter=" ")
df2 = pd.read_csv(open("pythonFunctions/SIMDOF/output/ASSET_TWO.txt"), delimiter=" ")
df3 = pd.read_csv(open("pythonFunctions/SIMDOF/output/ASSET_THREE.txt"), delimiter=" ")
df4 = pd.read_csv(open("pythonFunctions/SIMDOF/output/ASSET_FOUR.txt"), delimiter=" ")
df5 = pd.read_csv(open("pythonFunctions/SIMDOF/output/NESTED_ASSET_ONE.txt"), delimiter=" ")
df6 = pd.read_csv(open("pythonFunctions/SIMDOF/output/NESTED_ASSET_TWO.txt"), delimiter=" ")
df7 = pd.read_csv(open("pythonFunctions/SIMDOF/output/NESTED_ASSET_THREE.txt"), delimiter=" ")

plt.plot(df1.iloc[:]["TIMER"], df1.iloc[:]["VALUE"], label="ASSET ONE")
plt.plot(df2.iloc[:]["TIMER"], df2.iloc[:]["VALUE"], label="ASSET TWO")
plt.plot(df3.iloc[:]["TIMER"], df3.iloc[:]["VALUE"], label="ASSET THREE")
plt.plot(df4.iloc[:]["TIMER"], df4.iloc[:]["VALUE"], label="ASSET FOUR")
plt.plot(df5.iloc[:]["TIMER"], df5.iloc[:]["VALUE"], label="NESTED ASSET ONE")
plt.plot(df6.iloc[:]["TIMER"], df6.iloc[:]["VALUE"], label="NESTED ASSET TWO")
plt.plot(df7.iloc[:]["TIMER"], df7.iloc[:]["VALUE"], label="NESTED ASSET THREE")

plt.legend()
plt.show()
