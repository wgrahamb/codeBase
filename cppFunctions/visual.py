import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv(open("output/MCData.txt"), delimiter=" ")
plt.hist(df.iloc[:, 2], bins=100)
plt.show()