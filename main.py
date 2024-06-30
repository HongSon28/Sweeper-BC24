import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import math

df = pd.read_csv("/content/point.csv")
data = df.to_numpy()

plt.scatter([-y for y in data[:, 1]], data[:, 0])
plt.show()
