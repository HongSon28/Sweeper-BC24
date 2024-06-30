# https://colab.research.google.com/drive/10PxHgigsKHOE5KYTPEYs-1kieh329hQQ?usp=sharing

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import math

df = pd.read_csv("/content/point.csv")
data = df.to_numpy()

plt.scatter([-y for y in data[:, 1]], data[:, 0])
plt.show()
