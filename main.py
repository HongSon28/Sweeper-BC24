import cv2
import pandas as pd
import numpy as np

df = pd.read_csv("point.csv").values

img = np.ones((max(df[:, 1]) + abs(min(df[:, 1])) + 20, max(df[:, 0]) + abs(min(df[:, 0])) + 20, 3), np.uint8) * 255

for point in df:
    cv2.circle(img, (point[0] + abs(min(df[:, 0])) + 10, point[1] + abs(min(df[:, 1])) + 10), 1, (0, 0, 255), 3)

cv2.imshow("frame", img)

cv2.waitKey()
cv2.imwrite("output.png", img)
cv2.destroyAllWindows()