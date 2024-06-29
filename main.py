import cv2
import pandas as pd
import numpy as np

df = pd.read_csv("point.csv").values

img = np.ones((max(df[:, 1]) + abs(min(df[:, 1])) + 20, max(df[:, 0]) + abs(min(df[:, 0])) + 20, 3), np.uint8) * 255

height, width, channels = img.shape
cv2.line(img, (0, int(height / 2)), (width, int(height / 2)), (0, 0, 0), 1)
cv2.line(img, (int(width / 2), 0), (int(width / 2), height), (0, 0, 0), 1)

for point in df:
    cv2.circle(img, (point[0] + abs(min(df[:, 0])) + 10, point[1] + abs(min(df[:, 1])) + 10), 1, (0, 0, 255), 3)

cv2.imshow("frame", img)

cv2.waitKey()
cv2.imwrite("output.png", img)
cv2.destroyAllWindows()