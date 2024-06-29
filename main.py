import cv2
import pandas as pd
import numpy as np

df = pd.read_csv("point.csv")

img = np.ones((1000,1000,3), np.uint8) * 255

for point in df.values:
    cv2.circle(img, (point[0] + 500, point[1] + 500), 1, (0, 0, 255), 3)

cv2.imshow("frame", img)

cv2.waitKey()
cv2.imwrite("output.png", img)
cv2.destroyAllWindows()