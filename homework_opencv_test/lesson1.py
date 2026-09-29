import cv2
import numpy as np

# 造一张 300 行 × 400 列 × 3 通道的纯黑图
img = np.zeros((300, 400, 3), dtype=np.uint8)

# 把 y 从 100~200、x 从 50~150 的区域涂成 (255, 0, 0)
img[100:200, 50:150] = (255, 0, 0)

print("shape =", img.shape)            # 期望 (300, 400, 3)
print("左上角像素 =", img[0, 0])        # 期望 [0 0 0]
print("矩形中心像素 =", img[150, 100])   # 期望 [255 0 0]

cv2.imwrite("lesson1.png", img)
print("已保存 lesson1.png")