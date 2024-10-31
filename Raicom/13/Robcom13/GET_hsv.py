import cv2
import numpy as np


def preprocess_image(cv_image):
    height, width, _ = cv_image.shape
    center = ((width // 2) + 9, (height // 2) - 10)
    radius = 145
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.circle(mask, center, radius, (255), thickness=-1)
    masked_image = cv2.bitwise_and(cv_image, cv2.merge([mask, mask, mask]))
    white_background = np.full_like(cv_image, 255)
    circular_image = np.where(cv2.merge([mask, mask, mask]) == 0, white_background, masked_image)
    return circular_image
# 回调函数，无实际功能，仅用于滑动条创建时需要
def nothing(x):
    pass

# 创建窗口
cv2.namedWindow('Trackbars')

# 创建6个滑动条，分别用于调节HSV的上下阈值
cv2.createTrackbar('LH', 'Trackbars', 0, 255, nothing)
cv2.createTrackbar('LS', 'Trackbars', 0, 255, nothing)
cv2.createTrackbar('LV', 'Trackbars', 0, 255, nothing)
cv2.createTrackbar('UH', 'Trackbars', 255, 255, nothing)
cv2.createTrackbar('US', 'Trackbars', 255, 255, nothing)
cv2.createTrackbar('UV', 'Trackbars', 255, 255, nothing)


# 读取图像
# 你可以替换这部分代码读取视频的帧
image_path = "C:\\Users\\zsq\\Desktop\\calibration_image_03.jpg" # 替换为你的图像路径

image = cv2.imread(image_path)
#image = preprocess_image(image)
#height, width, _ = image.shape
#image = image[:height//2, :]
hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

while True:
    # 获取当前滑动条的值
    lh = cv2.getTrackbarPos('LH', 'Trackbars')
    ls = cv2.getTrackbarPos('LS', 'Trackbars')
    lv = cv2.getTrackbarPos('LV', 'Trackbars')
    uh = cv2.getTrackbarPos('UH', 'Trackbars')
    us = cv2.getTrackbarPos('US', 'Trackbars')
    uv = cv2.getTrackbarPos('UV', 'Trackbars')

    # 设置HSV的上下阈值
    lower_black = np.array([lh, ls, lv])
    upper_black = np.array([uh, us, uv])

    # 创建掩码
    mask = cv2.inRange(hsv, lower_black, upper_black)
    result = cv2.bitwise_and(image, image, mask=mask)

    # 显示图像
    cv2.imshow('Original', image)
    cv2.imshow('Mask', mask)
    cv2.imshow('Result', result)

    # 按下'q'键退出循环
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放窗口
cv2.destroyAllWindows()

# 64 92
