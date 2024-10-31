import cv2
import numpy as np

# 初始化全局变量
drawing = False  # 是否正在绘制矩形
ix, iy = -1, -1  # 矩形起点坐标
rectangle = []  # 保存矩形坐标

x_start2, y_start2 = 116, 248
x_end2, y_end2 = 355, 333

# 打开摄像头
cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("无法打开摄像头")
    exit()

while True:
    ret, cv_image = cap.read()
    if not ret:
        print("无法读取摄像头画面")
        break
    height, width, channels = cv_image.shape
    cv2.imshow('1',cv_image)
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    # 黄色线条的阈值
    lower_yellow = np.array([0, 80, 0])
    upper_yellow = np.array([60, 255, 255])
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # 处理黄色线条的掩码
    dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (1, 1))
    erode_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

    yellow_mask = cv2.dilate(yellow_mask, dilate_kernel)
    yellow_mask = cv2.erode(yellow_mask, erode_kernel)
    yellow_mask = cv2.medianBlur(yellow_mask, 9)

    # 计算指定区域的黄色的质心
    rect_yellow_mask = yellow_mask[y_start2:y_end2, x_start2:x_end2]
    m_yellow = cv2.moments(rect_yellow_mask, False)
    try:
        cx_yellow, cy_yellow = m_yellow['m10'] / m_yellow['m00'], m_yellow['m01'] / m_yellow['m00']
        cx_yellow += x_start2
        cy_yellow += y_start2
    except ZeroDivisionError:
        cx_yellow, cy_yellow = height / 2, width / 2

    cv2.rectangle(yellow_mask, (x_start2, y_start2), (x_end2, y_end2), (255, 255, 0), 7)

    # 在掩码图像上画出黄色线条的质心
    cv2.circle(yellow_mask, (int(cx_yellow), int(cy_yellow)), 0, (0, 0, 0), 50)
    cv2.imshow('yellow_mask', yellow_mask)

    k = cv2.waitKey(1) & 0xFF
    if k == 27:  # 按下ESC键退出
        break

# 释放摄像头并关闭窗口
cap.release()
cv2.destroyAllWindows()

# 输出保存的矩形坐标
if rectangle:
    print("矩形起点坐标: ", rectangle[0])
    print("矩形终点坐标: ", rectangle[1])
else:
    print("没有绘制矩形区域")
