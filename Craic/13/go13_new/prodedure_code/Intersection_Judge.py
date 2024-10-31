import cv2
import numpy as np
from go13.core.PID import PID

pid_yaw = PID(0.1, 1.2, -1.2, 0.018, 0.01, 0.00)
min_speed, max_speed = 0.05, 0.28

# 路口检测的阈值
INTERSECTION_THRESHOLD = 4400

def intersection_Judge(cv_image):
    # BGR 图像转换为 HSV 色彩空间
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # 定义黄色范围，创建了一个掩膜，只保留黄色部分
    lower_yellow = np.array([0, 5, 80])
    upper_yellow = np.array([50, 255, 255])
    color_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    # 减少噪声和小斑点
    # 定义一个用于膨胀操作的结构元素，
    dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))  # 椭圆形的内核
    erode_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))  # 矩形内核
    color_mask = cv2.dilate(color_mask, dilate_kernel)  # 对图像进行膨胀操作
    color_mask = cv2.erode(color_mask, erode_kernel)    # 对图像进行腐蚀操作
    color_mask = cv2.medianBlur(color_mask, 9)          # 对图像进行中值模糊操作

    # 计算黄色区域的面积
    yellow_area = np.sum(color_mask > 0)
    return yellow_area

if __name__ == '__main__':
    # # 视频文件路径
    video_path = 'F:/2024/GO_Robcom/ceshi/match_back/1.mp4'
    # 打开视频文件
    cap = cv2.VideoCapture(video_path)
    # 获取视频帧率
    fps = cap.get(cv2.CAP_PROP_FPS)
    wait_time = int(500 / fps)  # 计算等待时间

    previous_yellow_area = None
    while True:
        ret, cv_image = cap.read()
        cv2.imshow('color', cv_image)
        current_yellow_area = patrol_line(cv_image)

        if previous_yellow_area is not None:
            area_change = current_yellow_area - previous_yellow_area
            # if area_change > INTERSECTION_THRESHOLD :
            #     print("Intersection detected")
            if current_yellow_area > 83000 and previous_yellow_area < 83000:
                print("Intersection detected")

        previous_yellow_area = current_yellow_area
        mykey = cv2.waitKey(wait_time)

        if mykey & 0xFF == ord('q'):
            break
