# 巡线保存视频

import cv2
import numpy as np

from prodedure_code.robot_connector import RobotConnector
robot = RobotConnector(ip_address='192.168.123.161', port=8000)

#from core.DetectArucoTag import detect_aruco
#dt_aruco = detect_aruco()

from prodedure_code.PID import PID
pid_yaw = PID(0.1, 1.2, -1.2, 0.018, 0.01, 0.00)
min_speed, max_speed = 0.05, 0.28

# 定义视频写入对象
fourcc = cv2.VideoWriter_fourcc(*'XVID')
out = None
recording = False

def start_recording():
    global out
    global recording
    # 创建视频写入对象
    out = cv2.VideoWriter('output.avi', fourcc, 20.0, (640, 480))  # 假设视频大小为 640x480
    recording = True
    print("Recording started.")

def stop_recording():
    global out
    global recording
    if out is not None:
        out.release()
    recording = False
    print("Recording stopped.")

def patrol_line(cv_image):
    global out
    height, width, channels = cv_image.shape

    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    lower_yellow = np.array([10, 40, 100])
    upper_yellow = np.array([40, 255, 255])
    color_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

    dilate_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
    erode_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    color_mask = cv2.medianBlur(color_mask, 9)
    color_mask = cv2.dilate(color_mask, dilate_kernel)
    color_mask = cv2.erode(color_mask, erode_kernel)

    m = cv2.moments(color_mask, False)
    try:
        cx, cy = m['m10'] / m['m00'], m['m01'] / m['m00']
    except ZeroDivisionError:
        cx, cy = height / 2, width / 2

    radius = 0
    color = (0, 255, 0)
    thickness = 50
    cv2.circle(color_mask, (int(cx), int(cy)), radius, color, thickness)

    cv2.imshow('Original Video', cv_image)
    cv2.imshow('Processed Video', color_mask)

    velocity = [0.0, 0.0]
    yawspeed = 0.0

    deviation = abs(width / 2 - cx)
    velocity[0] = max_speed - (max_speed - min_speed) * (deviation / (width / 2))
    yawspeed = pid_yaw.calculate(0, cx - width / 2)
    #print(velocity[0], yawspeed)
    robot.robot_high_control(velocity=velocity, yawSpeed=yawspeed)

    return cv_image

if __name__ == '__main__':
    cap = cv2.VideoCapture(0)

    while True:
        ret, cv_image = cap.read()
        if not ret:
            break

        # 处理原始视频
        processed_image = patrol_line(cv_image)

        # 显示原始视频和处理后的视频
        #cv2.imshow('Original Video', cv_image)
        #cv2.imshow('Processed Video', processed_image)

        mykey = cv2.waitKey(1)
        if mykey & 0xFF == ord('q'):
            break
        elif mykey & 0xFF == ord('c'):
            if not recording:
                start_recording()
        elif mykey & 0xFF == ord('s'):
            if recording:
                stop_recording()

        # 如果正在录制视频，将原始视频帧写入视频文件
        if recording:
            out.write(cv_image)

    cap.release()
    cv2.destroyAllWindows()
