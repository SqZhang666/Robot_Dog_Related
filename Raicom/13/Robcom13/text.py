import cv2
import numpy as np
from core.robot_connector import RobotConnector
robot = RobotConnector(ip_address='192.168.123.161', port=8000)
from core.PID import PID
import time

pid_yaw = PID(0.1, 1.2, -1.2, 0.018, 0.01, 0.00)
min_speed, max_speed = 0.05, 0.28


def patrol_line(cv_image):
    height, width, channels = cv_image.shape

    # ����Բ����������ĺͰ뾶
    center = ((width // 2) + 9, (height // 2) - 10)
    radius = 145
    # ����Բ������
    mask = np.zeros((height, width), dtype=np.uint8)
    cv2.circle(mask, center, radius, (255), thickness=-1)
    # ����һ����ͨ��������
    mask_3ch = cv2.merge([mask, mask, mask])
    # Ӧ������
    masked_image = cv2.bitwise_and(cv_image, mask_3ch)
    # ����һ����ɫ����
    white_background = np.full_like(cv_image, 255)
    # ��Բ�������ڵ����ر�����������������Ϊ��ɫ
    circular_image = np.where(mask_3ch == 0, white_background, masked_image)
    cv2.imshow('chuli', circular_image)

    # ������Ӧ�õ�HSVͼ��
    hsv = cv2.cvtColor(circular_image, cv2.COLOR_BGR2HSV)

    # ��ɫ��������ֵ
    lower_yellow = np.array([0, 7, 0])
    upper_yellow = np.array([68, 255, 255])
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)


    # �����ɫ����������
    dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
    erode_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

    yellow_mask = cv2.dilate(yellow_mask, dilate_kernel)
    yellow_mask = cv2.erode(yellow_mask, erode_kernel)
    yellow_mask = cv2.medianBlur(yellow_mask, 9)

    # �����ɫ����������
    m_yellow = cv2.moments(yellow_mask, False)
    try:
        cx_yellow, cy_yellow = m_yellow['m10'] / m_yellow['m00'], m_yellow['m01'] / m_yellow['m00']
    except ZeroDivisionError:
        cx_yellow, cy_yellow = height / 2, width / 2
    # �����ɫ���������
    yellow_area = np.sum(yellow_mask > 0)
    #print(f"Yellow area: {yellow_area}")

    # ������ͼ���ϻ�����ɫ����������
    cv2.circle(yellow_mask, (int(cx_yellow), int(cy_yellow)), 0, (0, 255, 0), 50)
    cv2.imshow('yellow_mask', yellow_mask)


    velocity = [0.0, 0.0]
    yawspeed = 0.0

    deviation_yellow = abs(width / 2 - cx_yellow)
    velocity[0] = max_speed - (max_speed - min_speed) * (deviation_yellow / (width / 2))
    yawspeed = pid_yaw.calculate(0, cx_yellow - width / 2)
    robot.robot_high_control(velocity=velocity, yawSpeed=yawspeed)

    return yellow_area

if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    # # ��ȡ��Ƶ֡��
    # fps = cap.get(cv2.CAP_PROP_FPS)
    # wait_time = int(500 / fps)  # ����ȴ�ʱ��

    count_xiehuo = 0
    flag_xiehuo_over = 0
    count_taijie = 0
    flag_taijie_over = 0
    count_over = 0
    xiehuo_start_time = None  # ж������ʱ����ʼʱ��
    taijie_start_time = None  # ̨������ʱ����ʼʱ��

    while True:
        ret, cv_image = cap.read()
        cv2.imshow('color', cv_image)

        # ����Ѳ��
        current_yellow_area= patrol_line(cv_image)


        mykey = cv2.waitKey(1)

        if mykey & 0xFF == ord('q'):
            break