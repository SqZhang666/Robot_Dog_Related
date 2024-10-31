import cv2
import numpy as np
from core.robot_connector import RobotConnector
import argparse

#robot = RobotConnector(ip_address='192.168.123.161', port=8000)
from core.PID import PID
from DetectArucotag import detect_aruco
import time
import socket
min_speed, max_speed = 0.05, 0.28


pid_yaw = PID(0.1, 1.2, -1.2, 0.018, 0.01, 0.00)
min_speed, max_speed = 0.05, 0.28

def find_connected_component_containing_point(image, points):
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(image, connectivity=8)
    for point in points:
        image = cv2.circle(image, point, 0, (0, 255, 0), 10)
    cv2.imshow('find_connected_components_containing_points', image)
    cv2.waitKey(1000)
    found_labels = set()
    for label in range(1, num_labels):
        for point in points:
            x, y = point
            if labels[y, x] == label:
                found_labels.add(label)
                break
    output_image = np.zeros_like(image)
    for label in found_labels:
        output_image[labels == label] = 255
    return output_image


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


def detect_yellow_line(circular_image):
    height, width, _ = circular_image.shape
    # 400 480
    circular_image = circular_image[:height//2, :]
    #height, width, _ = circular_image.shape
    # (200, 480, 3)
    hsv = cv2.cvtColor(circular_image, cv2.COLOR_BGR2HSV)
    lower_yellow = np.array([0, 7, 0])
    upper_yellow = np.array([68, 255, 255])
    #lower_yellow = np.array([0, 64, 0])
    #upper_yellow = np.array([90, 255, 255])
    yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    #dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (15, 15))
    #erode_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    #yellow_mask = cv2.dilate(yellow_mask, dilate_kernel)
    #yellow_mask = cv2.erode(yellow_mask, erode_kernel)
    #yellow_mask = cv2.medianBlur(yellow_mask, 9)
    dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (1, 1))
    erode_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    yellow_mask = cv2.dilate(yellow_mask, dilate_kernel)
    yellow_mask = cv2.erode(yellow_mask, erode_kernel)
    yellow_mask = cv2.medianBlur(yellow_mask, 5)
    #points = [(230, 220), (260, 220)]
    points = [(230, 195), (260, 195)]
    yellow_mask = find_connected_component_containing_point(yellow_mask, points)
    m_yellow = cv2.moments(yellow_mask, False)
    try:
        cx_yellow, cy_yellow = m_yellow['m10'] / m_yellow['m00'], m_yellow['m01'] / m_yellow['m00']
    except ZeroDivisionError:
        cx_yellow, cy_yellow = height / 2, width / 2
    yellow_area = np.sum(yellow_mask > 0)
    # 236.7685759507208 130.35248320999273
    cv2.circle(yellow_mask, (int(cx_yellow), int(cy_yellow)), 0, (0, 255, 0), 20)
    cv2.imshow('yellow_mask', yellow_mask)
    cv2.waitKey(1000)
    return yellow_area, cx_yellow, cy_yellow

def patrol_line(cv_image):
    height, width, _ = cv_image.shape
    circular_image = preprocess_image(cv_image)
    #cv2.imshow('chuli', circular_image)
    yellow_area, cx_yellow, cy_yellow = detect_yellow_line(circular_image)
    #aruco_ids = dt_aruco.detect(cv_image)
    deviation_yellow = abs(width / 2 - cx_yellow)
    velocity = [0.0, 0.0]
    velocity[0] = max_speed - (max_speed - min_speed) * (deviation_yellow / (width / 2))
    yawspeed = pid_yaw.calculate(0, cx_yellow - width / 2)
    #robot.robot_high_control(velocity=velocity, yawSpeed=yawspeed)
    return yellow_area

image_path = "C:\\Users\\zsq\\Desktop\\calibration_image_04.jpg" # 替换为你的图像路径

image = cv2.imread(image_path)
patrol_line(image)
