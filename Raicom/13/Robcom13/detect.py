#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy as np
import cv2
import cv2.aruco as aruco

class detect_aruco():

    def __init__(self):
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()
        # Shape mapping
        self.shape_mapping = {
            1: 'Hemisphere',
            2: 'Cube',
            3: 'Trighgular_Pyramid',
            4: 'Cylinder'
        }

    def detect(self, frame):
        h, w = frame.shape[:2]
        new_width = 1000
        new_height = int(h * (new_width / w))
        frame = cv2.resize(frame, (new_width, new_height))
        (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)
        if len(corners) > 0:
            ids = ids.flatten()
            for (markerCorner, markerID) in zip(corners, ids):
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                cv2.line(frame, topLeft, topRight, (0, 255, 0), 4)
                cv2.line(frame, topRight, bottomRight, (0, 255, 0), 4)
                cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 4)
                cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 4)
                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(frame, (cX, cY), 6, (0, 0, 255), -1)
                shape_name = self.shape_mapping.get(markerID, "未知标记")
                text = f"{markerID}: {shape_name}"
                text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.7, 2)[0]
                text_x = cX - text_size[0] // 2
                text_y = cY + text_size[1] // 2
                cv2.rectangle(frame, (text_x - 5, text_y - text_size[1] - 5),
                              (text_x + text_size[0] + 5, text_y + 5), (0, 0, 0), -1)
                cv2.putText(frame, text, (text_x, text_y),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        # 创建一个可以调整大小的窗口
        cv2.namedWindow('color', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('color', 640, 480)  # 设置初始窗口大小
        cv2.imshow('color', frame)
        key = cv2.waitKey(1) & 0xFF
        return ids


if __name__ == '__main__':
    dt_aruco = detect_aruco()
    cap = cv2.VideoCapture(0)
    while True:
        ret, cv_image = cap.read()
        if ret:
            ids = dt_aruco.detect(cv_image)
        else:
            print("相机打开失败")
