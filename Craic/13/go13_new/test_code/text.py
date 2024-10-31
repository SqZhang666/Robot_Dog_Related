# ÊµÑé´úÂë

import cv2
import numpy as np

from prodedure_code.robot_connector import RobotConnector

robot = RobotConnector(ip_address='192.168.123.161', port=8000)

from prodedure_code.PID import PID

pid_yaw = PID(0.1, 1.2, -1.2, 0.018, 0.01, 0.00)
min_speed, max_speed = 0.05, 0.28

def patrol_line(cv_image):
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

    velocity = [0.0, 0.0]
    yawspeed = 0.0

    deviation = abs(width / 2 - cx)
    velocity[0] = max_speed - (max_speed - min_speed) * (deviation / (width / 2))
    yawspeed = pid_yaw.calculate(0, cx - width / 2)
    print(velocity[0], yawspeed)
    robot.robot_high_control(velocity=velocity, yawSpeed=yawspeed)

    return color_mask

if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    out = None
    recording = False

    while True:
        ret, cv_image = cap.read()
        if not ret:
            print("Failed to capture image, retrying...")
            continue

        processed_image = patrol_line(cv_image)

        cv2.imshow('Original Frame', cv_image)
        cv2.imshow('Processed Frame', processed_image)

        mykey = cv2.waitKey(1) & 0xFF

        if mykey == ord('q'):
            break

        if mykey == ord('c'):
            if not recording:
                frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                frame_rate = int(cap.get(cv2.CAP_PROP_FPS))
                out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'XVID'), frame_rate, (frame_width, frame_height))
                recording = True
                print("Started recording.")
            else:
                print("Recording already in progress.")

        if recording:
            out.write(cv_image)

    cap.release()
    if recording:
        out.release()
    cv2.destroyAllWindows()
