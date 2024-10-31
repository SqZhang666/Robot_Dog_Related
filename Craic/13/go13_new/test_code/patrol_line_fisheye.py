# 涉及到鱼眼相机处理


import cv2
import numpy as np

from prodedure_code.robot_connector import RobotConnector
robot = RobotConnector(ip_address='192.168.123.161', port=8000)

from prodedure_code.PID import PID
pid_yaw = PID(0.1, 1.2, -1.2, 0.018, 0.01, 0.00)
min_speed, max_speed = 0.05, 0.28

#鱼眼相机转化函数
#def undistort_fisheye_image(image, K, D):
#    h, w = image.shape[:2]
#    new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(K, D[:4], (w, h), np.eye(3))
#    undistorted_image = cv2.fisheye.undistortImage(image, K, D[:4], None, new_K)
#    return undistorted_image

def patrol_line_fisheye(fisheye_image):
    #undistorted_image = undistort_fisheye_image(fisheye_image, K, D)

    #height, width, channels = undistorted_image.shape

    #hsv = cv2.cvtColor(undistorted_image, cv2.COLOR_BGR2HSV)

    height, width, channels = fisheye_image.shape

    hsv = cv2.cvtColor(fisheye_image, cv2.COLOR_BGR2HSV)

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

    # Show the original image
    cv2.imshow('Original Fisheye', fisheye_image)
    # Show the undistorted image
    cv2.imshow('Undistorted Fisheye', undistorted_image)
    # Show the processed image
    cv2.imshow('Processed Fisheye', color_mask)

    velocity = [0.0, 0.0]
    yawspeed = 0.0

    deviation = abs(width / 2 - cx)
    velocity[0] = max_speed - (max_speed - min_speed) * (deviation / (width / 2))
    yawspeed = pid_yaw.calculate(0, cx - width / 2)
    #print(velocity[0], yawspeed)

    robot.robot_high_control(velocity=velocity, yawSpeed=yawspeed)


if __name__ == '__main__':
    # Load camera calibration parameters
    K = np.array([[217.30094148, 0.0, 236.61855261],
                  [0.0, 219.20743444, 188.12851091],
                  [0.0, 0.0, 1.0]])
    D = np.array([-0.70487354, 0.74169486, 0.02375571, 0.02432161, -0.42139288])

    cap = cv2.VideoCapture(0)
    while True:
        ret, fisheye_image = cap.read()
        if not ret:
            break
        patrol_line_fisheye(fisheye_image)
        mykey = cv2.waitKey(1)
        if mykey & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()