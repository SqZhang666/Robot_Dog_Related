import cv2
import numpy as np
import socket
import json


class RobotConnector():
    def __init__(self, ip_address='192.168.123.161', port=8000):
        address_server = (ip_address, port)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect(address_server)

    def robot_high_control(self, mode=2, gaitType=1, speedLevel=0,
                           footRaiseHeight=0, bodyHeight=0,
                           euler=[0, 0, 0], velocity=[0, 0], yawSpeed=0.0,
                           reverve=0):
        data = [mode, gaitType, speedLevel, footRaiseHeight, bodyHeight, euler, velocity, yawSpeed, reverve]
        data = json.dumps(data)
        self.s.send(bytes(data.encode('utf-8')))


class PID():
    def __init__(self, dt, max, min, Kp, Kd, Ki):
        self.dt = dt
        self.max = max
        self.min = min
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.integral = 0
        self.pre_error = 0

    def calculate(self, setPoint, pv):

        error = setPoint - pv
        Pout = self.Kp * error
        self.integral += error * self.dt
        Iout = self.Ki * self.integral
        derivative = (error - self.pre_error) / self.dt
        Dout = self.Kd * derivative

        output = Pout + Iout + Dout

        if output > self.max:
            output = self.max
        elif output < self.min:
            output = self.min

        self.pre_error = error
        return output


pid_yaw = PID(0.1, 1.2, -1.2, 0.018, 0.01, 0.00)
pid_yaw2 = PID(0.1, 1.2, -1.2, 0.018, 0.03, 0.00)

min_speed, max_speed = 0.05, 0.28

robot = RobotConnector(ip_address='192.168.123.161', port=8000)


def patrol_line(cv_image):
    height, width, _ = cv_image.shape

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
        cx, cy = width / 2, height / 2

    radius = 0
    color = (0, 255, 0)
    thickness = 50
    cv2.circle(color_mask, (int(cx), int(cy)), radius, color, thickness)

    cv2.imshow('color_mask', color_mask)

    velocity = [0.0, 0.0]
    yawspeed = 0.0

    deviation = abs(width / 2 - cx)
    velocity[0] = max_speed - (max_speed - min_speed) * (deviation / (width / 2))
    yawspeed = pid_yaw.calculate(0, cx - width / 2)
    if deviation > width / 4:
        yawspeed += 0.5 if cx < width / 2 else -0.5
    robot.robot_high_control(velocity=velocity, yawSpeed=yawspeed)


if __name__ == '__main__':
    cap = cv2.VideoCapture(0)
    frame_width2 = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height2 = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = int(cap.get(cv2.CAP_PROP_FPS))
    out = cv2.VideoWriter('output_cam1.avi', cv2.VideoWriter_fourcc(*'XVID'), fps,
                           (frame_width2, frame_height2))
    while True:
        ret, cv_image = cap.read()
        if not ret:
            print("Failed to read frame")
            break

        # patrol_line(cv_image)
        height, width, _ = cv_image.shape

        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        lower_yellow = np.array([0, 38, 0])
        upper_yellow = np.array([87, 255, 255])
        color_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        out.write(color_mask)

        dilate_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
        erode_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        color_mask = cv2.medianBlur(color_mask, 9)
        color_mask = cv2.dilate(color_mask, dilate_kernel)
        color_mask = cv2.erode(color_mask, erode_kernel)

        m = cv2.moments(color_mask, False)
        try:
            cx, cy = m['m10'] / m['m00'], m['m01'] / m['m00']
        except ZeroDivisionError:
            cx, cy = width / 2, height / 2

        radius = 0
        color = (0, 255, 0)
        thickness = 50
        cv2.circle(color_mask, (int(cx), int(cy)), radius, color, thickness)

        cv2.imshow('color_mask', color_mask)
        

        velocity = [0.0, 0.0]
        yawspeed = 0.0

        deviation = abs(width / 2 - cx)
        velocity[0] = max_speed - (max_speed - min_speed) * (deviation / (width / 2))
        yawspeed = pid_yaw.calculate(0, cx - width / 2)
        #velocity[1] = pid_yaw.calculate(0, cx - width / 2)
        print(velocity[0],yawspeed)
        
        robot.robot_high_control(velocity=velocity, yawSpeed=yawspeed)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    out.release()
    cv2.destroyAllWindows()

