import socket
import json
import time


class RobotConnector():
    def __init__(self, ip_address='192.168.123.161', port=8000):
        address_server = (ip_address, port)
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect(address_server)

    def robot_high_control(self, mode=2, gaitType=1, speedLevel=0,
                           footRaiseHeight=0, bodyHeight=0,
                           euler=[0, 0, 0], velocity=[0, 0], yawSpeed=0.0, reverve=0):
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

        if (output > self.max):
            output = self.max
        elif (output < self.min):
            output = self.min

        self.pre_error = error
        return output


class Patrol_line():

    def __init__(self):
        self.pid_yaw = PID(0.1, 1.2, -1.2, 0.018, 0.01, 0.00)
        self.min_speed, self.max_speed = 0.05, 0.28

    def pt(self, cv_image):
        height, width, channels = cv_image.shape

        # 设置圆形掩码的中心和半径
        center = ((width // 2) + 9, (height // 2) - 10)
        radius = 145
        # 创建圆形掩码
        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.circle(mask, center, radius, (255), thickness=-1)
        # 创建一个三通道的掩码
        mask_3ch = cv2.merge([mask, mask, mask])
        # 应用掩码
        masked_image = cv2.bitwise_and(cv_image, mask_3ch)
        # 创建一个白色背景
        white_background = np.full_like(cv_image, 255)
        # 将圆形区域内的像素保留，其他区域设置为白色
        circular_image = np.where(mask_3ch == 0, white_background, masked_image)
        cv2.imshow('chuli', circular_image)

        # 将掩码应用到HSV图像
        hsv = cv2.cvtColor(circular_image, cv2.COLOR_BGR2HSV)

        # 黄色线条的阈值
        lower_yellow = np.array([0, 38, 0])
        upper_yellow = np.array([87, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)

        # 处理黄色线条的掩码
        dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
        erode_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

        yellow_mask = cv2.dilate(yellow_mask, dilate_kernel)
        yellow_mask = cv2.erode(yellow_mask, erode_kernel)
        yellow_mask = cv2.medianBlur(yellow_mask, 9)

        # 计算黄色线条的质心
        m_yellow = cv2.moments(yellow_mask, False)
        try:
            cx_yellow, cy_yellow = m_yellow['m10'] / m_yellow['m00'], m_yellow['m01'] / m_yellow['m00']
        except ZeroDivisionError:
            cx_yellow, cy_yellow = height / 2, width / 2
        # 计算黄色线条的面积
        yellow_area = np.sum(yellow_mask > 0)
        # print(f"Yellow area: {yellow_area}")

        # 在掩码图像上画出黄色线条的质心
        cv2.circle(yellow_mask, (int(cx_yellow), int(cy_yellow)), 0, (0, 255, 0), 5)
        cv2.imshow('yellow_mask', yellow_mask)

        velocity = [0.0, 0.0]
        yawspeed = 0.0

        deviation_yellow = abs(width / 2 - cx_yellow)
        velocity[0] = self.max_speed - (self.max_speed - self.min_speed) * (deviation_yellow / (width / 2))
        yawspeed = self.pid_yaw.calculate(0, cx_yellow - width / 2)
        robot.robot_high_control(velocity=velocity, yawSpeed=yawspeed)

        return yellow_area


