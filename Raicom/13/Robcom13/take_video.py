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

min_speed, max_speed = 0.05, 0.28

robot = RobotConnector(ip_address='192.168.123.161', port=8000)


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
            
        out.write(cv_image)
        
        # patrol_line(cv_image)
        height, width, channels = cv_image.shape

        #
        center = ((width // 2) + 9, (height // 2) - 10)
        radius = 145
      
        mask = np.zeros((height, width), dtype=np.uint8)
        cv2.circle(mask, center, radius, (255), thickness=-1)
     
        mask_3ch = cv2.merge([mask, mask, mask])
    
        masked_image = cv2.bitwise_and(cv_image, mask_3ch)
      
        white_background = np.full_like(cv_image, 255)
      
        circular_image = np.where(mask_3ch == 0, white_background, masked_image)
        cv2.imshow('chuli', circular_image)
    
        hsv = cv2.cvtColor(circular_image, cv2.COLOR_BGR2HSV)
    
 
        lower_yellow = np.array([0, 38, 0])
        upper_yellow = np.array([87, 255, 255])
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    
     
        dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (10, 10))
        erode_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    
        yellow_mask = cv2.dilate(yellow_mask, dilate_kernel)
        yellow_mask = cv2.erode(yellow_mask, erode_kernel)
        yellow_mask = cv2.medianBlur(yellow_mask, 9)
    
  
        m_yellow = cv2.moments(yellow_mask, False)
        try:
            cx_yellow, cy_yellow = m_yellow['m10'] / m_yellow['m00'], m_yellow['m01'] / m_yellow['m00']
        except ZeroDivisionError:
            cx_yellow, cy_yellow = height / 2, width / 2
    
        yellow_area = np.sum(yellow_mask > 0)
        #print(f"Yellow area: {yellow_area}")
    
        # 
        cv2.circle(yellow_mask, (int(cx_yellow), int(cy_yellow)), 0, (0, 255, 0), 50)
        cv2.imshow('yellow_mask', yellow_mask)
    
    
        velocity = [0.0, 0.0]
        yawspeed = 0.0
    
        deviation_yellow = abs(width / 2 - cx_yellow)
        velocity[0] = max_speed - (max_speed - min_speed) * (deviation_yellow / (width / 2))
        yawspeed = pid_yaw.calculate(0, cx_yellow - width / 2)
        robot.robot_high_control(velocity=velocity, yawSpeed=yawspeed)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    out.release()
    cv2.destroyAllWindows()

