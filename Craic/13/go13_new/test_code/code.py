import time
import cv2
from core.functions import PID, intersection_Judge, patrol_line, RobotConnector

min_speed, max_speed = 0.05, 0.28
def right_deliver():
    t = 0
    while True:
        t += 1
        if 0 < t <= 5:
            print("right")
            turn_velocity = [0.0, max_speed]
            turn_yawspeed = 0
            robot.robot_high_control(velocity=turn_velocity, yawSpeed=turn_yawspeed)
            time.sleep(0.1)
        elif 5 < t <= 13:
            motiontime = 0
            while True:
                time.sleep(0.002)
                motiontime = motiontime + 1
                if 0 < motiontime < 5:
                    robot.robot_high_control(
                        mode=1,
                        euler=[0.0, -0.5, 0.0],
                        bodyHeight=0.2,
                        footRaiseHeight=0.0,
                        velocity=[0.0, 0.0],
                        yawSpeed=0.0
                    )
                    print(1)
                    time.sleep(0.1)

                elif 5 <= motiontime < 8:
                    robot.robot_high_control(
                        mode=1,
                        euler=[0.0, -1, 0.0],
                        bodyHeight=0.2,
                        footRaiseHeight=0.0,
                        velocity=[0.0, 0.0],
                        yawSpeed=0.0
                    )
                    print(2)
                    time.sleep(1)
                else:
                    break
        elif 13 < t <= 18:
            print("left")
            turn_velocity = [0.0, -max_speed]
            turn_yawspeed = 0
            robot.robot_high_control(velocity=turn_velocity, yawSpeed=turn_yawspeed)
            time.sleep(0.1)
        else:
            return
            


if __name__ == '__main__':
    robot = RobotConnector(ip_address='192.168.123.161', port=8000)
    right_deliver()
