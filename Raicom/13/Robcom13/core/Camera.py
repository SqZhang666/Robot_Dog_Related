
import cv2     # 导入OpenCV库，用于处理图像和视频操作
import time    # 导入time库，用于处理时间相关操作
import os      # 导入os库，用于处理操作系统相关功能
import numpy as np   # 导入NumPy库，用于处理数组和矩阵运算

class Camera:      # 定义一个名为Camera的类
    def __init__(self,source):    # 类的初始化方法，参数source默认值为0，表示使用默认摄像头
        #source = "udpsrc address = 192.168.12.14"+"port=9204"      # 此行为注释，可以将source替换为该URL格式以支持网络摄像头
        self.cap = cv2.VideoCapture(source)       # 使用OpenCV创建一个VideoCapture对象，用于捕获视频源
        print(self.cap.isOpened())

    def getframe(self):         # 定义一个名为getframe的方法，用于获取摄像头的每一帧画面
        ret, frame = self.cap.read()     # 读取摄像头的当前帧，返回两个值：ret表示是否成功读取，frame为读取到的图像帧
        return frame           # 返回读取到的图像帧


    def release(self):
        pass

'''
cam = Camera(0)
while(True):
    ret, frame = cam.cap.read()
    #print(ret)
    if ret:
        cv2.imshow('image', frame)
        key = cv2.waitKey(25)       # 等待一段时间，并且检测键盘输入
        if key == ord('q'):         # 若是键盘输入'q',则退出，释放视频
            cam.release()           # 释放视频
            break
    else:
        cam.release()
cv2.destroyAllWindows()             # 关闭所有窗口
'''


