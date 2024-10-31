import argparse
import time
import cv2
import sys

# 构建参数解析器并解析参数
ap = argparse.ArgumentParser()
ap.add_argument("-t", "--type", type=str, default="DICT_ARUCO_ORIGINAL", help="Type of ArUCo tag to detect")
args = vars(ap.parse_args())

# 定义 OpenCV 支持的每个可能的 ArUCo 标签的名称
ARUCO_DICT = {"DICT_4X4_50": cv2.aruco.DICT_4X4_50, "DICT_4X4_100": cv2.aruco.DICT_4X4_100,
              "DICT_4X4_250": cv2.aruco.DICT_4X4_250, "DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
              "DICT_5X5_50": cv2.aruco.DICT_5X5_50, "DICT_5X5_100": cv2.aruco.DICT_5X5_100,
              "DICT_5X5_250": cv2.aruco.DICT_5X5_250, "DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
              "DICT_6X6_50": cv2.aruco.DICT_6X6_50, "DICT_6X6_100": cv2.aruco.DICT_6X6_100,
              "DICT_6X6_250": cv2.aruco.DICT_6X6_250, "DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
              "DICT_7X7_50": cv2.aruco.DICT_7X7_50, "DICT_7X7_100": cv2.aruco.DICT_7X7_100,
              "DICT_7X7_250": cv2.aruco.DICT_7X7_250, "DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
              "DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
              "DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
              "DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
              "DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
              "DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11}

# 验证提供的 ArUCo 标签是否存在并且受 OpenCV 支持
if ARUCO_DICT.get(args["type"], None) is None:
    print("[INFO] ArUCo tag of '{}' is not supported".format(args["type"]))
    sys.exit(0)

# 加载 ArUCo 字典并获取 ArUCo 参数
print("[INFO] Detecting '{}' tags...".format(args["type"]))
arucoDict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT[args["type"]])
arucoParams = cv2.aruco.DetectorParameters_create()

# 初始化视频流并让相机预热
print("[INFO] Starting video stream...")
vs = cv2.VideoCapture(0)

# 循环视频流中的帧
while True:

    ret, frame = vs.read()
    print(ret)

    h, w = frame.shape[:2]
    new_width = 1000
    new_height = int(h * (new_width / w))
    frame = cv2.resize(frame, (new_width, new_height))
    # 检测输入帧中的 ArUco 标记
    (corners, ids, rejected) = cv2.aruco.detectMarkers(frame, arucoDict, parameters=arucoParams)
    # 验证*至少*一个 ArUco 标记被检测到
    if len(corners) > 0:
        # 展平 ArUco ID 列表
        ids = ids.flatten()
        # 循环检测到的 ArUCo 角
        for (markerCorner, markerID) in zip(corners, ids):
            # 提取标记角（始终按左上角、右上角、右下角和左下角顺序返回）
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners
            # 将每个 (x, y) 坐标对转换为整数
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))
            # 绘制ArUCo检测的边界框
            cv2.line(frame, topLeft, topRight, (0, 255, 0), 2)
            cv2.line(frame, topRight, bottomRight, (0, 255, 0), 2)
            cv2.line(frame, bottomRight, bottomLeft, (0, 255, 0), 2)
            cv2.line(frame, bottomLeft, topLeft, (0, 255, 0), 2)
            # 计算并绘制 ArUco 标记的中心 (x, y) 坐标
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(frame, (cX, cY), 4, (0, 0, 255), -1)
            # 在框架上绘制 ArUco 标记 ID
            cv2.putText(frame, str(markerID), (topLeft[0], topLeft[1] - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    # 显示输出帧
    cv2.imshow("Frame", frame)
    key = cv2.waitKey(1) & 0xFF
    # 如果按下了 `q` 键，则中断循环
    if key == ord("q"):
        break

# 做一些清理
vs.release()
cv2.destroyAllWindows()

