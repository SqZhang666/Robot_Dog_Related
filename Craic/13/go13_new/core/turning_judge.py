import cv2
import numpy as np

def preprocess_image(image):
    # 将图像转换为灰度
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # 高斯模糊
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    # 边缘检测
    edges = cv2.Canny(blurred, 50, 150)
    return edges

def detect_lines(image):
    # 霍夫直线变换检测线条
    lines = cv2.HoughLinesP(image, 1, np.pi/180, threshold=50, minLineLength=50, maxLineGap=10)
    return lines

def draw_lines(image, lines):
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(image, (x1, y1), (x2, y2), (0, 255, 0), 1)

def set_roi(image):
    height, width = image.shape[:2]
    top_left_x = width // 4
    top_left_y = height // 4
    bottom_right_x = 3 * width // 4
    bottom_right_y = 3 * height // 4
    roi = image[top_left_y:bottom_right_y, top_left_x:bottom_right_x]
    return roi


def detect_intersection(lines):
    if lines is None:
        return False

    line_count = len(lines)
    if line_count > 2:
        angles = []
        for line in lines:
            x1, y1, x2, y2 = line[0]
            angle = np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi
            angles.append(angle)

        for i in range(len(angles)):
            for j in range(i + 1, len(angles)):
                angle_diff = abs(angles[i] - angles[j])
                if 30 < angle_diff < 150:
                    return True
    return False

def turning_judge(frame):
    roi = set_roi(frame)
    edges = preprocess_image(roi)
    lines = detect_lines(edges)
    if detect_intersection(lines):
        print("Detected intersection in ROI")
        return 1



if __name__ == '__main__':
    # 使用示例
    cap = cv2.VideoCapture(0)  # 使用摄像头
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if frame is None:
            print("Error: Unable to read the image")
        else:
            roi = set_roi(frame)
        edges = preprocess_image(roi)
        lines = detect_lines(edges)
        if detect_intersection(lines):
            print("Detected intersection in ROI")
        draw_lines(roi, lines)
        cv2.imshow('Frame', frame)
        cv2.imshow('ROI', roi)
        cv2.waitKey(0)
    cv2.destroyAllWindows()
