import cv2
import os

def capture_calibration_images(output_dir, camera_id=1):
    """
    捕获校准图像，每次按 'c' 键保存一张图片，图片保存在指定路径下。

    Args:
    output_dir (str): 图片保存的目录。
    camera_id (int): 使用的摄像头的ID。
    """
    # 创建输出目录（如果不存在）
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # 打开摄像头
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print("Error: 无法打开摄像头。")
        return

    print("按 'c' 键捕获图像，按 'q' 键退出。")
    image_count = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("无法捕获帧。")
            break

        # 显示当前帧
        cv2.imshow('frame', frame)

        key = cv2.waitKey(1) & 0xFF

        # 按 'c' 键捕获图像并保存
        if key == ord('c'):
            image_path = os.path.join(output_dir, f'calibration_image_{image_count:02d}.jpg')
            cv2.imwrite(image_path, frame)
            print(f"图片已保存至 {image_path}")
            image_count += 1

        # 按 'q' 键退出
        elif key == ord('q'):
            break

    # 释放摄像头并关闭窗口
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    # 输出目录
    output_dir = 'calibration_images'
    # 捕获校准图像
    capture_calibration_images(output_dir)
