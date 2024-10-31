import cv2


def save_video():
    # 打开摄像头
    cap = cv2.VideoCapture(1)  # 0代表默认摄像头，你也可以替换成其他设备索引

    # 检查摄像头是否成功打开
    if not cap.isOpened():
        print("无法打开摄像头。")
        return

    # 获取摄像头的帧率和分辨率
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # 设置新的帧率值
    frame_rate = 20  # 这里设置为20帧/秒，你可以根据需要修改

    # 创建VideoWriter对象
    out = None
    recording = False

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            print("无法捕获帧。")
            break

        # 显示帧
        cv2.imshow('Frame', frame)

        # 按下 'c' 键开始录制视频
        key = cv2.waitKey(33) & 0xFF  # 延迟33毫秒，对应于大约30帧/秒的帧率
        if key == ord('c'):
            if not recording:
                out = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'XVID'), frame_rate,
                                      (frame_width, frame_height))
                recording = True
                print("开始录制视频。")
            else:
                print("视频已经在录制中。")

        # 按下 's' 键停止录制视频
        elif key == ord('s'):
            if recording:
                out.release()
                recording = False
                print("录制视频已停止。")
            else:
                print("当前未录制视频。")

        # 录制视频
        if recording:
            out.write(frame)

        # 按下 'q' 键退出循环
        if key == ord('q'):
            break

    # 释放摄像头和关闭窗口
    cap.release()
    if recording:
        out.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    save_video()
