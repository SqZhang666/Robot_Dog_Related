import cv2
import numpy as np


def undistort_fisheye_image(image, K, D):
    h, w = image.shape[:2]
    new_K, roi = cv2.getOptimalNewCameraMatrix(K, D[:4], (w, h), 1, (w, h))
    undistorted_image = cv2.fisheye.undistortImage(image, K, D[:4], None, new_K)
    x, y, w, h = roi
    undistorted_image = undistorted_image[y:y + h, x:x + w]
    return undistorted_image


if __name__ == "__main__":
    # Load camera calibration parameters
    K = np.array([[181.93955661, 0.0, 245.56273479],
                  [0.0, 181.50127835, 202.98849684],
                  [0.0, 0.0, 1.0]])
    D = np.array([-0.52227619, 0.44090531, -0.00826143, 0.00313807, -0.18603566])

    # Open video capture
    cap = cv2.VideoCapture('C:\\Users\\a\\Desktop\\GO_Robcom\\ceshi\\视频\\序列 01.mp4')  # Replace with your actual video file
    cv2.namedWindow('Undistorted Frame', cv2.WINDOW_NORMAL)
    cv2.resizeWindow('Undistorted Frame', 800, 600)  # Set window size to 800x600

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Undistort the fisheye image
        undistorted_frame = undistort_fisheye_image(frame, K, D)

        # Display the undistorted frame
        cv2.imshow('Undistorted Frame', undistorted_frame)

        # Check for exit key
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release video capture and close windows
    cap.release()
    cv2.destroyAllWindows()
