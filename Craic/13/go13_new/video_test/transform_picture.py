import cv2
import numpy as np


def undistort_fisheye_image(image, K, D):
    h, w = image.shape[:2]
    new_K, _ = cv2.getOptimalNewCameraMatrix(K, D[:4], (w, h), 1, (w, h))
    undistorted_image = cv2.fisheye.undistortImage(image, K, D[:4], None, new_K)
    return undistorted_image


if __name__ == "__main__":
    # Load camera calibration parameters
    K = np.array([[181.93955661, 0.0, 245.56273479],
                  [0.0, 181.50127835, 202.98849684],
                  [0.0, 0.0, 1.0]])
    D = np.array([-0.52227619, 0.44090531, -0.00826143, 0.00313807, -0.18603566])

    # Load fisheye image
    fisheye_image = cv2.imread('C:\\Users\\a\\Desktop\\GO_Robcom\\ceshi\\calibration_images\\calibration_image_14.jpg')  # Replace with your actual fisheye image

    # Undistort the fisheye image
    undistorted_image = undistort_fisheye_image(fisheye_image, K, D)

    # Display the original and undistorted images
    cv2.imshow('Original Fisheye Image', fisheye_image)
    cv2.imshow('Undistorted Image', undistorted_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
