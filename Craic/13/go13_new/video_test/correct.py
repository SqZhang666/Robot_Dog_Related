import cv2
import numpy as np
import glob

def calibrate_camera(image_files, pattern_size=(9, 6)):
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objpoints = []
    imgpoints = []

    for fname in image_files:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        if ret:
            objpoints.append(objp)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            cv2.drawChessboardCorners(img, pattern_size, corners2, ret)
            cv2.imshow('img', img)
            cv2.waitKey(500)
    cv2.destroyAllWindows()
    ret, K, D, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    return K, D

if __name__ == "__main__":
    image_files = glob.glob('calibration_images/*.jpg')
    K, D = calibrate_camera(image_files)
    print("Camera matrix (K):")
    print(K)
    print("\nDistortion coefficients (D):")
    print(D)
