import numpy as np
import cv2
import cv2.aruco as aruco
import math

img=cv2.imread('E:\\3D\\eYRTC\\task1#tc\\Task 1\\Task 1.1\\TestCases\\image_1.jpg',0)

aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)

parameters = aruco.DetectorParameters_create()

corners, ids, _ = aruco.detectMarkers(img, aruco_dict,parameters = parameters)

with np.load('E:\\3D\\eYRTC\\task1#tc\\Task 1\\Task 1.1\\TestCases\\System.npz') as X:
 camera_matrix, dist_coeff, _, _ = [X[i] for i in
 ('mtx','dist','rvecs','tvecs')]

rvec,tvec= aruco.estimatePoseSingleMarkers(corners,camera_matrix,dist_coeff)

cv2.imshow("img", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
