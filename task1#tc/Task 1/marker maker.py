import numpy as np
import math
import cv2
import cv2.aruco as aruco

markerLength=100;
length=100;

def drawCylinder(img, rvec, tvec, ids, camera_matrix, dist_coeff):
 markerLength = 100
 contours=[]
 contors=[]
 radius = markerLength/2; height = markerLength*1.5
 a = radius; b = radius/1.414 ; c = height
 pts = np.float32([[a,0,0],[b,-b,0],[0,-a,0],[-b,-b,0],[-a,0,0],[-b,b,0],[0,a,0],[b,b,0],[a,0,0],[a,0,c],[b,-b,c],[0,-a,c],[-b,-b,c],[-a,0,c],[-b,b,c],[0,a,c],[b,b,c],[a,0,c]])
 pt_dict = {}
 print(pts)
 imgpts, _ = cv2.projectPoints(pts, rvec, tvec, camera_matrix, dist_coeff)
 for i in range(len(pts)):
  pt_dict[tuple(pts[i])] = tuple(imgpts[i].ravel())
 bt1 = pt_dict[tuple(pts[0])];  bt2 = pt_dict[tuple(pts[1])];
 bt3 = pt_dict[tuple(pts[2])];  bt4 = pt_dict[tuple(pts[3])];
 bt5 = pt_dict[tuple(pts[4])];  bt6 = pt_dict[tuple(pts[5])];
 bt7 = pt_dict[tuple(pts[6])];  bt8 = pt_dict[tuple(pts[7])];
 tt1 = pt_dict[tuple(pts[9])];  tt2 = pt_dict[tuple(pts[10])];
 tt3 = pt_dict[tuple(pts[11])];  tt4 = pt_dict[tuple(pts[12])];
 tt5 = pt_dict[tuple(pts[13])];  tt6 = pt_dict[tuple(pts[14])];
 tt7 = pt_dict[tuple(pts[15])];  tt8 = pt_dict[tuple(pts[16])];
 img = cv2.line(img, bt1, bt5, (0,255,0), 2)
 img = cv2.line(img, bt2, bt6, (0,255,0), 2)
 img = cv2.line(img, bt3, bt7, (0,255,0), 2)
 img = cv2.line(img, bt4, bt8, (0,255,0), 2)
 img = cv2.line(img, tt1, tt5, (0,255,0), 2)
 img = cv2.line(img, tt2, tt6, (0,255,0), 2)
 img = cv2.line(img, tt3, tt7, (0,255,0), 2)
 img = cv2.line(img, tt4, tt8, (0,255,0), 2)
 img = cv2.line(img, bt1, tt1, (0,255,0), 2)
 img = cv2.line(img, bt3, tt3, (0,255,0), 2)
 img = cv2.line(img, bt5, tt5, (0,255,0), 2)
 img = cv2.line(img, bt7, tt7, (0,255,0), 2)
 img = cv2.line(img, tt2, bt2, (0,255,0), 2)
 img = cv2.line(img, tt4, bt4, (0,255,0), 2)
 img = cv2.line(img, tt6, bt6, (0,255,0), 2)
 img = cv2.line(img, tt8, bt8, (0,255,0), 2)
 for i in range(8):
  contours.append(pt_dict[tuple(pts[i])])
 ctr = np.array(contours).reshape((-1,1,2)).astype(np.int32)
 print (ctr)
 img = cv2.drawContours(img, [ctr], -1, (255,0,0), 1)
 for i in range(9,18):
  contors.append(pt_dict[tuple(pts[i])])
 ctnr = np.array(contors).reshape((-1,1,2)).astype(np.int32)
 print (ctnr)
 img = cv2.drawContours(img, [ctnr], -1, (0,255,255), 2)
 

 return img
  

def drawCube(img, rvec, tvec, ids, camera_matrix, dist_coeff):
	
	markerLength = 100
	m = markerLength/2
	pts = np.float32([[-m,m,0],[m,m,0],[-m,-m,0],[-m,m,m],[m,-m,0],[m,-m,m],[-m,-m,m],[m,m,m]])
	pt_dict = {}
	imgpts, _ = cv2.projectPoints(pts, rvec, tvec, camera_matrix, dist_coeff)
	for i in range(len(pts)):
		 pt_dict[tuple(pts[i])] = tuple(imgpts[i].ravel())
	src1 = pt_dict[tuple(pts[0])];   dst1 = pt_dict[tuple(pts[1])];
	dst2 = pt_dict[tuple(pts[2])];  dst3 = pt_dict[tuple(pts[3])];
	src2 = pt_dict[tuple(pts[4])];  dst4 = pt_dict[tuple(pts[5])];
	src3 = pt_dict[tuple(pts[2])];  dst5 = pt_dict[tuple(pts[6])];
	src4 = pt_dict[tuple(pts[1])];  dst6 = pt_dict[tuple(pts[7])];
	img = cv2.line(img, src1, dst1, (0,255,0), 4)
	img = cv2.line(img, src1, dst2, (0,255,0), 4)
	img = cv2.line(img, src1, dst3, (0,255,0), 4)
	img = cv2.line(img, src2, dst2, (0,255,0), 4)
	img = cv2.line(img, src2, dst1, (0,255,0), 4)
	img = cv2.line(img, src2, dst4, (0,255,0), 4)
	img = cv2.line(img, src3, dst5, (0,255,0), 4)
	img = cv2.line(img, src4, dst6, (0,255,0), 4)
	img = cv2.line(img, dst3, dst6, (0,255,0), 4)
	img = cv2.line(img, dst6, dst4, (0,255,0), 4)
	img = cv2.line(img, dst4, dst5, (0,255,0), 4)
	img = cv2.line(img, dst5, dst3, (0,255,0), 4)
	s=(src1[1]+dst2[1]+src2[1]+dst1[1])/4
	print(s)
	

	return img



aruco_dict=aruco.Dictionary_get(aruco.DICT_5X5_250)
#img = aruco.drawMarker(aruco_dict,11,400)

img = cv2.imread("E:\\3D\\eYRTC\\task1#tc\\Task 1\\Task 1.1\\TestCases\\image_6.jpg")
gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
#aruco.dict=aruco.Dictionary_get(aruco.DICT_5x5_250)
parameters=aruco.DetectorParameters_create()
corners,ids,_ = aruco.detectMarkers(gray,aruco_dict,parameters=parameters)

with np.load('E:\\3D\\eYRTC\\task1#tc\\Task 1\\Task 1.1\\Problem Statement\\System.npz') as X:
   camera_matrix, dist_coeff, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]

#print(markerLength)
ret = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeff)
rvec, tvec = ret[0][0,0,:],ret[1][0,0,:]
#print(parameters)
img =drawCylinder(img, rvec, tvec, ids, camera_matrix, dist_coeff)
#print(corners)
#print(ids)
#print(rvec)
#print(tvec)
#print(scr1)
#print(len(corners[0][0][0][1]))
#cop=corners
#for j in cop:
 #  print(j)
#aruco.drawAxis(img, camera_matrix, dist_coeff, rvec, tvec, 50)
cv2.imshow('image',img)
cv2.waitKey(0);
cv2.destroyAllWindows()
