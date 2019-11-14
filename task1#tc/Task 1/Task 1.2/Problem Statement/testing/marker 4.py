import numpy as np
import cv2
import cv2.aruco as aruco
import math
"""
**************************************************************************
*                  E-Yantra Robotics Competition
*                  ================================
*  This software is intended to check version compatiability of open source software
*  Theme: Thirsty Crow
*  MODULE: Task1.1
*  Filename: detect.py
*  Version: 1.0.0  
*  Date: October 31, 2018
*  
*  Author: e-Yantra Project, Department of Computer Science
*  and Engineering, Indian Institute of Technology Bombay.
*  
*  Software released under Creative Commons CC BY-NC-SA
*
*  For legal information refer to:
*        http://creativecommons.org/licenses/by-nc-sa/4.0/legalcode 
*     
*
*  This software is made available on an “AS IS WHERE IS BASIS”. 
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*  
*  e-Yantra - An MHRD project under National Mission on Education using 
*  ICT(NMEICT)
*
**************************************************************************
"""

####################### Define Utility Functions Here ##########################
"""
Function Name : getCameraMatrix()
Input: None
Output: camera_matrix, dist_coeff
Purpose: Loads the camera calibration file provided and returns the camera and
         distortion matrix saved in the calibration file.
"""
def getCameraMatrix():
	with np.load('E:\\e-yantra\\Task 1\\Task 1.1\\Problem Statement\\System.npz') as X:
		camera_matrix, dist_coeff, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]
	return camera_matrix, dist_coeff

"""
Function Name : sin()
Input: angle (in degrees)
Output: value of sine of angle specified
Purpose: Returns the sine of angle specified in degrees
"""
def sin(angle):
	return math.sin(math.radians(angle))

"""
Function Name : cos()
Input: angle (in degrees)
Output: value of cosine of angle specified
Purpose: Returns the cosine of angle specified in degrees
"""
def cos(angle):
	return math.cos(math.radians(angle))



################################################################################


"""
Function Name : detect_markers()
Input: img (numpy array), camera_matrix, dist_coeff
Output: aruco list in the form [(aruco_id_1, centre_1, rvec_1, tvec_1),(aruco_id_2,
        centre_2, rvec_2, tvec_2), ()....]
Purpose: This function takes the image in form of a numpy array, camera_matrix and
         distortion matrix as input and detects ArUco markers in the image. For each
         ArUco marker detected in image, paramters such as ID, centre coord, rvec
         and tvec are calculated and stored in a list in a prescribed format. The list
         is returned as output for the function
"""
def detect_markers(img, camera_matrix, dist_coeff):
 rs=[]
 rc=[]
 cyl=[]
 markerLength = 100
 m = markerLength/2
 aruco_centre=[]
 aruco_list=[]
 pts = np.float32([[-m,m,0],[m,m,0],[-m,-m,0],[-m,m,m],[m,-m,0],[m,-m,m],[-m,-m,m],[m,m,m]])
 aruco_dict=aruco.Dictionary_get(aruco.DICT_5X5_250)
 gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
 parameters=aruco.DetectorParameters_create()
 corners,ids,_ = aruco.detectMarkers(gray,aruco_dict,parameters=parameters)
 rvec, tvec, ran= aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeff)
 pt_dict = {}
 for j in range(len(ids)):
  imgpts, _ = cv2.projectPoints(pts, rvec[j], tvec[j], camera_matrix, dist_coeff)
  for i in range(len(pts)):
      pt_dict[tuple(pts[i])] = tuple(imgpts[i].ravel())
  src1 = pt_dict[tuple(pts[0])];  dst1 = pt_dict[tuple(pts[1])];
  src2 = pt_dict[tuple(pts[4])]; dst2 = pt_dict[tuple(pts[2])];
  midy=(src1[1]+dst2[1]+src2[1]+dst1[1])/4
  midx=(src1[0]+dst2[0]+src2[0]+dst1[0])/4
  
  np.array(ids).ravel().tolist()
  print(type(ids))
  aruco_id = ids[j]
  aruco_centre.append((midx, midy))
  aruco_list.append((aruco_id, aruco_centre[j], rvec[[j]], tvec[[j]]))
  

                 
 return aruco_list

"""
Function Name : drawAxis()
Input: img (numpy array), aruco_list, aruco_id, camera_matrix, dist_coeff
Output: img (numpy array)
Purpose: This function takes the above specified outputs and draws 3 mutually
         perpendicular axes on the specified aruco marker in the image and
         returns the modified image.
"""
def drawAxis(img, aruco_list, aruco_id, camera_matrix, dist_coeff):
	for x in aruco_list:
		if aruco_id == x[0]:
			rvec, tvec = x[2], x[3]
	markerLength = 100
	m = markerLength/2
	pts = np.float32([[-m,m,0],[m,m,0],[-m,-m,0],[-m,m,m]])
	pt_dict = {}
	imgpts, _ = cv2.projectPoints(pts, rvec, tvec, camera_matrix, dist_coeff)
	for i in range(len(pts)):
		 pt_dict[tuple(pts[i])] = tuple(imgpts[i].ravel())
	src = pt_dict[tuple(pts[0])];   dst1 = pt_dict[tuple(pts[1])];
	dst2 = pt_dict[tuple(pts[2])];  dst3 = pt_dict[tuple(pts[3])];
	
	img = cv2.line(img, src, dst1, (0,255,0), 2)
	img = cv2.line(img, src, dst2, (255,0,0), 2)
	img = cv2.line(img, src, dst3, (0,0,255), 2)
	return img

"""
Function Name : drawCube()
Input: img (numpy array), aruco_list, aruco_id, camera_matrix, dist_coeff
Output: img (numpy array)
Purpose: This function takes the above specified outputs and draws a cube
         on the specified aruco marker in the image and returns the modified
         image.
"""
def drawCube(img, ar_list, ar_id, camera_matrix, dist_coeff):
 
 for x in ar_list:
  if ar_id == x[0]:
   rvec, tvec = x[2], x[3]
 markerLength = 100
 m = markerLength/2
 pts = np.float32([[-m,m,0],[m,m,0],[-m,-m,0],[-m,m,2*m],[m,-m,0],[m,-m,2*m],[-m,-m,2*m],[m,m,2*m]])
 pt_dict = {}
 imgpts, _ = cv2.projectPoints(pts, rvec, tvec, camera_matrix, dist_coeff)
 for i in range(len(pts)):
  pt_dict[tuple(pts[i])] = tuple(imgpts[i].ravel())
 src1 = pt_dict[tuple(pts[0])];   dst1 = pt_dict[tuple(pts[1])];
 dst2 = pt_dict[tuple(pts[2])];  dst3 = pt_dict[tuple(pts[3])];
 src2 = pt_dict[tuple(pts[4])];  dst4 = pt_dict[tuple(pts[5])];
 src3 = pt_dict[tuple(pts[2])];  dst5 = pt_dict[tuple(pts[6])];
 src4 = pt_dict[tuple(pts[1])];  dst6 = pt_dict[tuple(pts[7])];
 img = cv2.line(img, src1, dst1, (0,255,0), 2)
 img = cv2.line(img, src1, dst2, (0,255,0), 2)
 img = cv2.line(img, src1, dst3, (0,255,0), 2)
 img = cv2.line(img, src2, dst2, (0,255,0), 2)
 img = cv2.line(img, src2, dst1, (0,255,0), 2)
 img = cv2.line(img, src2, dst4, (0,255,0), 2)
 img = cv2.line(img, src3, dst5, (0,255,0), 2)
 img = cv2.line(img, src4, dst6, (0,255,0), 2)
 img = cv2.line(img, dst3, dst6, (0,255,0), 2)
 img = cv2.line(img, dst6, dst4, (0,255,0), 2)
 img = cv2.line(img, dst4, dst5, (0,255,0), 2)
 img = cv2.line(img, dst5, dst3, (0,255,0), 2)

 return img

"""
Function Name : drawCylinder()
Input: img (numpy array), aruco_list, aruco_id, camera_matrix, dist_coeff
Output: img (numpy array)
Purpose: This function takes the above specified outputs and draws a cylinder
         on the specified aruco marker in the image and returns the modified
         image.
"""
def drawCylinder(img, ar_list, ar_id, camera_matrix, dist_coeff):
 rs=[]
 s=[]
 trs=[]
 r=[]
 contors=[]
 contours=[]
 for x in ar_list:
  if ar_id == x[0]:
   rvec, tvec = x[2], x[3]
 markerLength = 100
 radius = markerLength/2; height = markerLength*1.5
 a = radius; b = radius/1.414 ; c = height
 pts = np.float32([[a,0,0],[b,-b,0],[0,-a,0],[-b,-b,0],[-a,0,0],[-b,b,0],[0,a,0],[b,b,0],[a,0,0],[a,0,c],[b,-b,c],[0,-a,c],[-b,-b,c],[-a,0,c],[-b,b,c],[0,a,c],[b,b,c],[a,0,c]])
 for x in range(360):
  s =([50*cos(x),50*sin(x),0])
  rs.append(s)
 rs = np.float32(rs)
 for x in range(360):
  r =([50*cos(x),50*sin(x),150])
  trs.append(r)
 trs = np.float32(trs)
 pt_dict = {}
 imgpts, _ = cv2.projectPoints(pts, rvec, tvec, camera_matrix, dist_coeff)
 imgrs, _ = cv2.projectPoints(rs, rvec, tvec, camera_matrix, dist_coeff)
 imgtrs, _ = cv2.projectPoints(trs, rvec, tvec, camera_matrix, dist_coeff)
 for i in range(len(rs)):
  pt_dict[tuple(rs[i])] = tuple(imgrs[i].ravel())
 for i in range(len(trs)):
  pt_dict[tuple(trs[i])] = tuple(imgtrs[i].ravel())
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
 
 for i in range(len(rs)):
  contors.append(pt_dict[tuple(rs[i])])
 ctr = np.array(contors).reshape((-1,1,2)).astype(np.int32)
 #print (ctr)
 img = cv2.drawContours(img, ctr, -1, (255,0,0), 4)
 for i in range(len(trs)):
  contours.append(pt_dict[tuple(trs[i])])
 ctnr = np.array(contours).reshape((-1,1,2)).astype(np.int32)
 print (ctnr)
 img = cv2.drawContours(img, ctnr, -1, (0,255,255), 4)
 return img

"""
MAIN CODE
This main code reads images from the test cases folder and converts them into
numpy array format using cv2.imread. Then it draws axis, cubes or cylinders on
the ArUco markers detected in the images.
"""


if __name__=="__main__":
	cam, dist = getCameraMatrix()
	img = cv2.imread('E:\\e-yantra\\Task 1\\Task 1.1\\TestCases\\image_3.jpg')
	aruco_list = detect_markers(img, cam, dist)
	print(aruco_list)
	for i in aruco_list:
		img = drawAxis(img, aruco_list, i[0], cam, dist)
		img = drawCube(img, aruco_list, i[0], cam, dist)
		img = drawCylinder(img, aruco_list, i[0], cam, dist)
	cv2.imshow("img", img)
	cv2.waitKey(0)
	cv2.destroyAllWindows()
