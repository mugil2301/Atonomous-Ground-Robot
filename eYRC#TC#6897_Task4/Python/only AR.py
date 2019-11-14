import numpy as np
import cv2
import cv2.aruco as aruco
import math
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
from PIL import Image
import pygame
from objloader import *

#texture_object = None
texture_background = None
camera_matrix = None
dist_coeff = None
cap = cv2.VideoCapture(1)
stone= None
pot = None
pot1 = None
pot2 = None
crow = None
#img = frame
INVERSE_MATRIX = np.array([[ 1.0, 1.0, 1.0, 1.0],
                           [-1.0,-1.0,-1.0,-1.0],
                           [-1.0,-1.0,-1.0,-1.0],
                           [ 1.0, 1.0, 1.0, 1.0]])

"""
Function Name : getCameraMatrix()
Input: None
Output: camera_matrix, dist_coeff
Purpose: Loads the camera calibration file provided and returns the camera and
         distortion matrix saved in the calibration file.
"""
def getCameraMatrix():
    global camera_matrix, dist_coeff
    with np.load('Camera.npz') as X:
        camera_matrix, dist_coeff, _, _ = [X[i] for i in ('mtx','dist','rvecs','tvecs')]


"""
Function Name : init_gl()
Input: None
Output: None
Purpose: Initialises various parameters related to OpenGL scene.
"""  
def init_gl():
    global texture_object, texture_background,stone,pot1,pot2,crow,pot
    glClearColor(0.0, 0.0, 0.0, 0.0) #Red, green, blue, alpha values for clear colour buffer
    glClearDepth(1.0) #Clear depth buffer value
    glDepthFunc(GL_LESS) #specifies the function used to compare each incoming pixel depth value with the depth value present in the depth buffer
    glEnable(GL_DEPTH_TEST)
    glShadeModel(GL_SMOOTH) #Smooth shading technique 
    glMatrixMode(GL_PROJECTION)
    #glEnable(GL_DEPTH_TEST)
    glEnable(GL_LIGHTING) 
    glEnable(GL_LIGHT0) 
    gluPerspective(33.7, 1.3, 0.1, 100.0) #Perspective projection:  fov angle, aspect ratio, near distance, far distance 
    glMatrixMode(GL_MODELVIEW) 
     
    #enable textures
    glEnable(GL_TEXTURE_2D)
    texture_background = glGenTextures(1)
    texture_object = glGenTextures(1)
    stone = OBJ('stone.obj', swapyz=True)
    stone1 = OBJ('stone2.obj', swapyz=True)
    crow = OBJ('Crow1.obj', swapyz=True)
    pot = OBJ('claypotu.obj', swapyz=True)
    pot1 = OBJ('claypot14.obj', swapyz=True)
    pot2 = OBJ('claypotf.obj', swapyz=True)
    pot3 = OBJ('claypot1.obj', swapyz=True)
    

"""
Function Name : resize()
Input: None
Output: None
Purpose: Initialises the projection matrix of OpenGL scene
"""
def resize(w,h):
    ratio = 1.0* w / h
    glViewport(0,0,w,h)
    gluPerspective(45, ratio, 0.1, 100.0)
    glMatrixMode(GL_MODELVIEW)

"""
Function Name : drawGLScene()
Input: None
Output: None
Purpose: It is the main callback function which is called again and
         again by the event processing loop. In this loop, the webcam frame
         is received and set as background for OpenGL scene. ArUco marker is
         detected in the webcam frame and 3D model is overlayed on the marker
         by calling the overlay() function.
"""
def drawGLScene():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    ar_list = []
    aruco_list = []
    ret, frame = cap.read()
    cv2.waitKey(1)
    draw_background(frame)
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    aruco_list = detect_markers(frame)
    for j in range(len(aruco_list)):
        ar_list.append(aruco_list[j])
        for i in ar_list:
            if i[0] == 0:
                overlay(frame, ar_list, i[0])
            elif i[0] == 1:
                overlay(frame, ar_list, i[0])
            elif i[0] == 2:
                overlay(frame, ar_list, i[0])
            elif i[0] == 3:
                overlay(frame, ar_list, i[0])
            elif i[0] == 4:
                overlay(frame, ar_list, i[0])    
            elif i[0] == 5:
                overlay(frame, ar_list, i[0])
            elif i[0] == 6:
                overlay(frame, ar_list, i[0])
            elif i[0] == 7:
                overlay(frame, ar_list, i[0])
            elif i[0] == 8:
                overlay(frame, ar_list, i[0])    
            elif i[0] == 9:
                overlay(frame, ar_list, i[0])
            elif i[0] == 10:
                overlay(frame, ar_list, i[0])    
    cv2.imshow('frame', frame)
    cv2.waitKey(1)
    glutSwapBuffers()


"""
Function Name : detect_markers()
Input: img (numpy array)
Output: aruco list in the form [(aruco_id_1, centre_1, rvec_1, tvec_1),(aruco_id_2,
        centre_2, rvec_2, tvec_2), ()....]
Purpose: This function takes the image in form of a numpy array, camera_matrix and
         distortion matrix as input and detects ArUco markers in the image. For each
         ArUco marker detected in image, paramters such as ID, centre coord, rvec
         and tvec are calculated and stored in a list in a prescribed format. The list
         is returned as output for the function
"""

def detect_markers(img):
    global camera_matrix
    global dist_coeff
    markerLength = 100
    aruco_centre = []
    aruco_list = []
    aruco_dict = aruco.Dictionary_get(aruco.DICT_5X5_250)
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    parameters = aruco.DetectorParameters_create()
    corners,ids,_ = aruco.detectMarkers(gray,aruco_dict,parameters=parameters)
    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeff)
    if ids is not None:
        for j in range(len(ids)):
            mid = ( corners[j][0][0] + corners[j][0][1] + corners[j][0][2] + corners[j][0][3] )/4
            aruco_id = ids[j]
            aruco_centre.append((mid[0], mid[1]))
            aruco_list.append((aruco_id, aruco_centre[j], rvec[[j]], tvec[[j]]))
            #print(aruco_id)
    return aruco_list


"""
Function Name : draw_background()
Input: img (numpy array)
Output: None
Purpose: Takes image as input and converts it into an OpenGL texture. That
         OpenGL texture is then set as background of the OpenGL scene
"""
def draw_background(image):
    glLoadIdentity()
    # convert image to OpenGL texture format
    bg_image = cv2.flip(image, 0)   #flip image horizontally
    bg_image = Image.fromarray(bg_image)     
    ix = bg_image.size[0]
    iy = bg_image.size[1]
    bg_image = bg_image.tobytes("raw", "BGRX", 0, -1)#Returns the data in the buffer as a string
    glMatrixMode(GL_MODELVIEW)

    # create background texture
    glBindTexture(GL_TEXTURE_2D, texture_background)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
    glTexImage2D(GL_TEXTURE_2D, 0, 3, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, bg_image)
    glBindTexture(GL_TEXTURE_2D, texture_background)
    glPushMatrix()#duplicating the current matrix
    glTranslatef(0.0,0.0,-10.0)#multiply the current matrix by a translation matrix
    # draw background
    glBegin(GL_QUADS)
    glTexCoord2f(0.0, 1.0); glVertex3f(-4.0, -3.0, 0.0)
    glTexCoord2f(1.0, 1.0); glVertex3f( 4.0, -3.0, 0.0)
    glTexCoord2f(1.0, 0.0); glVertex3f( 4.0,  3.0, 0.0)
    glTexCoord2f(0.0, 0.0); glVertex3f(-4.0,  3.0, 0.0)
    glEnd( )
    glPopMatrix()

    return None
"""
Function Name : init_object_texture()
Input: Image file path
Output: None
Purpose: Takes the filepath of a texture file as input and converts it into OpenGL
         texture. The texture is then applied to the next object rendered in the OpenGL
         scene.
"""
def init_object_texture(image):
    image = cv2.imread(image)
     

    # convert image to OpenGL texture format
    image = Image.fromarray(image)     
    ix = image.size[0]
    iy = image.size[1]
    image = image.tobytes("raw", "BGRX", 0, -1)
    glMatrixMode(GL_MODELVIEW)
    # create object texture
    glBindTexture(GL_TEXTURE_2D, texture_object)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
    glTexImage2D(GL_TEXTURE_2D, 0, 3, ix, iy, 0, GL_RGBA, GL_UNSIGNED_BYTE, image)

    return None

"""
Function Name : overlay()
Input: img (numpy array), aruco_list, aruco_id, texture_file (filepath of texture file)
Output: None
Purpose: Receives the ArUco information as input and overlays the 3D Model of a teapot
         on the ArUco marker. That ArUco information is used to
         calculate the rotation matrix and subsequently the view matrix. Then that view matrix
         is loaded as current matrix and the 3D model is rendered.

         Parts of this code are already completed, you just need to fill in the blanks. You may
         however add your own code in this function.
"""
def overlay(frame, ar_list, ar_id):
    i = 1
    for x in ar_list:
        if ar_id == x[0]:
            centre, rvec, tvec = x[1], x[2], x[3]
            rmtx = cv2.Rodrigues(rvec)[0]
            #tx=(tvec[0][0][0]*0.00282)+((tvec[0][0][1]*tvec[0][0][1])/(1000*305))
            tx=(tvec[0][0][0]*0.00282)+1.1
            ty=(tvec[0][0][1]/392.14)+0.1
            tz=(tvec[0][0][2]*0.00443)-0.2
            view_matrix = np.array([[rmtx[0][0],rmtx[0][1],rmtx[0][2],tx],
                                    [rmtx[1][0],rmtx[1][1],rmtx[1][2],ty],
                                    [rmtx[2][0],rmtx[2][1],rmtx[2][2],tz],
                                    [0.0       ,0.0       ,0.0       ,1.0    ]])
            view_matrix = view_matrix * INVERSE_MATRIX
            view_matrix = np.transpose(view_matrix)
            
            draw_background(frame)           
            #print(x[0])
            glPushMatrix()
            glLoadMatrixd(view_matrix)
            
            if x[0] == 0  :
                    glCallList(pot.gl_list)
            
            elif x[0] == 2 :
                    glCallList(stone.gl_list)
            elif x[0] == 4 :
                    glCallList(stone.gl_list)
            elif x[0] == 6 :
                    glCallList(stone.gl_list)        
            elif x[0] == 10 :
                    glCallList(crow.gl_list)
                    
            glPopMatrix()
    i=i+1
    print(tvec)

"""
Function Name : main()
Input: None
Output: None
Purpose: Initialises OpenGL window and callback functions. Then starts the event
         processing loop.
"""        
def main():
    i = 0
    glutInit()
    getCameraMatrix()
    glutInitWindowSize(640, 480)
    glutInitWindowPosition(625, 100)
    glutInitDisplayMode(GLUT_RGB | GLUT_DEPTH | GLUT_DOUBLE)
    window_id = glutCreateWindow("OpenGL")
    init_gl()
    glutDisplayFunc(drawGLScene)
    glutIdleFunc(drawGLScene)
    glutReshapeFunc(resize)
    i=i+1
    print(i)
    glutMainLoop()
    print('ha ha ')
    
if __name__ == "__main__":
    main()
