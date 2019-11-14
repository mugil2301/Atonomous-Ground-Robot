from collections import deque
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
import threading
import time
import serial
'''
* Team Id : 6897
* Author List : Nitin Kasshyap, Mugil Raj, Yoogesh Kumar, Saran
* Filename: 
* Theme: thirsty Crow
* Functions: getCameraMatrix,init_gl,resize,drawGLScene,draw_background,init_object_texture,get_info,get_serial,
*            put_serial,bfs,shortestPath,configuration,direct,node_action,best_route,arena_conf,main
* Global Variables: n,adj,arena,swap,traverse,g,visited,dist,pred,axis_new,queue,node,psnode,pvnode,pitcher,cell_list,
*                   orient_list,c,path,texture_background,camera_matrix,dist_coeff,cap,stonehalf,pot,travel_index,INVERSE_MATRIX,
*                   ser,arena_config,Robot_start,arena_aruco,change_ar,pot_visit
'''

class CircularQueue:

    #Constructor
    def __init__(self):
        self.queue = list()
        self.head = 0
        self.tail = 0
        self.maxSize = 54
        
    def reset(self):
        self.queue = list()
        self.head = 0
        self.tail = 0
        self.maxSize = 54
        return


    #Adding elements to the queue
    def enqueue(self,data):
        if self.size() == self.maxSize-1:
            return ("Queue Full!")
        self.queue.append(data)
        self.tail = (self.tail + 1) % self.maxSize
        return True

    #Removing elements from the queue
    def dequeue(self):
        if self.size()==0:
            return ("Queue Empty!") 
        data = self.queue[self.head]
        self.head = (self.head + 1) % self.maxSize
        return data

    #Calculating the size of the queue
    def size(self):
        if self.tail>=self.head:
            return (self.tail-self.head)
        return (self.maxSize - (self.head-self.tail))

n = 54
adj = [[2,4,-1], [1,5,-1], [4,8,-1], [1,3,9], [2,6,10], [5,11,-1], [8,13,-1], [3,7,14], [4,10,15], [5,9,16], [6,12,17], [11,18,-1], [7,19,-1], [8,15,20], [9,14,21], [10,17,22], [11,16,23], [12,24,-1], [13,20,25], [14,19,26], [15,22,27], [16,21,28], [17,24,29], [18,23,30], [19,31,-1], [20,27,32], [21,26,33], [22,29,34], [23,28,35], [24,36,-1], [25,32,37], [26,31,38], [27,34,39], [28,33,40], [29,36,41], [30,35,42], [31,43,-1], [32,39,44], [33,38,45], [34,41,46], [35,40,47], [36,48,-1], [37,44,-1], [38,43,49], [39,46,50], [40,45,51], [41,48,52], [42,47,-1], [44,50,-1], [45,49,53], [46,52,54], [47,51,-1], [50,54,-1], [51,53,-1]];
arena = [[2,9,4,5,1,10], [4,14,8,9,3,15], [10,21,15,16,9,22], [6,16,10,11,5,17], [8,19,13,14,7,20], [15,26,20,21,14,27], [22,33,27,28,21,34], [17,28,22,23,16,29], [12,23,17,18,11,24], [20,31,25,26,19,32], [27,38,32,33,26,39], [34,45,39,40,33,46], [29,40,34,35,28,41], [24,35,29,30,23,36], [32,43,37,38,31,44], [39,49,44,45,38,50], [46,53,50,51,45,54], [41,51,46,47,40,52], [36,47,41,42,35,48]]
swap = [43, 49, 53, 54, 52, 48, 42 , 30, 18]
traverse = [];
g = 0
change = ['s','i','e']


change_ar = [-1]*10
pot_visit = 0
visited = [0] * n
dist = [-1] * n
pred = [-1] * n
axis_new = 0
#path = [-1] * 2*n
queue = CircularQueue()
node = 0
psnode = 0
pvnode = 0
pvnodex = 0
pitcher= []
cell_list = []
orient_list = []
c = 0
path = [[-1 for i in range(n)] for j in range(2)]
texture_background = None
camera_matrix = None
dist_coeff = None
cap = cv2.VideoCapture(1)
stone = None
stone2 = None
pot = None
pot1 = None
pot2 = None
crow = None
#img = frame
INVERSE_MATRIX = np.array([[ 1.0, 1.0, 1.0, 1.0],
                           [-1.0,-1.0,-1.0,-1.0],
                           [-1.0,-1.0,-1.0,-1.0],
                           [ 1.0, 1.0, 1.0, 1.0]])

ser = serial.Serial("COM17", 9600, timeout=0.005)

arena_config = {0: ("Water Pitcher", 6, "2-2"), 2:("Pebble", 8, "3-3"), 4:("Pebble", 16, "2-2"), 6:("Pebble", 19, "1-1")} 

Robot_start = "START-1"
arena_aruco = []
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
    global texture_object, texture_background,stone,pot1,pot2,crow,pot,stone1,stone2
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
    stonehalf = OBJ('stonefull.obj', swapyz=True)
    pot = OBJ('claypotf.obj', swapyz=True)

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
                if pot_visit == 1 :
                    glCallList(pot.gl_list)
                elif pot_visit == 3 :
                    glCallList(pot1.gl_list)
                elif pot_visit == 5 :
                    glCallList(pot2.gl_list)
                elif pot_visit == 7 :
                    glCallList(pot3.gl_list)    
            elif x[0] == 1 :
                if change_ar[x[0]] == -1 :
                    glCallList(stone.gl_list)
                elif change_ar[x[0]] == 1:
                    glCallList(stone2.gl_list)
            elif x[0] == 2 :
                if change_ar[x[0]] == -1 :
                    glCallList(stone.gl_list)
                elif change_ar[x[0]] == 1:
                    glCallList(stone2.gl_list)
            elif x[0] == 3 :
                if change_ar[x[0]] == -1 :
                    glCallList(stone.gl_list)
                elif change_ar[x[0]] == 1 :
                    glCallList(stone2.gl_list)
            elif x[0] == 4 :
                if change_ar[x[0]] == -1 :
                    glCallList(stone.gl_list)
                elif change_ar[x[0]] == 1:
                    glCallList(stone2.gl_list)
            elif x[0] == 5 :
                if change_ar[x[0]] == -1 :
                    glCallList(stone.gl_list)
                elif change_ar[x[0]] == 1 :
                    glCallList(stone2.gl_list)
            elif x[0] == 6 :
                if change_ar[x[0]] == -1 :
                    glCallList(stone.gl_list)
                elif change_ar[x[0]] == 1:
                    glCallList(stone2.gl_list)
            elif x[0] == 7 :
                if change_ar[x[0]] == -1 :
                    glCallList(stone.gl_list)
                elif change_ar[x[0]] == -1 :
                    glCallList(stone2.gl_list)
            elif x[0] == 8 :
                if change_ar[x[0]] == -1 :
                    glCallList(stone.gl_list)
                elif change_ar[x[0]] == 1:
                    glCallList(stone2.gl_list)        
            elif x[0] == 9 :
                if change_ar[x[0]] == -1 :
                    glCallList(stone.gl_list)
                elif change_ar[x[0]] == 1:
                    glCallList(stone2.gl_list)
                    
            elif x[0] == 10 :
                    glCallList(crow.gl_list)
                    
            glPopMatrix()
    
    print(tvec)


    
'''
* Function Name: get_info()
* Input:  None
* Output: None
* Logic: Get the serial data from the bot by calling the function get_serial
*        and if the recieved character is 'a' which is acknowledgement from the bot
         on reaching the node and put_serial is called for sending further operation
         parameters.
* in the function>
* Example Call: get_info()
'''
def get_info():
    serial_char = get_serial()
    if(serial_char == 'a'):
        put_serial()
        
    else:
        return
'''
* Function Name: get_serial()
* Input:  None
* Output: Returns the serial data recieved from the bot
* Logic:  Serial data from the bot is recieved using the serial object and read functions.
* 
* Example Call: serial_char = get_serial()
'''
    
def get_serial():

    global ser
    
    i=ser.readline()
    time.sleep(0.05)
    #print(i)
    return i

'''
* Function Name: put_serial()
* Input:  None
* Output: None
* Logic: Upon recieving an acknowledgement next character in the traverse list is sent
*        to the bot using travel_index. On reaching the end of traverse character
          'b' is sent for buzzer function.

* Example Call: put_serial()
'''


def put_serial():
    global ser,travel_index
    if travel_index is not len(traverse):
        direction = traverse[travel_index]
        
    elif travel_index is len(traverse):
        direction = 'b'

    if direction in change:
        if(pot_visit%2==0):
            change_ar[arena_aruco[pot%2]]=1
        pot_visit = pot_visit +1
        
        
    ser.write(str.encode(direction))
    time.sleep(0.05)
    travel_index = travel_index+1
    
'''
* Function Name: bfs()
* Input:  source node, destination node
* Output: Two lists pred and dist which are used to create a list of the shortest path from the source to the destination
* Logic:  Carries out Breadth-First Search of the given graph to identify the shortest
  path between the source and the destination node using the predefined adjacency list.
* in the function>
* Example Call: get_info()
'''

    
def bfs(source, dest):
    queue.reset()
    front = rear = -1
    for i in range(n):
        visited[i] = 0
        dist[i] = -1
        pred[i] = -1
    
    visited[source-1] = 1
    dist[source-1] = 0
    queue.enqueue(source)
        
    while queue.size() != 0:
        a = queue.dequeue()
        for i in range(3):
            b = adj[a-1][i]
            if b != -1:
                if visited[b-1] == 0:
                    visited[b-1] = 1
                    dist[b-1] = dist[a-1] + 1
                    pred[b-1] = a
                    queue.enqueue(b)
                    if b == dest:
                        return 1
    return 0

'''
* Function Name: shortestPath()
* Input:  source node, destination node, index
* Output: List path containing the sequence of nodes in the shortest path from the source to the destination
* Logic:  Crawls through the pred list to find the node previous to the destination and so on,
          hence creating the list of nodes forming the shortest path
* in the function>
* Example Call: get_info()
'''            
def shortestPath(source, dest, index):
    z = bfs(source, dest)
    if z == 0:
        print ("Given source and destination not connected.")
        return
    
    #print("source,dest ",source,dest)
    crawl = dest
    c = 0
    path[index][c]=dest
    c=c+1
    while pred[crawl-1] != -1:
        crawl = pred[crawl-1]
        #print('    c      : ',c)
        path[index][c] = crawl
        c = c + 1    
    return int(dist[dest-1])

###############################################################################
'''
* Function Name: configuration()
* Input:  node number
* Output: Return either X or Y configuration
* Logic:  We split the hexagon into two configurations. The left half is X configuration
*         and the right half into Y configuration. The node connected to 2 smaller nodes
     is Y and the node connected to 2 larger nodes is X configuration. The edge nodes at
     the lower half are connected to empty node with value -1 at the larger node points so
     they need to configuration swapped. The swap list contains  the list of nodes present on
     the lower half of the arena.
* in the function>
* Example Call: config = configuration(node)
'''

def configuration(node):
        
        config = None
        conf = -1
        if node in swap:
            conf = conf * -1
        for j in range(3):
            
            if adj[node-1][j] < node:
               conf = conf * -1
               
        if (conf == 1):
            config = 'x'
        else:
            config = 'y'
        return config



###############################################################################
'''
* Function Name: direct()
* Input:  mapx (shortest path),pvnode,axis
* Output: None
* Logic: Based on the planned shortest path the directional command like left -> 'l' and right -> 'r' for the bot is planned and appended in the list traverse.

* Example Call: direct(map,pvnode,axis)
'''
    
def direct (mapx , pvnode, axis):
    #print(axis, " axis in direct")
    node =0
    prnode=0
    nxnode=0
    bnode=0
    count = 0
    pvnode = 0
    global axis_new
    conf = -1
    global g,pvnodex
    turn =None
    config = None
    for i in range (g-1):
        prnode= mapx[i]
        
        nxnode = mapx[i+1]
        node = mapx[i]
        config = configuration(node)                             ##### finding conf using configuration function
        
        
        if pvnode == 0 and i == 0:
            #print(prnode,'  prnode', config," config")
            if config == 'x':
                if axis == 1:
                    if (nxnode==prnode+1):
                    
                        turn = 'r'
                    elif (nxnode<prnode):
                        turn = 'l'
                    elif nxnode == pvnodex or nxnode > prnode :
                        turn = 'u'
                elif axis == 2:
                    if (nxnode<prnode):
                    
                        turn = 'r'
                    elif nxnode == prnode+1:
                        turn = 'u'
                    elif nxnode>prnode:
                        turn = 'l'
                elif axis == 3:
                    if (nxnode==prnode+1):
                    
                        turn = 'l'
                    elif nxnode > prnode:
                        turn = 'r'
                    elif nxnode == pvnodex or nxnode < prnode:
                        turn = 'u'
            if config == 'y':
                if axis == 1:
                    if (nxnode+1==prnode):
                        turn = 'r'
                    elif nxnode > prnode:
                        turn = 'l'
                    elif nxnode ==pvnodex or nxnode < prnode:
                        turn = 'u'
                elif axis == 2:
                    
                    if (nxnode>prnode):
                        
                        turn = 'r'
                    elif nxnode < prnode and (nxnode!=prnode-1):
                        turn = 'l'
                    elif nxnode == pvnodex or (nxnode<prnode):
                        turn = 'u'
                    
                elif axis == 3:
                    if (nxnode+1==prnode):
                    
                        turn = 'l'
                    elif nxnode<prnode:
                        turn = 'r'
                    elif nxnode == pvnodex or nxnode > prnode:
                        turn = 'u'
             

        
        else:
                
            if i>0 :
                pvnode = mapx[i-1]
            bnode = 0
            if node in swap:
                
                bnode = bnode + 1
            
            if ((config == 'x') and (pvnode>0)):
                if pvnode>prnode or ((bnode!=0)and(pvnode>prnode)) :
                    if pvnode!= (prnode+1):
                        if nxnode == (prnode+1):
                            turn = 'r'
                            
                        else:
                            turn = 'l'
                    else :
                        if nxnode > prnode and (bnode==0):
                            turn = 'l'
                        else:
                            turn = 'r'
                            
                else :
                    if nxnode == (prnode+1):
                        turn = 'l'
                    else:
                        turn = 'r'
                        
            elif config == 'y'and pvnode>0:
                
                if pvnode < prnode:
                    if prnode == (pvnode+1):
                        #print ("prev node"+ pvnode +"present node"+ prnode)
                        if prnode>nxnode:
                            turn = 'l'
                        else :
                            turn = 'r'
                            
                    else:
                        if prnode<nxnode:
                            turn = 'l'
                        else:
                            turn = 'r'
                else:
                    if nxnode == (prnode-1):
                        turn  = 'l'
                    else:
                        turn = 'r'

    



                       
        traverse.append (turn)
    
        traverse.append (node_action(nxnode,prnode,axis_new))

    
    
    print(str(drive))
    return prnode

'''
* Function Name: node_action()
* Input:  previous node, present node, axis to be operated
* Output: One out (s/e/i)
* Logic:  The bot reached the node and now it has to turn according to particular axis as given in the configuration.
          The turn are calculated using the previous and present node.
*         S -> go straight without any turn, i -> r'i'ght turn., e -> l'e'ft turn.
* Example Call: get_info()
'''
        
def node_action(prnode,pvnode,axis):              #####   s =staright,  e = 120 degree left turn , i = 120 degree right turn

    go = []
    config = configuration(prnode)
    
    if axis == 3:                       ###### all the axis is reference to our aerena design and not to the flex
        if config == 'x':
            if pvnode<prnode:
                go = 's'
            elif pvnode == prnode+1:
                go = 'e'
            elif pvnode > prnode:
                go = 'i'
        elif config == 'y':
            
            if pvnode > prnode:
                go = 's'
            elif prnode == pvnode+1:
                go = 'e'
            elif pvnode < prnode:
                go = 'i'
    if axis == 2:
        if config == 'y':
            if prnode == pvnode+1:
                go = 's'
            elif pvnode > prnode:
                go = 'i'
            elif pvnode < prnode:
                go = 'e'
        elif config == 'x':
            if pvnode == prnode+1:
                go = 's'
            elif pvnode > prnode:
                go = 'e'
            elif pvnode < prnode:
                go = 'i'
    if axis == 1:
        if config == 'x':
            if pvnode < prnode:
                go = 'e'
            elif pvnode == prnode+1:
                go = 'i'
            elif pvnode > prnode:
                go = 's'
        elif config == 'y':
            if pvnode+1==prnode:
                go = 'i'
            elif pvnode > prnode:
                go = 'e'
            elif pvnode < prnode:
                go = 's'
    
    return go
'''
* Function Name: best_route()
* Input:  cell, orient
* Output: Path is planned for a particular cell and axis using shortest_path function
          and the distance is stored in the variables d1 and d2.
          The path is list contains the path for both nodes of axis a cell so on comparing the distance best route is calculated
* Example Call: d = best_route(cell,orient,s,m)
'''

def best_route(cell,orient,s,m):

    
    for i in range(len(arena)+1):                     # m - Iterator from main function ; s - start node from main function
        
            
            for j in range(4):
                
                if i == cell[m]:
                    
                    if j == orient[m]:
                        
                        if j==1:
                            
                            d1=int(arena[i-1][0])
                            d2=int(arena[i-1][1])
                        if j==2:
                            
                            d1=int(arena[i-1][2])
                            d2=int(arena[i-1][3])
                        if j==3:
                            
                            d1=int(arena[i-1][4])
                            d2=int(arena[i-1][5])
                        print(d1)               
                   
    l1 = shortestPath(s, d1,0)
       
       
    l2 = shortestPath(s, d2,1)
                   
       
    if l1<l2:
            index = 0
            c = l1
           
    else :
            index = 1
            c = l2
            
    return index
'''
* Function Name: arena_conf()
* Input:  None
* Output: None
* Logic:  The configuration of the task was obtained from the arena_config dictionary and stored in the lists pitcher ,cell-list, orient_list  
*         for calculation of path
* Example Call: arena_conf()
'''

def arena_conf():


    j=0
    
    for k,v in arena_config.items():
        
   
        if arena_config[k][j] == "Water Pitcher":
            pitcher.append(arena_config[k][j+1])
            pitcher.append(int(arena_config[k][j+2][0]))
        elif arena_config[k][j] == "Pebble":
            cell_list.append(arena_config[k][j+1])
            orient_list.append(int(arena_config[k][j+2][0]))
            arena_aruco.append(k)


            

def main():

    map_ = []
    m = 0
    global pvnode,g,n,c,pvnodex
    axis = 2
    global axis_new
    cell = []
    
    orient = []
    
    

    if "1" in Robot_start:
        start = 1
    else:
        start = 2

    arena_conf()

    print(cell_list)
    print(orient_list)
    print(arena_aruco)


    for i in range(3):
        cell.append(cell_list[i])
        cell.append(pitcher[0])
        orient.append(orient_list[i])
        orient.append(pitcher[1])

    if start is 1:
        s = 25
    elif start is 2:
        s = 30                       #### s is new start variable
    for p in range(len(orient)):
        if orient[p] is 2:
            orient[p] = 3
        elif orient[p] is 3:
            orient[p] = 2
    
    while((m<len(orient))):
        d1 = 0
        d2 = 0   
        axis_new = orient[m]
        index = best_route(cell,orient,s,m)
        
        g=0
        
        for i in range(c, -1, -1):
                g = g+1
                map_.append(int(path[index][i]))
                
        print(map_)
        
        x = direct(map_,pvnode,axis)
        pvnodex = x
        axis = axis_new
        pvnode = 0 
        s = int(map_[g-1])

        for i in range(2):
            for j in range(n):
                path[i][j] = -1
                map_.clear()
       
        
        m = m+1
    print ("\n\t\t directions of bot \n")

    print(str(traverse))
   
    
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
    
    
    glutMainLoop()
    
if __name__ == '__main__':
    main()
    
