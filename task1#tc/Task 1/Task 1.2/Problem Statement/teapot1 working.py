
from OpenGL.GL import *
from OpenGL.GLUT import *

def init():
   glClearColor(0, 0, 0, 0)
   glMatrixMode(GL_PROJECTION)
   glLoadIdentity()
   glLightfv(GL_LIGHT0, GL_AMBIENT,
   [0.0, 1.0, 1.0, 100.0])
   glLightfv(GL_LIGHT0, GL_DIFFUSE,
   [1.0, 1.0, 1.0, 100.0])
   glLightfv(GL_LIGHT0, GL_POSITION,
   [0.0, 0.0, 0.0, 50.0])
   glLightModelfv(GL_LIGHT_MODEL_AMBIENT,
   [0.2, 0.2, 0.2, 100.0])
   glEnable(GL_LIGHTING)
   glEnable(GL_LIGHT0)
   #glEnable(GL_DEPTH_TEST)

def display():
   glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
   glMaterialfv(GL_FRONT, GL_AMBIENT,
   [1.1605, 0.0, 1.1, 0.0])
   glMaterialfv(GL_FRONT, GL_DIFFUSE,
   [1.1, 0.0    , 0.6, 0.0])
   glMaterialfv(GL_FRONT, GL_SPECULAR,
   [1.7, 0.6, 0.8, 0.0])
   glMaterialf(GL_FRONT, GL_SHININESS, 10)
   glutSolidTeapot(0.5)
   glutSwapBuffers()

glutInit('')
glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB)
glutInitWindowSize(550, 550)
glutCreateWindow('Hello GLUT')
#glutSetDisplayFuncCallback(display)
#glutDisplayFunc()
glutDisplayFunc(display)
init()
glutMainLoop()
