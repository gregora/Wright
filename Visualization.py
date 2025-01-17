import sys, pygame
from pygame.locals import *
from pygame.constants import *
from OpenGL.GL import *
from OpenGL.GLU import *

from OBJFileLoader.objloader import *

import numpy as np

class Visualization:

    obj = None
    camera_pos = np.array([0, 0, 0])
    rotate = False

    def __init__(self, viewport = (800, 800)):
        pygame.init()
        viewport = (800,600)
        hx = viewport[0]/2
        hy = viewport[1]/2
        srf = pygame.display.set_mode(viewport, OPENGL | DOUBLEBUF)

        glLightfv(GL_LIGHT0, GL_POSITION,  (-40, 200, 100, 0.0))
        glLightfv(GL_LIGHT0, GL_AMBIENT, (0.2, 0.2, 0.2, 1.0))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (0.5, 0.5, 0.5, 1.0))
        glEnable(GL_LIGHT0)
        glEnable(GL_LIGHTING)
        glEnable(GL_COLOR_MATERIAL)
        glEnable(GL_DEPTH_TEST)
        glShadeModel(GL_SMOOTH)

        # LOAD OBJECT AFTER PYGAME INIT
        self.obj = OBJ("CAD/plane.obj", swapyz=True)
        self.obj.generate()
        
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        width, height = viewport
        gluPerspective(90.0, width/float(height), 1, 100.0)
        glEnable(GL_DEPTH_TEST)
        glMatrixMode(GL_MODELVIEW)

    def update(self, t, euler):          
        
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
  
        glTranslate(0.0, 0.0, -5)

        glRotate(-50, 1, 0, 0)


        glRotate(euler[0], 1, 0, 0)
        glRotate(euler[1], 0, 1, 0)
        glRotate(euler[2], 0, 0, 1)

        glRotate(self.camera_pos[0], 0, 0, 1)
        glRotate(self.camera_pos[1], 1, 0, 0)

        self.obj.render()

        pygame.display.flip()

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
            elif event.type == pygame.MOUSEBUTTONDOWN:
                self.rotate = True

            elif event.type == pygame.MOUSEBUTTONUP:
                self.rotate = False
            
            elif event.type == pygame.MOUSEMOTION:
                i, j = event.rel
                if self.rotate:
                    self.camera_pos[0] += i
                    self.camera_pos[1] += j

    def close(self):
        pygame.quit()