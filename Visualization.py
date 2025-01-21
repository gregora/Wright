import sys, pygame
from pygame.locals import *
from pygame.constants import *
from OpenGL.GL import *
from OpenGL.GLU import *

from OBJFileLoader.objloader import *

import numpy as np

import cv2

class Visualization:

    obj = None
    ground = None
    camera_pos = np.array([0, 0, 0])
    rotate = False
    record = False

    def __init__(self, viewport = (800, 800), record = False, fps = 60, filename = "output.mp4"):
        pygame.init()
        self.viewport = viewport
        hx = viewport[0]/2
        hy = viewport[1]/2
        srf = pygame.display.set_mode(viewport, OPENGL | DOUBLEBUF)
        
        pygame.display.set_caption("Wright")

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
        
        self.ground = OBJ("CAD/ground.obj", swapyz=True)
        self.ground.generate()

        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        width, height = viewport
        gluPerspective(90.0, width/float(height), 1, 3000.0)
        glEnable(GL_DEPTH_TEST)
        glMatrixMode(GL_MODELVIEW)

        self.record = record

        if self.record:
            self.fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.out = cv2.VideoWriter(filename, self.fourcc, fps, (int(viewport[0]),int(viewport[1])))
    
    def update(self, t, x, euler):        
        
        glClearColor(0.2, 0.4, 0.4, 1)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glLoadIdentity()
  

        glTranslate(0.0, 0.0, -5)

        glRotate(-70, 1, 0, 0)

        glRotate(self.camera_pos[0], 0, 0, 1)
        glRotate(self.camera_pos[1], 1, 0, 0)

        glTranslate(-x[0, 0], x[1, 0], x[2, 0])
        self.ground.render()
        glTranslate(x[0, 0], -x[1, 0], -x[2, 0])


        glRotate(euler[0], 1, 0, 0)
        glRotate(-euler[1], 0, 1, 0)
        glRotate(-euler[2], 0, 0, 1)

        self.obj.render()




        if self.record:
            data = glReadPixels(0, 0, self.viewport[0], self.viewport[1], GL_RGB, GL_UNSIGNED_BYTE)
            image = pygame.image.fromstring(data, self.viewport, 'RGB')
            image = np.transpose(pygame.surfarray.array3d(image), (1, 0, 2))
            image = cv2.flip(image, 0)
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            self.out.write(image)

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