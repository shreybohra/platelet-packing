import pybullet as p
import pybullet_data
import time
import numpy as np
import math
import random

#%%

class BulletSim:
    def __init__(self, gui = True, fps = 200):

        if gui:
            self.client = p.connect(p.GUI)
        else:
            self.client = p.connect(p.DIRECT)

        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.81)
        self.planeId = p.loadURDF("plane.urdf")
        self.start_pos = [0, 0, 0]
        self.start_orientation = p.getQuaternionFromEuler([0, 0, 0])
        
        self.cameraDistance = 30
        self.cameraYaw = 0
        self.cameraPitch = -60
        self.cameraTargetPosition = [0, 0, 0]

        self.__set_camera()

        self.fps = fps
        self.dt = 1/fps
        p.setTimeStep(self.dt)


    def __set_camera(self):
        p.resetDebugVisualizerCamera(cameraDistance=self.cameraDistance, 
                                     cameraYaw=self.cameraYaw, 
                                     cameraPitch=self.cameraPitch, 
                                     cameraTargetPosition=self.cameraTargetPosition)

    def set_zoom(self, distance):
        self.cameraDistance = distance
        self.__set_camera()

    def set_yaw(self, yaw):
        self.cameraYaw = yaw
        self.__set_camera()
    
    def set_pitch(self, pitch):
        self.cameraPitch = pitch
        self.__set_camera()

    def set_target(self, target):
        self.cameraTargetPosition = target
        self.__set_camera()

    def zoom_in(self):
        self.cameraDistance = self.cameraDistance * 0.9
        self.__set_camera()

    def zoom_out(self):
        self.cameraDistance = self.cameraDistance * 1.1
        self.__set_camera()
        
