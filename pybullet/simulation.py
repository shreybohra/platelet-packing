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
        # zoom out
        p.resetDebugVisualizerCamera(cameraDistance=30, cameraYaw=0, cameraPitch=-60, cameraTargetPosition=[0, 0, 0])

        self.fps = fps
        self.dt = 1/fps
        p.setTimeStep(self.dt)

    def set_density(self, density):
        self.density = density

    def set_friction(self, friction):
        self.friction = friction
    
    def set_restitution(self, restitution):
        self.restitution = restitution

    def set_container_size(self, container_size):
        self.container_size = container_size
    
