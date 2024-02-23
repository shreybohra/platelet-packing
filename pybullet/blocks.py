#%%

import pybullet as p
import time
import pybullet_data
import numpy as np
import math
import random

#%%

class Distribution:
    def __init__(self, mean, std):
        self.mean = mean
        self.std = std
        
    def generate(self):
        return random.gauss(self.mean, self.std)

width_mean = 100
width_std = 10
ar_mean = 5
ar_std = 2
depth = 30
density = 1

width_gen = Distribution(width_mean, width_std)
ar_gen = Distribution(ar_mean, ar_std)


#%%

def create_platelet():
    width = width_gen.generate()
    height = width*ar_gen.generate()

    mass = width*height*depth*density

    visualShapeId = -1
    collisionShapeId = p.createCollisionShape(p.GEOM_BOX,halfExtents=[width/2, height/2, depth/2])
    body = p.createMultiBody(mass, collisionShapeId, visualShapeId, [0,0,0])
    return body

#%%

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.81)
planeId = p.loadURDF("plane.urdf")

# create an empty container
container_size = [2000, 1000, 500]/2

# create the container
container_id = p.createCollisionShape(p.GEOM_BOX,halfExtents=container_size)
container_visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=container_size, rgbaColor=[0.8, 0.8, 0.8, 1])
container_body_id = p.createMultiBody(0, container_id, container_visual_id, [0, 0, 0])
