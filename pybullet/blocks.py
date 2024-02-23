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
# initialize the physics engine
p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.81)
planeId = p.loadURDF("plane.urdf")
start_pos = [0, 0, 0]
start_orientation = p.getQuaternionFromEuler([0, 0, 0])

fps = 200
p.setTimeStep(1/fps)

# start_pos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)

# create an empty container - this is half dims
container_size = [1000, 500, 250]

# create the container
container_id = p.createCollisionShape(p.GEOM_BOX,halfExtents=container_size)
container_visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=container_size, rgbaColor=[0.8, 0.8, 0.8, 1])
container_body_id = p.createMultiBody(0, container_id, container_visual_id, [0, 0, 0])

#%%
# initialise platelets
def generate_random_position(container_size=container_size):
    x = random.uniform(-container_size[0], container_size[0])
    y = random.uniform(-container_size[1], container_size[1])
    z = container_size[2] + 200
    orientation = p.getQuaternionFromEuler([random.uniform(0, 2 * 3.1416) for _ in range(3)])
    return [x, y, z], orientation

def create_platelet():
    width = width_gen.generate()
    height = width * ar_gen.generate()
    mass = width * height * depth * density
    position, orientation = generate_random_position()
    platelet_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[width, height, depth])
    platelet_visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[width, height, depth], rgbaColor=[random.uniform(0, 1) for _ in range(3)] + [1])
    platelet_body_id = p.createMultiBody(mass, platelet_id, platelet_visual_id, position, orientation)
    return platelet_body_id

def check_init_collision(platelet_id, existing_platelets):
    for platelet in existing_platelets:
        contact_points = p.getClosestPoints(platelet_id, platelet)
        if contact_points:
            return True # collision detected
    return False

existing_platelets = []

for _ in range(10):
    pos, orn = generate_random_position()
    
    platelet_id = create_platelet()

    while check_init_collision(platelet_id, existing_platelets):
        pos, orn = generate_random_position()
        p.resetBasePositionAndOrientation(platelet_id, pos, orn)

    for _ in range(100):
        p.stepSimulation()
        
    existing_platelets.append(platelet_id)

for _ in range(1000):
    p.stepSimulation()
    


#%%
# exit
p.disconnect()