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

width_mean = 1
width_std = 0.1
ar_mean = 5
ar_std = 2
depth = 0.3
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
# zoom out
p.resetDebugVisualizerCamera(cameraDistance=50, cameraYaw=0, cameraPitch=-45, cameraTargetPosition=[0, 0, 0])

fps = 200
dt = 1/fps
p.setTimeStep(dt)

# start_pos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)

# create an empty container - this is half dims
container_size = [20, 10, 0.5]

# create the container
container_id = p.createCollisionShape(p.GEOM_BOX,halfExtents=container_size)
container_visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=container_size, rgbaColor=[0.8, 0.8, 0.8, 1])
container_body_id = p.createMultiBody(0, container_id, container_visual_id, [0, 0, 0])

# Create collision shapes for the container walls
wall_thickness = 0.1
wall_height = 5  
half_extents_x = (container_size[0] + wall_thickness)
half_extents_y = (container_size[1] + wall_thickness)

# Left wall
left_wall_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_thickness, half_extents_y, wall_height])
left_wall_visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[wall_thickness, half_extents_y, wall_height], rgbaColor=[0.8, 0.8, 0.8, 1])
# Right wall
right_wall_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_thickness, half_extents_y, wall_height])
right_wall_visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[wall_thickness, half_extents_y, wall_height], rgbaColor=[0.8, 0.8, 0.8, 1])
# Top wall
top_wall_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half_extents_x, wall_thickness, wall_height])
top_wall_visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[half_extents_x, wall_thickness, wall_height], rgbaColor=[0.8, 0.8, 0.8, 1])
# Bottom wall
bottom_wall_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half_extents_x, wall_thickness, wall_height])
bottom_wall_visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[half_extents_x, wall_thickness, wall_height], rgbaColor=[0.8, 0.8, 0.8, 1])
# Attach collision shapes to the container body
p.createMultiBody(0, left_wall_id, basePosition=[-container_size[0] - wall_thickness, 0, wall_height], baseOrientation=[0, 0, 0, 1])
p.createMultiBody(0, right_wall_id, basePosition=[container_size[0] + wall_thickness, 0, wall_height], baseOrientation=[0, 0, 0, 1])
p.createMultiBody(0, top_wall_id, basePosition=[0, container_size[1] + wall_thickness, wall_height], baseOrientation=[0, 0, 0, 1])
p.createMultiBody(0, bottom_wall_id, basePosition=[0, -container_size[1] - wall_thickness, wall_height], baseOrientation=[0, 0, 0, 1])
#%%
# initialise platelets
def generate_random_position(container_size=container_size):
    x = random.uniform(-container_size[0], container_size[0])
    y = random.uniform(-container_size[1], container_size[1])
    z = container_size[2] + 20
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
        contact_points = p.getClosestPoints(platelet_id, platelet, distance=0.1)
        if contact_points:
            return True # collision detected
    return False

def check_movement(existing_platelets, threshold=0.1):
    
    linear_velocities = [np.linalg.norm(p.getBaseVelocity(platelet)[0]) for platelet in existing_platelets]

    if any(abs(v) > threshold for v in linear_velocities):
        return True
    
    return False

existing_platelets = []

def shake(container_body_id, amplitude=0.8, frequency=8, duration=10):
    
    start_time = time.time()
    end_time = start_time + duration

    while time.time() < end_time:
        t = time.time() - start_time
        dz = [0, 0, amplitude * math.sin(2 * math.pi * frequency * t)] # z axis
        orn = [0, 0, 0, 1]
        p.resetBasePositionAndOrientation(container_body_id, dz, orn)

        p.stepSimulation()
        time.sleep(dt)

print("Creating platelets...")
for _ in range(50):
    pos, orn = generate_random_position()
    
    platelet_id = create_platelet()

    while check_init_collision(platelet_id, existing_platelets):
        pos, orn = generate_random_position()
        p.resetBasePositionAndOrientation(platelet_id, pos, orn)

    for _ in range(10):
        p.stepSimulation()
        time.sleep(dt)
        
    existing_platelets.append(platelet_id)

print("Settling...")
while check_movement(existing_platelets):
    for _ in range(10):
        p.stepSimulation()
        time.sleep(dt)
print("Shaking...")
shake(container_body_id)
    
simend = input("Continue?")

#%%
# exit
# p.disconnect()