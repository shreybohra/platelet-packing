#%%
import pybullet as p
import time
import pybullet_data
import numpy as np
import math
import random

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
def generate_random_position(container_size=container_size):
    x = random.uniform(-container_size[0], container_size[0])
    y = random.uniform(-container_size[1], container_size[1])
    z = container_size[2] + 20
    orientation = p.getQuaternionFromEuler([random.uniform(0, 2 * 3.1416) for _ in range(3)])
    return [x, y, z], orientation

def create_sphere(radius, position, orientation):

    density = 1
    mass = density * (4/3) * math.pi * radius**3    
    sphere_id = p.createCollisionShape(p.GEOM_SPHERE, radius=radius)
    sphere_visual_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=[random.uniform(0, 1) for _ in range(3)] + [1])
    sphere_body_id = p.createMultiBody(mass, sphere_id, sphere_visual_id, position, orientation)

    return sphere_body_id

def check_init_collision(sphere_id, existing_spheres):
    for sphere in existing_spheres:
        contact_points = p.getClosestPoints(sphere_id, sphere, distance=0.1)
        if contact_points:
            return True # collision detected
    return False

def check_movement(existing_spheres, threshold=0.1):
    
    linear_velocities = [np.linalg.norm(p.getBaseVelocity(sphere)[0]) for sphere in existing_spheres]

    if any(abs(v) > threshold for v in linear_velocities):
        return True
    
    return False

