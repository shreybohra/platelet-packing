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
