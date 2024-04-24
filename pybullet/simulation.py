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

        if gui:        
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

    def step(self):
        p.stepSimulation()

    def create_container(self, dims, wall_thickness = 0.1, wall_color = [0.8, 0.8, 0.8, 1]):

        base_dims = [dims[0]/2, dims[1]/2, 0.5] # half dims

        # create the container
        container_id = p.createCollisionShape(p.GEOM_BOX,halfExtents=base_dims)
        container_visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=base_dims, rgbaColor=[0.8, 0.8, 0.8, 1])
        container_body_id = p.createMultiBody(0, container_id, container_visual_id, [0, 0, 0])

        # Create collision shapes for the container walls
        wall_height = dims[2]  
        half_extents_x = (base_dims + wall_thickness)
        half_extents_y = (base_dims + wall_thickness)

        # Left wall
        left_wall_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_thickness, half_extents_y, wall_height])
        left_wall_visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[wall_thickness, half_extents_y, wall_height], rgbaColor=wall_color)
        # Right wall
        right_wall_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_thickness, half_extents_y, wall_height])
        right_wall_visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[wall_thickness, half_extents_y, wall_height], rgbaColor=wall_color)
        # Top wall
        top_wall_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half_extents_x, wall_thickness, wall_height])
        top_wall_visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[half_extents_x, wall_thickness, wall_height], rgbaColor=wall_color)
        # Bottom wall
        bottom_wall_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[half_extents_x, wall_thickness, wall_height])
        bottom_wall_visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=[half_extents_x, wall_thickness, wall_height], rgbaColor=wall_color)

        # Attach collision shapes to the container body
        p.createMultiBody(0, left_wall_id, basePosition=[-base_dims[0] - wall_thickness, 0, wall_height], baseOrientation=[0, 0, 0, 1])
        p.createMultiBody(0, right_wall_id, basePosition=[base_dims[0] + wall_thickness, 0, wall_height], baseOrientation=[0, 0, 0, 1])
        p.createMultiBody(0, top_wall_id, basePosition=[0, base_dims[1] + wall_thickness, wall_height], baseOrientation=[0, 0, 0, 1])
        p.createMultiBody(0, bottom_wall_id, basePosition=[0, -base_dims[1] - wall_thickness, wall_height], baseOrientation=[0, 0, 0, 1])

        self.container_bodies = [container_body_id, left_wall_id, right_wall_id, top_wall_id, bottom_wall_id]

    def set_container_dynamics(self, friction = 0.5, restitution = 0.2):
        if isinstance(friction, (int, float)):
            friction = [friction] * 5
        if isinstance(restitution, (int, float)):
            restitution = [restitution] * 5

        if isinstance(friction, list) and len(friction) != 5:
            raise ValueError("friction should be a scalar or a list of length 5")
        if isinstance(restitution, list) and len(restitution) != 5:
            raise ValueError("restitution should be a scalar or a list of length 5")
        
        p.changeDynamics(self.container_bodies, -1, lateralFriction=friction, restitution=restitution)