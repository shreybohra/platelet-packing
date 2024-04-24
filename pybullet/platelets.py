import pybullet as p
import pybullet_data
import time
import numpy as np
import math
import random


#%%

class Platelet:

    def __init__(self, container_size, density = 1, friction = 0.5, restitution = 0.2):
        self.container_size = [dim/2 for dim in container_size]
        self.density = density
        self.friction = friction
        self.restitution = restitution

    def set_density(self, density):
        self.density = density

    def set_friction(self, friction):
        self.friction = friction
    
    def set_restitution(self, restitution):
        self.restitution = restitution

    def set_container_size(self, container_size):
        self.container_size = container_size
        print("Container size set to: ", container_size)
    
    def default_generator(self):
        return 1
    
    def default_3d_generator(self):
        return [0.1, 0.1, 0.1] # note: half extents (real dims are 2x)
    
    def generate_random_position(self):
        container_size = self.container_size
        x = random.uniform(-container_size[0]+2, container_size[0]-2)
        y = random.uniform(-container_size[1]+2, container_size[1]-2)
        z = container_size[2] + 20
        orientation = p.getQuaternionFromEuler([random.uniform(0, 2 * math.pi) for _ in range(3)])
        print (f"Generated position: {x:.2f}, {y:.2f}, {z:.2f}")
        print (f"Generated orientation: {orientation}")
        return [x, y, z], orientation
    
    def check_collision(self, body_id, existing_bodies):
        for body in existing_bodies:
            contact_points = p.getClosestPoints(body_id, body, distance=0.5)
            if contact_points:
                return True
        return False


class Sphere(Platelet):

    def __init__(self, density = 1, friction = 0.5, restitution = 0.2, 
                 radius_generator = None, position_generator = None):
        
        super().__init__(density, friction, restitution)

        if radius_generator is None:
            self.radius_generator = self.default_generator
        else:
            self.radius_generator = radius_generator

        if position_generator is None:
            self.position_generator = self.generate_random_position
        else:
            self.position_generator = position_generator

    def __get_sphere_properties(self, sphere_id):
        pos, _ = p.getBasePositionAndOrientation(sphere_id)
        radius = p.getVisualShapeData(sphere_id)[0][3][0]
        return pos, radius

    def calculate_volume(self, sphere_id):
        _, radius = self.__get_sphere_properties(sphere_id)
        return (4/3) * math.pi * radius**3

    def calculate_mass(self, sphere_id):
        return self.density * self.calculate_volume(self, sphere_id)

    def __initialise_body(self):
        
        self.radius = self.radius_generator()
        sphere_id = p.createCollisionShape(p.GEOM_SPHERE, radius=self.radius, 
                                        lateralFriction=self.friction, restitution=self.restitution)
        sphere_visual_id = p.createVisualShape(p.GEOM_SPHERE, radius=self.radius, 
                                               rgbaColor=[random.uniform(0.1, 1) for _ in range(3)] + [1])
        print("Sphere ID:", sphere_id)
        print("Sphere Visual ID:", sphere_visual_id)

        return sphere_id, sphere_visual_id
    
    def create(self, enforce_collision = False, existing_bodies = []):
            
            sphere_id, sphere_visual_id = self.__initialise_body()
            position, orientation = self.position_generator()
            self.volume = (4/3) * math.pi * self.radius**3
            self.mass = self.density * self.volume

            body_id = p.createMultiBody(self.mass, sphere_id, sphere_visual_id, 
                                        position, orientation)

            if enforce_collision:
                while self.check_collision(body_id, existing_bodies):
                    position, orientation = self.position_generator()
                    p.resetBasePositionAndOrientation(body_id, position, orientation)
            
            return body_id
    

class Cuboid(Platelet):
    
        def __init__(self, density = 1, friction = 0.5, restitution = 0.2, 
                     dims_generator = None, position_generator = None):
            
            super().__init__(density, friction, restitution)
    
            if dims_generator is None:
                self.dims_generator = self.default_3d_generator
            else:
                self.dims_generator = dims_generator
    
            if position_generator is None:
                self.position_generator = self.generate_random_position
            else:
                self.position_generator = position_generator
    
        def __get_cuboid_properties(self, cuboid_id):
            pos, _ = p.getBasePositionAndOrientation(cuboid_id)
            halfExtents = p.getVisualShapeData(cuboid_id)[0]
            return pos, halfExtents
    
        def calculate_volume(self, cuboid_id):
            _, halfExtents = self.__get_cuboid_properties(cuboid_id)
            return halfExtents[0] * halfExtents[1] * halfExtents[2] * 8
    
        def calculate_mass(self, cuboid_id):
            return self.density * self.calculate_volume(self, cuboid_id)
    
        def __initialise_body(self):
            
            self.halfExtents = self.dims_generator()
            cuboid_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=self.halfExtents, 
                                        lateralFriction=self.friction, restitution=self.restitution)
            cuboid_visual_id = p.createVisualShape(p.GEOM_BOX, halfExtents=self.halfExtents, 
                                                   rgbaColor=[random.uniform(0, 1) for _ in range(3)] + [1])
    
            return cuboid_id, cuboid_visual_id
        
        def create(self, enforce_collision = False, existing_bodies = []):
                
                cuboid_id, cuboid_visual_id = self.__initialise_body()
                position, orientation = self.position_generator()
                
                self.volume = np.prod(self.halfExtents) * 8
                self.mass = self.density * self.volume

                body_id = p.createMultiBody(self.mass, cuboid_id, cuboid_visual_id, 
                                            position, orientation)
    
                if enforce_collision:
                    while self.check_collision(body_id, existing_bodies):
                        position, orientation = self.position_generator()
                        p.resetBasePositionAndOrientation(body_id, position, orientation)
                
                return body_id
        