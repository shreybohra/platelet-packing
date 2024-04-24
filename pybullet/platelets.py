import pybullet as p
import pybullet_data
import time
import numpy as np
import math
import random


#%%

class Platelet:

    def __init__(self, container_size = [1, 1, 1], density = 1, friction = 0.5, restitution = 0.2):
        self.container_size = container_size
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
    
    def default_generator(self):
        return 1
    
    def generate_random_position(self):
        container_size = self.container_size
        x = random.uniform(-container_size[0]+2, container_size[0]-2)
        y = random.uniform(-container_size[1]+2, container_size[1]-2)
        z = container_size[2] + 20
        orientation = p.getQuaternionFromEuler([random.uniform(0, 2 * math.pi) for _ in range(3)])
        return [x, y, z], orientation
    
    def check_collision(self, body_id, existing_bodies):
        for body in existing_bodies:
            contact_points = p.getClosestPoints(body_id, body, distance=0.5)
            if contact_points:
                return True
        return False


class Sphere(Platelet):

    def __init__(self, density = 1, friction = 0.5, restitution = 0.2, radius_generator = None, position_generator = None):
        super().__init__(density, friction, restitution)

        if radius_generator is None:
            self.radius_generator = self.default_generator()
        else:
            self.radius_generator = radius_generator

        if position_generator is None:
            self.position_generator = self.generate_random_position()
        else:
            self.position_generator = position_generator

    def get_volume(self):
        return (4/3) * math.pi * self.radius**3

    def get_mass(self):
        return self.density * self.get_volume()

    def get_inertia(self):
        return (2/5) * self.get_mass() * self.radius**2
    
    def __initialise_sphere(self):
        
        self.radius = self.radius_generator()
        sphere_id = p.createCollisionShape(p.GEOM_SPHERE, radius=self.radius)
        sphere_visual_id = p.createVisualShape(p.GEOM_SPHERE, radius=self.radius, rgbaColor=[random.uniform(0.1, 1) for _ in range(3)] + [1])


        return sphere_id, sphere_visual_id
    
    def create_sphere(self, position, orientation):
            
            sphere_id, sphere_visual_id = self.__initialise_sphere()
            position, orientation = self.position_generator()
            mass = self.get_mass()
            inertia = self.get_inertia()
            body_id = p.createMultiBody(mass, sphere_id, sphere_visual_id, position, orientation, lateralFriction=self.friction, restitution=self.restitution, localInertiaDiagonal=[inertia, inertia, inertia])

            return body_id
    
    