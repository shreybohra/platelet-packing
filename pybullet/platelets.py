import pybullet as p
import pybullet_data
import time
import numpy as np
import math
import random


#%%

class Platelet:

    def __init__(self, density = 1, friction = 0.5, restitution = 0.2):
        self.density = density
        self.friction = friction
        self.restitution = restitution

    def set_density(self, density):
        self.density = density

    def set_friction(self, friction):
        self.friction = friction
    
    def set_restitution(self, restitution):
        self.restitution = restitution
    
    def default_generator(self):
        return 1


class Sphere(Platelet):

    def __init__(self, density = 1, friction = 0.5, restitution = 0.2, radius_generator = None):
        super().__init__(density, friction, restitution)
        
        if radius_generator is None:
            self.radius_generator = self.default_generator()
        else:
            self.radius_generator = radius_generator

    def set_radius(self, radius):
        self.radius = radius

    def get_volume(self):
        return (4/3) * math.pi * self.radius**3

    def get_mass(self):
        return self.density * self.get_volume()

    def get_inertia(self):
        return (2/5) * self.get_mass() * self.radius**2


            