import pybullet as p
import pybullet_data
import time
import numpy as np
import math
import random

#%%

class BulletSim:
    def __init__(self, container_size = [1, 1, 1], density = 1, friction = 0.5, restitution = 0.2, gui = True):
        self.container_size = container_size
        self.density = density
        self.friction = friction
        self.restitution = restitution

        if gui:
            self.client = p.connect(p.GUI)
        else:
            self.client = p.connect(p.DIRECT)

    def set_density(self, density):
        self.density = density

    def set_friction(self, friction):
        self.friction = friction
    
    def set_restitution(self, restitution):
        self.restitution = restitution

    def set_container_size(self, container_size):
        self.container_size = container_size
    
