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

width_gen = Distribution(width_mean, width_std)
ar_gen = Distribution(ar_mean, ar_std)
depth = 5

#%%
