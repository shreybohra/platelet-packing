import pybullet as p
import pybullet_data
import time
import numpy as np
import math
import random

# custom classes for simulation abstraction
import simulation
import platelets

#%%
# create the simulation object
print ("Initialising simulation...")
sim = simulation.BulletSim(gui=True, fps=200)

container_size = [10, 5, 5]
container_vol = np.prod(container_size)

sim.create_container(container_size)
sim.set_container_dynamics(friction=0.5, restitution=0.2)

# initialise list of generated bodies 
bodies = []



