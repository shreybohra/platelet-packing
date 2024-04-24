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

#%%
# create a sphere object
print("Initialising sphere generator")
def sphere_generator():
    return 0.1 # return desired radius of sphere

random_sphere = platelets.Sphere(radius_generator=sphere_generator)
random_sphere.set_container_size(container_size)
random_sphere.set_density(1)
random_sphere.set_friction(0.5)
random_sphere.set_restitution(0.2)

#%%
# create a cube object
print("Initialising cube generator")
def cuboid_generator():
    x = random.uniform(0.1, 0.2)
    y = random.uniform(0.3, 0.6)
    z = random.uniform(0.01, 0.05)
    return [x, y, z] # return desired half extents of cuboid


cuboid = platelets.Cuboid()
cuboid.set_container_size(container_size)
cuboid.set_density(3)
cuboid.set_friction(0.2)
cuboid.set_restitution(0.8)


    