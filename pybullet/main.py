#%%
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

container_size = [5, 5, 1]
container_vol = np.prod(container_size)

sim.create_container(container_size)
sim.set_container_dynamics(friction=0.5, restitution=0.2)

sim.set_zoom(10)


#%%
# create a sphere object
print("Initialising sphere generator")
def sphere_generator():
    return 0.1 # return desired radius of sphere

random_sphere = platelets.Sphere(container_size, radius_generator=sphere_generator)
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


cuboid = platelets.Cuboid(container_size, dims_generator=cuboid_generator)
cuboid.set_container_size(container_size)
cuboid.set_density(3)
cuboid.set_friction(0.2)
cuboid.set_restitution(0.8)


 #%%
# start the simulation
print ("Starting simulation...")

generated_vol = 0
# initialise list of generated bodies 
bodies = []
body_count = 0

while generated_vol < container_vol*1.5: #allow for some extra
    
    # decide cube or sphere
    weight = random.uniform(0, 1)
    if weight > 0.7:
        #print("Generating cuboid")
        body = cuboid
    else:
        #print("Generating sphere")
        body = random_sphere

    new_body = body.create(enforce_collision=True)
    bodies.append(new_body)
    body_count += 1
    generated_vol += body.volume

    print(f"\r Generated {body_count} bodies, total volume: {generated_vol:.2f}", end="", flush=True)
    sim.step(count=20)

   