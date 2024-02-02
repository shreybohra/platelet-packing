#%%
# Importing the libraries
import pygame
import pymunk
import pymunk.pygame_util
import random
import math

# initial parameters
dt = 0.01
box_width = 600
box_height = 200
width_mean = 10
width_std = 5
ar_mean = 5
ar_std = 2

class Distribution:
    def __init__(self, mean, std):
        self.mean = mean
        self.std = std
        
    def generate(self):
        return random.gauss(self.mean, self.std)

width_gen = Distribution(width_mean, width_std)
ar_gen = Distribution(ar_mean, ar_std)


pygame.init()
screen = pygame.display.set_mode((box_width + 200, box_height*2))
draw_options = pymunk.pygame_util.DrawOptions(screen)

space = pymunk.Space()
space.gravity = (0, -981)


#%%

floor = pymunk.Segment(space.static_body, (0, 5), (box_width, 5), 5)
left_wall = pymunk.Segment(space.static_body, (0, 5), (0, box_height), 5)
right_wall = pymunk.Segment(space.static_body, (box_width, 5), (box_width, box_height), 5)

floor.elasticity = 0.95
floor.friction = 0.5

left_wall.elasticity = 0.95
left_wall.friction = 0.5

right_wall.elasticity = 0.95
right_wall.friction = 0.5

space.add(floor, left_wall, right_wall)

#%%

# create a class to generate rectangles
class Rectangle:
    def __init__(self, box_width, box_height, width_gen, ar_gen):
        self.box_width = box_width
        self.box_height = box_height
        self.width_gen = width_gen
        self.ar_gen = ar_gen
        self.density = 1       
        
        

    def create(self):
        # platelet parameters
        width = self.width_gen.generate()
        height = width * self.ar_gen.generate()
        mass = width * height * self.density

        # initial position and angle
        self.y = box_height + 100
        self.x = random.uniform(0, self.box_width - width)  # Ensure it fits within the box
        angle = random.uniform(-0.5*math.pi, 0.5*math.pi)

        # create the platelet
        self.body = pymunk.Body(mass, pymunk.moment_for_box(mass, width, height))
        self.body.position = self.x, self.y
        self.body.angle = angle
        self.shape = pymunk.Poly.create_box(self.body, (width, height))
        self.shape.elasticity = 0.95
        self.shape.friction = 0.9
        space.add(self.body, self.shape)

    def draw(self):
        pygame.draw.polygon(screen, (0, 0, 0), self.shape.get_vertices())

# game loop
        
running = True
clock = pygame.time.Clock()

platelets = Rectangle(box_width, box_height, width_gen, ar_gen)

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            
    screen.fill((255, 255, 255))

    if random.random() < 0.01:
        platelets.create()

    for body in space.bodies:
        for shape in body.shapes:
            pygame.draw.polygon(screen, (0, 0, 0), shape.get_vertices())
    
    pygame.display.flip()
    space.step(dt)
    clock.tick(1/dt) # keep realtime

pygame.quit()
        
