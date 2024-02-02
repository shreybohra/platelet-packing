#%%
# Importing the libraries
import pygame
import pymunk
import pymunk.pygame_util
import random

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

space.add(floor, left_wall, right_wall)

#%%

# create a class to generate rectangles
class Rectangle:
    def __init__(self, box_width, box_height):
        self.box_width = box_width
        self.box_height = box_height
        
        

    def create(self, x, y, width, height, mass):
        self.body = pymunk.Body(mass, pymunk.moment_for_box(mass, width, height))
        self.body.position = x, y
        self.shape = pymunk.Poly.create_box(self.body, (width, height))
        self.shape.elasticity = 0.95
        self.shape.friction = 0.9
        space.add(self.body, self.shape)

    def draw(self):
        pygame.draw.polygon(screen, (0, 0, 0), self.shape.get_vertices())

# create a class for normal distribution of dimensions
        

        
