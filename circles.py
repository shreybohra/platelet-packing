#%%
# Importing the libraries
import pygame
import pymunk
import pymunk.pygame_util
import random
import math

# initial parameters
dt = 0.005
box_width = 600
box_height = 200
width_mean = 10
width_std = 5
ar_mean = 5
ar_std = 2

screen_width = box_width + 200
screen_height = box_height*2

wall_elasticity = 0.4
wall_friction = 0.5

class Distribution:
    def __init__(self, mean, std):
        self.mean = mean
        self.std = std
        
    def generate(self):
        return random.gauss(self.mean, self.std)

width_gen = Distribution(width_mean, width_std)
ar_gen = Distribution(ar_mean, ar_std)


pygame.init()
screen = pygame.display.set_mode((screen_width, screen_height))
pymunk.pygame_util.positive_y_is_up = True
draw_options = pymunk.pygame_util.DrawOptions(screen)

font = pygame.font.Font(None, 36)

space = pymunk.Space()
space.gravity = (0, -981)


#%%

floor = pymunk.Segment(space.static_body, ((screen_width-box_width)/2, 5), (box_width+(screen_width-box_width)/2, 5), 5)
left_wall = pymunk.Segment(space.static_body, ((screen_width-box_width)/2 + 5, 5), ((screen_width-box_width)/2 + 5, box_height), 5)
right_wall = pymunk.Segment(space.static_body, (box_width + (screen_width-box_width)/2 - 5, 5), (box_width + (screen_width-box_width)/2 - 5, box_height), 5)

floor.elasticity = wall_elasticity
floor.friction = wall_friction

left_wall.elasticity = wall_elasticity
left_wall.friction = wall_friction

right_wall.elasticity = wall_elasticity
right_wall.friction = wall_friction

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
        self.elasticity = 0.4
        self.friction = 0.5     
 
    def create(self):
        # platelet parameters
        # bodge to make sure always positive and non-zero
        width = abs(self.width_gen.generate()) + 1
        height = abs(width * self.ar_gen.generate()) + 1
        self.area = width * height
        mass = self.area * self.density

        # initial position and angle
        self.y = box_height + 100
        self.x = random.uniform(0, self.box_width - width) + 100  # Ensure it fits within the box
        angle = random.uniform(-0.5*math.pi, 0.5*math.pi)

        # create the platelet
        self.body = pymunk.Body(mass, pymunk.moment_for_box(mass, (width, height)))
        self.body.position = self.x, self.y
        self.body.angle = angle
        self.shape = pymunk.Poly.create_box(self.body, (width, height))
        self.shape.elasticity = self.elasticity
        self.shape.friction = self.friction
        space.add(self.body, self.shape)
        

    def draw(self):
        pygame.draw.polygon(screen, (0, 0, 0), self.shape.get_vertices())

    def get_area(self):
        return self.area

# game loop
        
running = True
clock = pygame.time.Clock()

platelets = Rectangle(box_width, box_height, width_gen, ar_gen)
total_area = 0

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            
    screen.fill((255, 255, 255))

    if random.random() < 0.02:
        platelets.create()
        total_area += platelets.get_area()

    space.debug_draw(draw_options)
    
    pygame.display.flip()
    space.step(dt)
    clock.tick(1/dt) # keep realtime

pygame.quit()
        
