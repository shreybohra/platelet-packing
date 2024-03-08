#%%
# Importing the libraries
import pygame
import pymunk
import pymunk.pygame_util
import random
import math

# initial parameters
dt = 0.05
box_width = 600
box_height = 200
width_mean = 10
width_std = 5
ar_mean = 5
ar_std = 2

wall_width = 5
screen_width = box_width + wall_width
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

floor = pymunk.Segment(space.static_body, ((screen_width-box_width)/2, wall_width), (box_width+(screen_width-box_width)/2, wall_width), wall_width)
left_wall = pymunk.Segment(space.static_body, ((screen_width-box_width)/2 + wall_width, wall_width), ((screen_width-box_width)/2 + wall_width, box_height), wall_width)
right_wall = pymunk.Segment(space.static_body, (box_width + (screen_width-box_width)/2 - wall_width, wall_width), (box_width + (screen_width-box_width)/2 - wall_width, box_height), wall_width)

floor.elasticity = wall_elasticity
floor.friction = wall_friction

left_wall.elasticity = wall_elasticity
left_wall.friction = wall_friction

right_wall.elasticity = wall_elasticity
right_wall.friction = wall_friction

space.add(floor, left_wall, right_wall)

#%%
    
# create a class to generate circles
class Circle:
    def __init__(self, box_width, box_height, radius_gen):
        self.box_width = box_width
        self.box_height = box_height
        self.radius_gen = radius_gen
        self.density = 1
        self.elasticity = 0.4
        self.friction = 0.5

    def create(self):
        # platelet parameters
        # bodge to make sure always positive and non-zero
        radius = abs(self.radius_gen.generate()) + 1
        self.area = math.pi * radius**2
        mass = self.area * self.density

        # initial position and angle
        self.y = box_height + 100
        self.x = random.uniform(radius + 5, self.box_width - radius - 5)  # Ensure it fits within the box

        # create the platelet
        self.body = pymunk.Body(mass, pymunk.moment_for_circle(mass, 0, radius))
        self.body.position = self.x, self.y
        self.body.type = pymunk.Body.DYNAMIC
        self.shape = pymunk.Circle(self.body, radius)
        self.shape.elasticity = self.elasticity
        self.shape.friction = self.friction
        space.add(self.body, self.shape)
        
    def draw(self):
        pygame.draw.circle(screen, (0, 0, 0), (int(self.body.position.x), int(self.body.position.y)), int(self.shape.radius))

    def get_area(self):
        return self.area

sticky = True
sticky_bodies = {left_wall.body, right_wall.body, floor.body}

def platelet_collision_handler(arbiter, space, data):
    # Get the two colliding shapes
    shape_a, shape_b = arbiter.shapes

    # print("Collision Handler Called")
    # print("Body A Type:", shape_a.body.body_type)
    # print("Body B Type:", shape_b.body.body_type)

    if sticky:
        if shape_a.body in sticky_bodies and shape_b.body not in sticky_bodies:
            # shape_b.body.velocity = (0, 0)
            # shape_b.body.position = (shape_b.body.position.x, shape_b.body.position.y + 50)
            space.add_post_step_callback(set_body_static, None, shape_b.body)
            sticky_bodies.add(shape_b.body)
            return False
        elif shape_b.body in sticky_bodies and shape_a.body not in sticky_bodies:
            # shape_a.body.velocity = (0, 0)
            # shape_a.body.position = (shape_a.body.position.x, shape_a.body.position.y + 50)
            space.add_post_step_callback(set_body_static, None, shape_a.body)
            sticky_bodies.add(shape_a.body)
            return False
        
    return True

def set_body_static(space, key, body):
    body.velocity = (0, 0)
    body.body_type = pymunk.Body.STATIC


space.add_collision_handler(0, 0).begin = platelet_collision_handler


# game loop
        
running = True
clock = pygame.time.Clock()

radius_gen = Distribution(10, 0)
platelets = Circle(box_width, box_height, radius_gen)

total_area = 0
elapsed_time = 0
generation_interval = 1//dt
gen_frame = 1

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            
    screen.fill((255, 255, 255))

    if gen_frame % generation_interval == 0:
        platelets.create()
        total_area += platelets.get_area()
        gen_frame = 1
    else:
        gen_frame += 1

    space.debug_draw(draw_options)

    elapsed_time += dt
    time_display = font.render(f"Elapsed time: {elapsed_time:.2f}", True, (0, 0, 0))
    screen.blit(time_display, (100, 10))

    area_display = font.render(f"Total area: {total_area:.2f}", True, (0, 0, 0))
    screen.blit(area_display, (100, 50))
    
    pygame.display.flip()
    space.step(dt)
    clock.tick(1/dt) # keep realtime

pygame.quit()
        
