#%%
# Importing the libraries
import pygame
import pymunk
import pymunk.pygame_util
import random
import math
import collections
import time

# initial parameters
dt = 0.005
box_width = 1500
box_height = 400
width_mean = 10
width_std = 5
ar_mean = 5
ar_std = 2
radius = 8

wall_width = 5
screen_width = box_width + wall_width
screen_height = box_height*2

wall_elasticity = 0
wall_friction = 100

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
# IMPORTANT note: gravity is in pixels per second squared
# if dt is too high, the platelets will fall through the floor and physics breaks
space.gravity = (0, -981)
# space.gravity = (0, -450)


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
        self.elasticity = 0
        self.friction = 100
        

    def create(self):
        # platelet parameters
        # bodge to make sure always positive and non-zero
        radius = abs(self.radius_gen.generate()) + 1
        self.area = math.pi * radius**2
        mass = self.area * self.density

        # initial position and angle
        self.y = box_height + radius*2 + 10
        self.x = random.uniform(radius + 2*wall_width, self.box_width - radius - 2*wall_width)  # Ensure it fits within the box

        # create the platelet
        self.body = pymunk.Body(mass, pymunk.moment_for_circle(mass, 0, radius))
        self.body.position = self.x, self.y
        self.body.type = pymunk.Body.DYNAMIC
        self.shape = pymunk.Circle(self.body, radius)
        self.shape.elasticity = self.elasticity
        self.shape.friction = self.friction
        space.add(self.body, self.shape)
        
        # self.body.gravity_scale = 0.2
        
    def draw(self):
        pygame.draw.circle(screen, (0, 0, 0), (int(self.body.position.x), int(self.body.position.y)), int(self.shape.radius))

    def get_area(self):
        return self.area

sticky = True
sticky_bodies = {left_wall.body, right_wall.body, floor.body}


def platelet_collision_handler(arbiter, space, data):
    # Get the two colliding shapes
    shape_a, shape_b = arbiter.shapes
    
    
    if sticky:
        if shape_a.body in sticky_bodies and shape_b.body not in sticky_bodies:
            pos = shape_b.body.position
            rot = shape_b.body.angle
            shp = shape_b
            radius = shp.radius

            space.remove(shape_b.body, shape_b)

            static_body = pymunk.Body(body_type=pymunk.Body.STATIC)
            static_body.position = pos
            static_body.angle = rot

            static_shape = pymunk.Circle(static_body, radius)
            static_shape.color = (255, 0, 0, 0)

            static_shape.elasticity = 0
            static_shape.friction = 1e9

            space.add(static_body, static_shape)
            sticky_bodies.add(static_body)

            return False
        
        elif shape_b.body in sticky_bodies and shape_a.body not in sticky_bodies:
            pos = shape_a.body.position
            rot = shape_a.body.angle
            shp = shape_a
            radius = shp.radius

            space.remove(shape_a.body, shape_a)

            static_body = pymunk.Body(body_type=pymunk.Body.STATIC)
            static_body.position = pos
            static_body.angle = rot

            static_shape = pymunk.Circle(static_body, radius)
            static_shape.color = (255, 0, 0, 0)

            static_shape.elasticity = 0
            static_shape.friction = 1e9

            space.add(static_body, static_shape)
            sticky_bodies.add(static_body)
                        
            return False
        
    return True

def stop_body(space, key, body, radius):
    body.velocity = (0, 0)
    body.angular_velocity = 0
    body.rotation = 0
    body.position = (body.position.x, body.position.y + radius)
    

if sticky:
    space.add_collision_handler(0, 0).pre_solve = platelet_collision_handler

class InactiveBodies:
    def __init__(self, max_inactive_bodies=50, velocity_threshold=0.1):
        self.bodies = collections.deque(maxlen=max_inactive_bodies)
        self.velocity_threshold = velocity_threshold

    def check_stopped(self, body):
        if abs(body.velocity.y) < self.velocity_threshold:
            return True
        return False

    def check_settled(self, body): # check if the body is touching another body - indicates its settled
        for shape in body.shapes:
            if shape.body != body and pymunk.ShapeFilter().should_collide(shape.filter, body.filter):
                return True
        return False            
            
    
    def add(self, body):
        self.bodies.append(body)
    
    def update(self, space):
        for body in space.bodies:
            if body not in self.bodies:
                if self.check_stopped(body):
                    # if self.check_settled(body):
                    self.add(body)



    def highest(self):
        highest = 1
        for body in self.bodies:
            if body.position.y > highest:
                highest = body.position.y
        return highest
    
    def number(self):
        return len(self.bodies)
        

# game loop
        
running = True
clock = pygame.time.Clock()

radius_gen = Distribution(radius, 0)
platelets = Circle(box_width, box_height, radius_gen)
inactives = InactiveBodies()

total_area = 0
elapsed_time = 0
generation_interval = math.ceil(0.2/dt)
gen_frame = 1

physics = True

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.KEYDOWN and event.key == pygame.K_SPACE:
            physics = not physics
            
    screen.fill((255, 255, 255))

    if physics:

        if gen_frame % generation_interval == 0:
            platelets.create()
            total_area += platelets.get_area()
            gen_frame = 1
        else:
            gen_frame += 1


        

    
        space.step(dt)
        clock.tick(1/dt) # keep realtime

        elapsed_time += dt

    space.debug_draw(draw_options)

        
        
    
    time_display = font.render(f"Elapsed time: {elapsed_time:.2f}", True, (0, 0, 0))
    screen.blit(time_display, (100, 10))

    area_display = font.render(f"Total area: {total_area:.2f}", True, (0, 0, 0))
    screen.blit(area_display, (100, 50))

    pygame.display.flip()

    if not sticky:
        inactives.update(space)
        highest = inactives.highest()

        if highest > box_height:
            running = False

    else:
        highest = 0
        for body in space.bodies:
            if body.position.y > highest:
                if body.body_type == pymunk.Body.STATIC:
                    highest = body.position.y
                    
        if highest > (box_height+radius*2):
            print(f"static body at {body.position.x:.2f}, {body.position.y:.2f}")
            # time.sleep(5)
            running = False

    # print(f"current highest: {highest}")
    # print(f"inactives: {inactives.number()}")

pygame.quit()
        
print(f"Total area: {total_area:.2f}")
print(f"Elapsed time: {elapsed_time:.2f}")
print(f"Packing density: {total_area/(box_width*box_height):.2f}")
