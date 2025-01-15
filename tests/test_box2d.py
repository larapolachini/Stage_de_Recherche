#!/usr/bin/env python3

import pygame
import Box2D
from Box2D.b2 import world, circleShape, edgeShape, staticBody, dynamicBody, distanceJoint
import random
import math

# Constants
WIDTH, HEIGHT = 600, 600  # Arena dimensions
NUM_ROBOTS = 150           # Number of robots
ROBOT_RADIUS = 6          # Radius of each robot (Box2D units)
MEMBRANE_RADIUS = 150     # Radius of the membrane (in pixels)
NUM_MEMBRANE_DOTS = 100    # Number of dots in the membrane
FPS = 100                  # Frames per second
TIME_STEP = 1.0 / FPS     # Simulation time step
PPM = 20                  # Pixels per meter (for rendering Box2D world in pygame)

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
ROBOT_COLOR = (0, 255, 0)
MEMBRANE_COLOR = (255, 0, 0)

# Initialize pygame
pygame.init()
screen = pygame.display.set_mode((600, 600))
pygame.display.set_caption("Swarm Robotics Simulator with Box2D")
clock = pygame.time.Clock()

# Box2D world setup
world = world(gravity=(0, 0), doSleep=True)


# Walls (static bodies)
walls = [
    world.CreateStaticBody(
        shapes=edgeShape(vertices=[(0, 0), (WIDTH / PPM, 0)])
    ),
    world.CreateStaticBody(
        shapes=edgeShape(vertices=[(WIDTH / PPM, 0), (WIDTH / PPM, HEIGHT / PPM)])
    ),
    world.CreateStaticBody(
        shapes=edgeShape(vertices=[(WIDTH / PPM, HEIGHT / PPM), (0, HEIGHT / PPM)])
    ),
    world.CreateStaticBody(
        shapes=edgeShape(vertices=[(0, HEIGHT / PPM), (0, 0)])
    ),
]

## Robot class
#class Robot:
#    def __init__(self, x, y):
#        # Create a dynamic body for the robot
#        self.body = world.CreateDynamicBody(position=(x / PPM, y / PPM))
#        self.shape = circleShape(radius=ROBOT_RADIUS / PPM)
#        fixture = self.body.CreateCircleFixture(shape=self.shape, density=1, friction=0.3, restitution=0.9)
#        self.body.linearVelocity = (random.uniform(-20, 20), random.uniform(-20, 20))
#
#    def draw(self, screen):
#        # Get position in pixels
#        pos = self.body.position
#        pygame.draw.circle(
#            screen,
#            ROBOT_COLOR,
#            (int(pos.x * PPM), int(pos.y * PPM)),
#            int(ROBOT_RADIUS),
#        )

class Robot:
    def __init__(self, x, y):
        # Create a dynamic body for the robot
        self.body = world.CreateDynamicBody(position=(x / PPM, y / PPM))
        self.shape = circleShape(radius=ROBOT_RADIUS / PPM)
        self.fixture = self.body.CreateCircleFixture(
            shape=self.shape, density=1, friction=0.3, restitution=0.9
        )
        self.change_timer = 0  # Timer to track when to change velocity
        self.randomize_velocity()

    def randomize_velocity(self):
        # Set a random linear velocity
        self.vel = (
            random.uniform(-20, 20),  # Random velocity in x-direction
            random.uniform(-20, 20),  # Random velocity in y-direction
        )
        self.body.linearVelocity = self.vel

    def update(self, delta_time):
        # Update the timer and change velocity if time exceeds threshold
        self.change_timer += delta_time
        self.body.linearVelocity = self.vel
        if self.change_timer >= 0.5:  # Change direction every 2 seconds
            self.randomize_velocity()
            self.change_timer = 0

    def draw(self, screen):
        # Get position in pixels
        pos = self.body.position
        pygame.draw.circle(
            screen,
            ROBOT_COLOR,
            (int(pos.x * PPM), int(pos.y * PPM)),
            int(ROBOT_RADIUS),
        )

class Membrane:
    def __init__(self, center_x, center_y, radius, num_dots):
        self.dots = []
        self.joints = []
        angle_step = 2 * math.pi / num_dots

        # Create membrane dots
        for i in range(num_dots):
            angle = i * angle_step
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            dot = world.CreateDynamicBody(position=(x / PPM, y / PPM))
            dot.CreateCircleFixture(radius=ROBOT_RADIUS / PPM, density=5, friction=0.3)  # Increased density
            self.dots.append(dot)

        # Connect dots with distance joints (adjacent connections)
        for i in range(num_dots):
            dot_a = self.dots[i]
            dot_b = self.dots[(i + 1) % num_dots]  # Connect in a loop
            joint = world.CreateDistanceJoint(
                bodyA=dot_a,
                bodyB=dot_b,
                anchorA=dot_a.worldCenter,
                anchorB=dot_b.worldCenter,
                length=(radius * 2 * math.pi / num_dots) / PPM,
                frequencyHz=30.0,  # Increased stiffness
                dampingRatio=0.1,  # Reduced damping
            )
            self.joints.append(joint)

        # Add limited cross-joints for more rigidity
        cross_joint_radius = 3  # Number of neighbors to skip for cross-joints
        for i in range(num_dots):
            for j in range(1, cross_joint_radius + 1):
                dot_a = self.dots[i]
                dot_b = self.dots[(i + j) % num_dots]  # Connect to nearby neighbors
                joint = world.CreateDistanceJoint(
                    bodyA=dot_a,
                    bodyB=dot_b,
                    anchorA=dot_a.worldCenter,
                    anchorB=dot_b.worldCenter,
                    length=((dot_a.worldCenter - dot_b.worldCenter).length),
                    frequencyHz=20.0,  # Slightly lower stiffness for cross-joints
                    dampingRatio=0.1,
                )
                self.joints.append(joint)

    def draw(self, screen):
        # Draw dots
        for dot in self.dots:
            pos = dot.position
            pygame.draw.circle(
                screen,
                MEMBRANE_COLOR,
                (int(pos.x * PPM), int(pos.y * PPM)),
                int(ROBOT_RADIUS),
            )

        # Draw joints (springs)
        for joint in self.joints:
            pos_a = joint.bodyA.position
            pos_b = joint.bodyB.position
            pygame.draw.line(
                screen,
                MEMBRANE_COLOR,
                (int(pos_a.x * PPM), int(pos_a.y * PPM)),
                (int(pos_b.x * PPM), int(pos_b.y * PPM)),
                1,
            )

# Helper function to generate random points inside the membrane
def generate_point_inside_membrane(center_x, center_y, radius):
    angle = random.uniform(0, 2 * math.pi)  # Random angle
    r = random.uniform(0, radius)  # Random radius within the membrane
    x = center_x + r * math.cos(angle)
    y = center_y + r * math.sin(angle)
    return x, y

# Create robots
#robots = [
##    Robot(random.randint(WIDTH//5 + ROBOT_RADIUS * PPM, WIDTH - WIDTH//5 - ROBOT_RADIUS * PPM),
##          random.randint(HEIGHT//5 + ROBOT_RADIUS * PPM, HEIGHT - HEIGHT//5 - ROBOT_RADIUS * PPM))
#    Robot(random.randint(ROBOT_RADIUS * PPM, WIDTH - ROBOT_RADIUS * PPM),
#          random.randint(ROBOT_RADIUS * PPM, HEIGHT - ROBOT_RADIUS * PPM))
#    for _ in range(NUM_ROBOTS)
#]
robots = [
    Robot(
        *generate_point_inside_membrane(WIDTH / 2, HEIGHT / 2, MEMBRANE_RADIUS - ROBOT_RADIUS)
    )
    for _ in range(NUM_ROBOTS)
]

# Create membrane
membrane = Membrane(WIDTH / 2, HEIGHT / 2, MEMBRANE_RADIUS, NUM_MEMBRANE_DOTS)

# Main loop
running = True
while running:
    screen.fill(BLACK)  # Clear the screen
    delta_time = clock.tick(FPS) / 1000.0  # Time elapsed in seconds

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Step the Box2D simulation
    world.Step(TIME_STEP, 10, 10)

    # Draw robots
    for robot in robots:
        robot.update(delta_time)  # Update robot velocity based on timer
        robot.draw(screen)
    membrane.draw(screen)

    pygame.display.flip()  # Update the display
    clock.tick(FPS)

# Quit pygame
pygame.quit()

