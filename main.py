import pygame
import sys
import numpy as np
from enum import IntEnum
import random

class Bead:

    def __init__(self, pos, vel, radius, color):
        self.pos = pos
        self.old_pos = np.copy(pos)
        self.vel = vel
        self.color = color
        self.radius = radius

        # hom density assumption
        self.mass = self.radius**2

class DistanceConstraint:
    
    def __init__(self, bead, position, radius):
        self.bead = bead
        self.position = position
        self.radius = radius

    def partiallySolve(self, alpha):
        
        n_fixed_to_beat = self.bead.pos - self.position
        n_fixed_to_beat = n_fixed_to_beat / np.linalg.norm(n_fixed_to_beat)

        pos_proj = (n_fixed_to_beat * self.radius) + self.position

        pos_error = pos_proj - self.bead.pos

        self.bead.pos += alpha * pos_error


class CollisionConstraint:

    def __init__(self, beadA, beadB):
        self.beadA = beadA
        self.beadB = beadB

    def partiallySolve(self, alpha):
        
        aToB = self.beadB.pos - self.beadA.pos

        distAtoB = np.linalg.norm(aToB)

        # no collision resolution needed
        if distAtoB > self.beadA.radius + self.beadB.radius:
            return

        nAtoB = aToB / distAtoB

        deltaDist = (self.beadA.radius + self.beadB.radius) - distAtoB

        # collision
        M = self.beadA.mass + self.beadB.mass

        distAMoves = deltaDist * (self.beadB.mass / M)
        distBMoves = deltaDist * (self.beadA.mass / M)

        self.beadA.pos += alpha * (distAMoves * (-nAtoB))
        self.beadB.pos += alpha * (distBMoves * nAtoB)
        

# Set up the display
screen_width, screen_height = 800, 800
max_bead_radius = 50
min_bead_radius = 25
wire_radius = (screen_width - 2 * max_bead_radius)// 2
wire_pos = np.array([screen_width//2, screen_height // 2])

initial_angle = 40

# span on the wire
bead_cols = [(255, 0, 0), (255, 153, 153), (153, 255, 153), (153, 204, 255), (255, 255, 153)]
beads_pos = []


delta_angle = 2 * initial_angle / len(bead_cols)

for i, col in enumerate(bead_cols):

    angle = -initial_angle + i * delta_angle

    # start at the top

    angle += 90

    rad = angle * (2 * np.pi / 360.0)

    # to (x, y) position
    x = np.cos(rad) * wire_radius + wire_pos[0]
    y = -np.sin(rad) * wire_radius + wire_pos[1]

    beads_pos.append(np.array([x, y]))

beads = []

for i in range(len(beads_pos)):
    beads.append(Bead(pos=beads_pos[i], vel=np.array([0, 0]), radius =  random.randint(min_bead_radius, max_bead_radius), color=bead_cols[i]))

g = np.array([0, 1000])

constraints = []

for bead in beads:
    constraints.append(DistanceConstraint(bead=bead, position=wire_pos, radius=wire_radius))

# pairwise collision constraints
for i, bead_i in enumerate(beads):
    for j, bead_j in enumerate(beads[i + 1:]):

        constraints.append(CollisionConstraint(beadA=bead_i, beadB = bead_j))

n_iterations_constraints = 10

# "learning rate for the constraint solver"
alpha = 1 / n_iterations_constraints

hasStarted = False

def main():
    # Initialize Pygame
    pygame.init()

    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("Bead on wire")

    # Create a clock to limit the frame rate
    clock = pygame.time.Clock()

    # Main game loop
    running = True
    while running:

        # Limit the frame rate to 60 FPS
        dt = clock.tick(60) / 1000

        # Event handling
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    global hasStarted
                    hasStarted = True

        # update physics (semi implicit euler, later leapfrog)
        if hasStarted:

            # only do physics update if has started
            for i in range(len(beads)):
                beads[i].vel = beads[i].vel + dt * g
                beads[i].old_pos = np.copy(beads[i].pos)
                beads[i].pos = beads[i].pos + dt * beads[i].vel

            # solve for constraints
            for _ in range(n_iterations_constraints):

                for constraint in constraints:
                    constraint.partiallySolve(alpha=alpha)
                    
            
            # constraints are approx. solved 
            for i in range(len(beads)):
                beads[i].vel = (beads[i].pos - beads[i].old_pos) / dt

        # Fill the screen with a background color (RGB)
        screen.fill((0, 0, 0))

        # draw wire
        pygame.draw.circle(screen, color=(255, 255, 255), center=wire_pos,
                            radius=wire_radius, width=3)
        
        # draw bead
        for i in range(len(beads)):
            pygame.draw.circle(screen, color=beads[i].color, center=beads[i].pos, radius=beads[i].radius)

        # Update the display
        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
