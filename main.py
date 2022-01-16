import sys
import pygame
from pygame.locals import *

# Setup Variables
FPS = 30

ArenaColour = pygame.Color(255, 255, 255)
RobotColour = pygame.Color(65, 82, 110)

WindowSize = (800, 800)

# Setup
pygame.init()

Clock = pygame.time.Clock()

DisplaySurface = pygame.display.set_mode(WindowSize)
pygame.display.set_caption("Meltybrain Simulator")

# TODO: Draw wheels and vectors
class RobotSprite(pygame.sprite.Sprite):
    def __init__(self, windowSize):
        super().__init__()

        x_position = windowSize[0] / 2
        y_position = windowSize[1] / 2

        self.position = (x_position, y_position)
    
    def draw(self, surface):
        pygame.draw.circle(surface, RobotColour, self.position, 30)

# Main
RobotSprite = RobotSprite(WindowSize)

while True:     
    for event in pygame.event.get():              
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
    
    RobotSprite.update()

    DisplaySurface.fill(ArenaColour)
    RobotSprite.draw(DisplaySurface)
    pygame.display.update()
    Clock.tick(FPS)