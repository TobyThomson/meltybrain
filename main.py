import sys
import pygame
from pygame.locals import *

# Setup Variables
FPS = 30

ArenaColour = pygame.Color(255, 255, 255)
RobotColour = pygame.Color(65, 82, 110)
NorthVectorColour = pygame.Color(255, 0, 0)
PointingVectorColour = pygame.Color(0, 255, 0)

WindowSize = (800, 800)

# Setup
pygame.init()

Clock = pygame.time.Clock()

DisplaySurface = pygame.display.set_mode(WindowSize)
pygame.display.set_caption("Meltybrain Simulator")

# Convert coordinates into pygame coordinates (lower-left => top left)
def Convert_Cordinates(coordinates):
    height = WindowSize[1]
    return (coordinates[0], height - coordinates[1])

# TODO: Draw wheels and vectors
class RobotSprite(pygame.sprite.Sprite):
    def __init__(self, windowSize):
        super().__init__()

        x_position = windowSize[0] / 2
        y_position = windowSize[1] / 2

        self.position = (x_position, y_position)
    
    def draw(self, surface):
        # Body
        pygame.draw.circle(surface, RobotColour, Convert_Cordinates(self.position), 30)

        # Vectors
        NorthVector = (self.position[0], self.position[1] + 50)
        PointingVector = (self.position[0] + 30, self.position[1] + 30)

        pygame.draw.line(surface, NorthVectorColour, Convert_Cordinates(self.position), Convert_Cordinates(NorthVector), 3)
        pygame.draw.line(surface, PointingVectorColour, Convert_Cordinates(self.position), Convert_Cordinates(PointingVector), 3)

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