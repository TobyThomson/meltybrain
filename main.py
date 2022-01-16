import sys
import pygame
from pygame.locals import *
import math

# Setup Variables
FPS = 30

ArenaColour = pygame.Color(255, 255, 255)
RobotColour = pygame.Color(65, 82, 110)
NorthVectorColour = pygame.Color(255, 0, 0)
PointingVectorColour = pygame.Color(0, 255, 0)

PixelsPermm = 15

WindowSize = (800, 800)

# Robot Variables
RobotDiamater_mm = 250
RobotWheelSpacing_mm = 180
RobotWheelDiameter_mm = 56

# Setup
pygame.init()

Clock = pygame.time.Clock()

DisplaySurface = pygame.display.set_mode(WindowSize)
pygame.display.set_caption("Meltybrain Simulator")

class RobotSprite(pygame.sprite.Sprite):
    def __init__(self, windowSize, diameter, wheelSpacing_mm, wheelDiameter_mm):
        super().__init__()

        self.diameter = math.ceil(diameter / PixelsPermm)
        self.wheelSpacing_mm = math.ceil(wheelSpacing_mm / PixelsPermm)
        self.wheelDiameter_mm = math.ceil(wheelDiameter_mm / PixelsPermm)

        self.position = pygame.math.Vector2(windowSize[0] / 2, windowSize[1] / 2)
        self.northVector = pygame.math.Vector2(0, -50)
        self.angle = 5
    
    def update(self):
        self.angle = (self.angle + 5) % 360
        self.pointingVector = self.northVector.rotate(self.angle)
    
    def draw(self, surface):
        # Body
        pygame.draw.circle(surface, RobotColour, self.position, self.diameter)

        # Vectors
        pygame.draw.line(surface, NorthVectorColour, self.position, (self.northVector + self.position), 3)
        pygame.draw.line(surface, PointingVectorColour, self.position, (self.pointingVector + self.position), 3)

# Main
RobotSprite = RobotSprite(WindowSize, RobotDiamater_mm, RobotWheelSpacing_mm, RobotWheelDiameter_mm)

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