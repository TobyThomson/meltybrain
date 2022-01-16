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

LiPoCellVoltage_V = 3.7

# Robot Variables
RobotDiamater_mm = 250
RobotWheelSpacing_mm = 180
RobotWheelDiameter_mm = 56
RobotMotorReduction = 3
RobotMotorKv = 100
RobotBatteryCells = 4

# Setup
pygame.init()

Clock = pygame.time.Clock()

DisplaySurface = pygame.display.set_mode(WindowSize)
pygame.display.set_caption("Meltybrain Simulator")

class RobotSprite(pygame.sprite.Sprite):
    def __init__(self, windowSize, diameter, wheelSpacing_mm, wheelDiameter_mm, motorReduction, motorKv, batteryCells):
        super().__init__()

        self.diameter = diameter
        self.wheelSpacing_mm = wheelSpacing_mm
        self.wheelDiameter_mm = wheelDiameter_mm

        self.motorReduction = motorReduction
        self.motorKv = motorKv
        self.batteryCells = batteryCells

        self.position = pygame.math.Vector2(windowSize[0] / 2, windowSize[1] / 2)
        self.motorSpeed = 1

        self.northVector = pygame.math.Vector2(0, -50)
        self.pointingVector = self.northVector
    
    def CalculateRotationPerFrame(self):
        wheelAngularVelocity_cps = (self.motorKv * self.batteryCells * LiPoCellVoltage_V * self.motorSpeed) / (self.motorReduction * 60)
        wheelTangentialVelocity_ms = (math.pi * (self.wheelDiameter_mm / 1000)) * wheelAngularVelocity_cps
        robotAngularVelocity_cps = wheelTangentialVelocity_ms / (math.pi * (self.diameter / 1000))
        
        rotationPerFrame = ((robotAngularVelocity_cps * 360) / FPS) % 360

        return rotationPerFrame
    
    def update(self):
        self.pointingVector = self.pointingVector.rotate(self.CalculateRotationPerFrame())
    
    def draw(self, surface):
        # Body
        pygame.draw.circle(surface, RobotColour, self.position, math.ceil(self.diameter / PixelsPermm))

        # Vectors
        pygame.draw.line(surface, NorthVectorColour, self.position, (self.northVector + self.position), 3)
        pygame.draw.line(surface, PointingVectorColour, self.position, (self.pointingVector + self.position), 3)

# Main
RobotSprite = RobotSprite(WindowSize, RobotDiamater_mm, RobotWheelSpacing_mm, RobotWheelDiameter_mm, RobotMotorReduction, RobotMotorKv, RobotBatteryCells)

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