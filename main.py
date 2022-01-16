import sys
import pygame
from pygame.locals import *
import math

# Setup Variables
FPS = 30

ArenaColour = pygame.Color(255, 255, 255)
RobotColour = pygame.Color(65, 82, 110)
JoystickColour = pygame.Color(0, 0, 0)
NorthVectorColour = pygame.Color(255, 0, 0)
HeadingVectorColour = pygame.Color(0, 255, 0)
PointingVectorColour = pygame.Color(0, 0, 255)

PixelsPermm = 3

WindowSize_px = (800, 800)
JoystickDiameter_px = 200
JoystickKnobDiameter_px = 20

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

DisplaySurface = pygame.display.set_mode(WindowSize_px)
pygame.display.set_caption("Meltybrain Simulator")

class Joystick(pygame.sprite.Sprite):
    def __init__(self):
        super().__init__()

        self.center = pygame.math.Vector2(WindowSize_px[0] - (JoystickDiameter_px / 2), WindowSize_px[1] - (JoystickDiameter_px / 2))
        self.joystickPosition = self.center
    
    def update(self):
        mouseX = pygame.mouse.get_pos()[0]
        mouseY = pygame.mouse.get_pos()[1]

        deltaXSquared = (mouseX - self.center[0]) ** 2
        deltaYSquared = (mouseY - self.center[1]) ** 2

        if math.sqrt(deltaXSquared + deltaYSquared) < (JoystickDiameter_px / 2):
            self.joystickPosition = pygame.math.Vector2(mouseX, mouseY)

            scaleFactor = 1 / (JoystickDiameter_px / 2)

            headingVector = self.joystickPosition - self.center
            headingVector.scale_to_length(headingVector.length() * scaleFactor)

            return headingVector
        
        else:
            self.joystickPosition = self.center

            return pygame.math.Vector2(0, 0)
    
    def draw(self, surface):
        pygame.draw.circle(surface, JoystickColour, self.center, (JoystickDiameter_px / 2), 3)
        pygame.draw.circle(surface, JoystickColour, self.joystickPosition, (JoystickKnobDiameter_px / 2))

class Robot(pygame.sprite.Sprite):
    def __init__(self, diameter, wheelSpacing_mm, wheelDiameter_mm, motorReduction, motorKv, batteryCells):
        super().__init__()

        self.diameter = diameter
        self.wheelSpacing_mm = wheelSpacing_mm
        self.wheelDiameter_mm = wheelDiameter_mm

        self.motorReduction = motorReduction
        self.motorKv = motorKv
        self.batteryCells = batteryCells

        self.position = pygame.math.Vector2(WindowSize_px[0] / 2, WindowSize_px[1] / 2)
        self.motorSpeed = 1

        self.northVector = pygame.math.Vector2(0, -50)
        self.headingVector = pygame.math.Vector2(0, 0)
        self.pointingVector = self.northVector
    
    def CalculateRotationPerFrame(self):
        wheelAngularVelocity_cps = (self.motorKv * self.batteryCells * LiPoCellVoltage_V * self.motorSpeed) / (self.motorReduction * 60)
        wheelTangentialVelocity_ms = (math.pi * (self.wheelDiameter_mm / 1000)) * wheelAngularVelocity_cps
        robotAngularVelocity_cps = wheelTangentialVelocity_ms / (math.pi * (self.diameter / 1000))
        
        rotationPerFrame = ((robotAngularVelocity_cps * 360) / FPS) % 360

        return rotationPerFrame
    
    def update(self, headingVector):
        self.pointingVector = self.pointingVector.rotate(self.CalculateRotationPerFrame())
        self.headingVector = headingVector
    
    def draw(self, surface):
        # Body
        pygame.draw.circle(surface, RobotColour, self.position, math.ceil(self.diameter / 2 / PixelsPermm))

        # Vectors
        pygame.draw.line(surface, NorthVectorColour, self.position, (self.northVector + self.position), 3)
        pygame.draw.line(surface, HeadingVectorColour, self.position, ((self.headingVector * 100) + self.position), 3)
        pygame.draw.line(surface, PointingVectorColour, self.position, (self.pointingVector + self.position), 3)

# Main
joystick = Joystick()
robot = Robot(RobotDiamater_mm, RobotWheelSpacing_mm, RobotWheelDiameter_mm, RobotMotorReduction, RobotMotorKv, RobotBatteryCells)

while True:     
    for event in pygame.event.get():              
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
    
    headingVector = joystick.update()
    robot.update(headingVector)

    DisplaySurface.fill(ArenaColour)

    joystick.draw(DisplaySurface)
    robot.draw(DisplaySurface)

    pygame.display.update()
    Clock.tick(FPS)