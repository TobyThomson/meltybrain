import sys
import pygame
from pygame.locals import *
import math

# Setup Variables
FPS = 60

ArenaColour = pygame.Color(255, 255, 255)
RobotColour = pygame.Color(65, 82, 110)
BreadcrumbColour = pygame.Color(170, 70, 15)
JoystickColour = pygame.Color(0, 0, 0)
NorthVectorColour = pygame.Color(255, 0, 0)
steeringVectorColour = pygame.Color(0, 255, 0)
headingVectorColour = pygame.Color(0, 0, 255)

PixelsPermm = 10

WindowSize_px = (800, 800)
JoystickDiameter_px = 200
JoystickKnobDiameter_px = 20
BreadcrumbDiameter_px = 3

LiPoCellVoltage_V = 3.7

MaximumBreadcrumbTrail = 100

# Robot Variables
RobotDiamater_mm = 250
RobotWheelSpacing_mm = 180
RobotWheelDiameter_mm = 56
RobotMotorReduction = 3
RobotMotorKv = 650
RobotBatteryCells = 4
RobotSpinThrottle = 0.01

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

            steeringVector = self.joystickPosition - self.center
            steeringVector.scale_to_length(steeringVector.length() * scaleFactor)

            return steeringVector
        
        else:
            self.joystickPosition = self.center

            return pygame.math.Vector2(0, 0)
    
    def draw(self, surface):
        pygame.draw.circle(surface, JoystickColour, self.center, (JoystickDiameter_px / 2), 3)
        pygame.draw.circle(surface, JoystickColour, self.joystickPosition, (JoystickKnobDiameter_px / 2))

class Robot(pygame.sprite.Sprite):
    def __init__(self, diameter_mm, wheelSpacing_mm, wheelDiameter_mm, motorReduction, motorKv, batteryCells, spinThrottle):
        super().__init__()

        self.diameter_mm = diameter_mm
        self.wheelSpacing_mm = wheelSpacing_mm
        self.wheelDiameter_mm = wheelDiameter_mm

        motorTopVelocity_cps = (motorKv * batteryCells * LiPoCellVoltage_V) / (motorReduction * 60)
        self.maximumTangentialWheelVelocity_mms = math.pi * self.wheelDiameter_mm * motorTopVelocity_cps

        self.position_px = pygame.math.Vector2(WindowSize_px[0] / 2, WindowSize_px[1] / 2)
        self.angle_rad = 0

        self.spinThrottle = spinThrottle
        self.leftMotorThrottle = self.spinThrottle / 2
        self.rightMotorThrottle = self.spinThrottle / 2

        self.northVector = pygame.math.Vector2(0, -50)
        self.steeringVector = pygame.math.Vector2(0, 0)
        self.headingVector = self.northVector

        self.breadcrumbs = []

    def UpdatePosition(self):
        leftMotorVelocity_mms = self.maximumTangentialWheelVelocity_mms * self.leftMotorThrottle
        rightMotorVelocity_mms = self.maximumTangentialWheelVelocity_mms * self.rightMotorThrottle

        # Algorithm below almost completly lifted from here: http://rossum.sourceforge.net/papers/DiffSteer/
        try:
            turnRadius_mm = (self.wheelSpacing_mm / 2) * (rightMotorVelocity_mms + leftMotorVelocity_mms) / (rightMotorVelocity_mms - leftMotorVelocity_mms)

            deltaX_mm = turnRadius_mm * (math.sin((rightMotorVelocity_mms - leftMotorVelocity_mms) * (1 / FPS) / self.wheelSpacing_mm + self.angle_rad) - math.sin(self.angle_rad))
            deltaY_mm = -turnRadius_mm * (math.cos((rightMotorVelocity_mms - leftMotorVelocity_mms) * (1 / FPS) / self.wheelSpacing_mm + self.angle_rad) - math.cos(self.angle_rad))
        
        except ZeroDivisionError:
            deltaX_mm = rightMotorVelocity_mms * (1 / FPS)
            deltaY_mm = rightMotorVelocity_mms * (1 / FPS)

        delta_px = pygame.math.Vector2(deltaX_mm / PixelsPermm, deltaY_mm / PixelsPermm)

        self.angle_rad = (rightMotorVelocity_mms - leftMotorVelocity_mms) * (1 / FPS) / self.wheelSpacing_mm + self.angle_rad
        self.position_px = self.position_px + delta_px

        headingAngle = self.angle_rad + (math.pi / 2)
        self.headingVector = self.northVector.rotate(math.degrees(headingAngle))
    
    def CalculateClosableAngle():
        spinTangentialWheelVelocity_mms = self.maximumTangentialWheelVelocity_mms * self.spinThrottle
        angularVelocity_rads = spinTangentialWheelVelocity_mms / (self.wheelSpacing_mm / 2)
        closableAngle_rad = angularVelocity * (1 / FPS)

        return closableAngle_rad
    
    def update(self, steeringVector):
        self.steeringVector = steeringVector

        # TODO: Need to replace these lines with the magic code that calculates correct motor throttle
        leftSteeringThrottle = abs(steeringVector[0])
        rightSteeringThrottle = -abs(steeringVector[1])

        self.leftMotorThrottle = self.spinThrottle + leftSteeringThrottle
        self.rightMotorThrottle = -self.spinThrottle + rightSteeringThrottle

        self.leftMotorThrottle = max(min(self.leftMotorThrottle, 1), 0)
        self.rightMotorThrottle = max(min(self.rightMotorThrottle, 0), -1)

        print(self.leftMotorThrottle, self.rightMotorThrottle)

        self.UpdatePosition()
    
    def draw(self, surface):
        # Body
        pygame.draw.circle(surface, RobotColour, self.position_px, math.ceil(self.diameter_mm / 2 / PixelsPermm))

        # Vectors
        pygame.draw.line(surface, NorthVectorColour, self.position_px, (self.northVector + self.position_px), 3)
        pygame.draw.line(surface, steeringVectorColour, self.position_px, ((self.steeringVector * 100) + self.position_px), 3)
        pygame.draw.line(surface, headingVectorColour, self.position_px, (self.headingVector + self.position_px), 3)

        # Breadcrumbs
        for breadcrumb in self.breadcrumbs:
            pygame.draw.circle(surface, BreadcrumbColour, breadcrumb, (BreadcrumbDiameter_px / 2))

        breadcrumb = [math.ceil(self.position_px[0]), math.ceil(self.position_px[1])]
        self.breadcrumbs.append(breadcrumb)

        if len(self.breadcrumbs) >= MaximumBreadcrumbTrail:
            self.breadcrumbs.pop(0)

# Main
joystick = Joystick()
robot = Robot(RobotDiamater_mm, RobotWheelSpacing_mm, RobotWheelDiameter_mm, RobotMotorReduction, RobotMotorKv, RobotBatteryCells, RobotSpinThrottle)

while True:     
    for event in pygame.event.get():              
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
    
    steeringVector = joystick.update()
    robot.update(steeringVector)

    DisplaySurface.fill(ArenaColour)

    joystick.draw(DisplaySurface)
    robot.draw(DisplaySurface)

    pygame.display.update()
    Clock.tick(FPS)