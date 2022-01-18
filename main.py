import sys
import pygame
from pygame.locals import *
import math

# Setup Variables
FPS = 300
PixelsPermm = 10
MaximumBreadcrumbTrail = 500

ArenaColour = pygame.Color(255, 255, 255)
RobotColour = pygame.Color(65, 82, 110)
BreadcrumbColour = pygame.Color(170, 70, 15)
JoystickColour = pygame.Color(0, 0, 0)
SteeringVectorColour = pygame.Color(255, 0, 0)
HeadingVectorColour = pygame.Color(0, 0, 255)

WindowSize_px = (800, 800)
JoystickDiameter_px = 200
JoystickKnobDiameter_px = 20
BreadcrumbDiameter_px = 3
HeadingVectorLength_px = 30
SteeringVectorMaxLength_px = 100

# Robot Variables
RobotDiamater_mm = 250
RobotWheelSpacing_mm = 180
RobotWheelDiameter_mm = 56
RobotMotorReduction = 3
RobotMotorKv = 650
RobotBatteryCells = 4
RobotSpinThrottle = 0.1
RobotCellVoltage_V = 3.7

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
        mouseX_px = pygame.mouse.get_pos()[0]
        mouseY_px = pygame.mouse.get_pos()[1]

        deltaXSquared = (mouseX_px - self.center[0]) ** 2
        deltaYSquared = (mouseY_px - self.center[1]) ** 2

        thumbStickDistance_px = math.sqrt(deltaXSquared + deltaYSquared)

        if thumbStickDistance_px < (JoystickDiameter_px / 2) and thumbStickDistance_px > 0:
            self.joystickPosition = pygame.math.Vector2(mouseX_px, mouseY_px)

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
    def __init__(self, diameter_mm, wheelSpacing_mm, wheelDiameter_mm, motorReduction, motorKv, batteryCells, cellVoltage_V, spinThrottle):
        super().__init__()

        self.diameter_mm = diameter_mm
        self.wheelSpacing_mm = wheelSpacing_mm
        self.wheelDiameter_mm = wheelDiameter_mm

        motorMaxAngularVelocity_cps = (motorKv * batteryCells * cellVoltage_V) / (motorReduction * 60)
        self.maximumTangentialWheelVelocity_mms = math.pi * self.wheelDiameter_mm * motorMaxAngularVelocity_cps

        self.position_px = pygame.math.Vector2(WindowSize_px[0] / 2, WindowSize_px[1] / 2)
        self.angle_rad = 0

        self.spinThrottle = spinThrottle
        self.leftMotorThrottle = self.spinThrottle
        self.rightMotorThrottle = -self.spinThrottle

        self.steeringVector = pygame.math.Vector2()
        self.headingVector = pygame.math.Vector2()

        self.breadcrumbs = []

    def UpdatePosition(self):
        leftMotorVelocity_mms = self.maximumTangentialWheelVelocity_mms * self.leftMotorThrottle
        rightMotorVelocity_mms = self.maximumTangentialWheelVelocity_mms * self.rightMotorThrottle

        # Algorithm below almost completly lifted from here: http://rossum.sourceforge.net/papers/DiffSteer/
        try:
            turnRadius_mm = (self.wheelSpacing_mm / 2) * (rightMotorVelocity_mms + leftMotorVelocity_mms) / (rightMotorVelocity_mms - leftMotorVelocity_mms)

            # X and Y flipped and inverted to convert to the pygame cordinate system
            deltaX_mm = turnRadius_mm * (math.cos((rightMotorVelocity_mms - leftMotorVelocity_mms) * (1 / FPS) / self.wheelSpacing_mm + self.angle_rad) - math.cos(self.angle_rad))
            deltaY_mm = -turnRadius_mm * (math.sin((rightMotorVelocity_mms - leftMotorVelocity_mms) * (1 / FPS) / self.wheelSpacing_mm + self.angle_rad) - math.sin(self.angle_rad))
        
        except ZeroDivisionError:
            deltaX_mm = rightMotorVelocity_mms * (1 / FPS)
            deltaY_mm = rightMotorVelocity_mms * (1 / FPS)

        delta_px = pygame.math.Vector2(deltaX_mm / PixelsPermm, deltaY_mm / PixelsPermm)

        self.angle_rad = (rightMotorVelocity_mms - leftMotorVelocity_mms) * (1 / FPS) / self.wheelSpacing_mm + self.angle_rad
        self.angle_rad = self.angle_rad
        self.position_px = self.position_px + delta_px

        headingAngle_deg = math.degrees(-self.angle_rad)
        self.headingVector.from_polar((HeadingVectorLength_px, headingAngle_deg))
    
    def CalculateSpinAngle(self):
        spinTangentialWheelVelocity_mms = self.maximumTangentialWheelVelocity_mms * self.spinThrottle
        angularVelocity_rads = spinTangentialWheelVelocity_mms / (self.wheelSpacing_mm / 2)

        return math.degrees(angularVelocity_rads * (1 / FPS))
    
    def CalculateSteeringThrottle(self, startWheelPosition_mm, translationThrottle):
        # pygame .angle_to() seems to get confused by (0, 0) vectors
        if translationThrottle == 0: return 0

        rotatedSteeringVector = self.steeringVector.rotate(math.degrees(self.angle_rad))

        spinAngle_deg = self.CalculateSpinAngle()
        steerAngle_deg = (360 + startWheelPosition_mm.angle_to(rotatedSteeringVector)) / 2

        if steerAngle_deg > 90: steerAngle_deg = -steerAngle_deg
        
        extraAngle_deg = steerAngle_deg - spinAngle_deg

        extraWheelArcLength_mm = (self.wheelSpacing_mm / 2) * math.radians(extraAngle_deg)
        extraWheelTangentialVelocity_mms = extraWheelArcLength_mm / (1 / FPS)
        steeringThrottle = extraWheelTangentialVelocity_mms / self.maximumTangentialWheelVelocity_mms
        steeringThrottle = steeringThrottle * translationThrottle

        '''print(f'\nSteering vector: {self.steeringVector}')
        print(f'Steering trottle: {translationThrottle}')
        print(f'Start wheel postion: {startWheelPosition_mm}')
        print(f'Spin angle (deg): {spinAngle_deg}')
        print(f'Steer angle (deg): {steerAngle_deg}')
        print(f'Extra angle (deg): {extraAngle_deg}')
        print(f'extraWheelArcLength_mm: {extraWheelArcLength_mm}')
        print(f'extraWheelTangentialVelocity_mms: {extraWheelTangentialVelocity_mms}')
        print(f'steeringThrottle: {steeringThrottle}')'''

        return steeringThrottle
    
    def update(self, steeringVector):
        self.steeringVector = steeringVector
        translationThrottle = self.steeringVector.length()

        leftWheelPositionVector_mm = pygame.math.Vector2(-self.wheelSpacing_mm / 2, 0)
        rightWheelPositionVector_mm = pygame.math.Vector2(self.wheelSpacing_mm / 2, 0)

        leftSteeringThrottle = self.CalculateSteeringThrottle(leftWheelPositionVector_mm, translationThrottle)
        rightSteeringThrottle = self.CalculateSteeringThrottle(rightWheelPositionVector_mm, translationThrottle)

        self.leftMotorThrottle = self.spinThrottle + leftSteeringThrottle
        self.rightMotorThrottle = -self.spinThrottle + rightSteeringThrottle

        self.leftMotorThrottle = max(min(self.leftMotorThrottle, 1), 0)
        self.rightMotorThrottle = max(min(self.rightMotorThrottle, 0), -1)

        '''print(f'\nleftSteeringThrottle: {leftSteeringThrottle}')
        print(f'leftMotorThrottle: {self.leftMotorThrottle}')
        print(f'rightSteeringThrottle: {rightSteeringThrottle}')
        print(f'rightMotorThrottle: {self.rightMotorThrottle}')'''
        
        self.UpdatePosition()
    
    def draw(self, surface):
        # Body
        pygame.draw.circle(surface, RobotColour, self.position_px, math.ceil(self.diameter_mm / 2 / PixelsPermm))

        # Vectors
        pygame.draw.line(surface, SteeringVectorColour, self.position_px, ((self.steeringVector * SteeringVectorMaxLength_px) + self.position_px), 3)
        pygame.draw.line(surface, HeadingVectorColour, self.position_px, (self.headingVector + self.position_px), 3)

        # Breadcrumbs
        for breadcrumb in self.breadcrumbs:
            pygame.draw.circle(surface, BreadcrumbColour, breadcrumb, (BreadcrumbDiameter_px / 2))

        breadcrumb = [math.ceil(self.position_px[0]), math.ceil(self.position_px[1])]
        self.breadcrumbs.append(breadcrumb)

        if len(self.breadcrumbs) >= MaximumBreadcrumbTrail:
            self.breadcrumbs.pop(0)

# Main
joystick = Joystick()
robot = Robot(RobotDiamater_mm, RobotWheelSpacing_mm, RobotWheelDiameter_mm, RobotMotorReduction, RobotMotorKv, RobotBatteryCells, RobotCellVoltage_V, RobotSpinThrottle)

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