import sys
import pygame
from pygame.locals import *
import pygame.freetype
import math
import matplotlib.pyplot as plt

# Setup Variables
FPS = 30
PixelsPermm = 10
MaximumBreadcrumbTrail = 0
MaximumReadings = 50

ArenaColour = pygame.Color(255, 255, 255)
StatsTextColour = pygame.Color(0, 0, 0)
RobotColour = pygame.Color(65, 82, 110)
BreadcrumbColour = pygame.Color(170, 70, 15)
JoystickColour = pygame.Color(0, 0, 0)
SteeringVectorColour = pygame.Color(255, 0, 0)
HeadingVectorColour = pygame.Color(0, 255, 0)

WindowSize_px = (800, 800)
StatsRootLocation_px = (25, 25)

JoystickDiameter_px = 200
JoystickKnobDiameter_px = 20
BreadcrumbDiameter_px = 3
HeadingVectorLength_px = 30
SteeringVectorMaxLength_px = 100
StatsTextHeight_px = 20

# Robot Variables
RobotDiamater_mm = 250
RobotWheelSpacing_mm = 180
RobotWheelDiameter_mm = 56
RobotMotorReduction = 3
RobotMotorKv = 650
RobotBatteryCells = 4
RobotSpinThrottle = 0.3
RobotCellVoltage_V = 3.7

# Pygame Setup
pygame.init()

StatsFont = pygame.freetype.SysFont('Comic Sans MS', StatsTextHeight_px)
Clock = pygame.time.Clock()

DisplaySurface = pygame.display.set_mode(WindowSize_px)
pygame.display.set_caption("Meltybrain Simulator")

# Matplotlib Setup
def CloseApp(event):
    pygame.quit()

fig, axs = plt.subplots(3, 1)
fig.canvas.mpl_connect('close_event', CloseApp)

xs = []
LeftWheelData = []
RightWheelData = []
AngularVelocityData = []

plt.ion()
plt.show()

class Joystick(pygame.sprite.Sprite):
    def __init__(self):
        super().__init__()

        self.center = pygame.math.Vector2(WindowSize_px[0] - (JoystickDiameter_px / 2), WindowSize_px[1] - (JoystickDiameter_px / 2))
        self.joystickPosition = self.center
    
    def Update(self):
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
    
    def Draw(self, surface):
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
            deltaX_mm = turnRadius_mm * (math.cos((rightMotorVelocity_mms - leftMotorVelocity_mms) * (1.0 / FPS) / self.wheelSpacing_mm + self.angle_rad) - math.cos(self.angle_rad))
            deltaY_mm = -turnRadius_mm * (math.sin((rightMotorVelocity_mms - leftMotorVelocity_mms) * (1.0 / FPS) / self.wheelSpacing_mm + self.angle_rad) - math.sin(self.angle_rad))
        
        except ZeroDivisionError:
            deltaX_mm = rightMotorVelocity_mms * (1.0 / FPS)
            deltaY_mm = rightMotorVelocity_mms * (1.0 / FPS)

        delta_px = pygame.math.Vector2(deltaX_mm / PixelsPermm, deltaY_mm / PixelsPermm)

        deltaVector_mm = pygame.math.Vector2(deltaX_mm, deltaY_mm)
        deltaAngle_rad = (rightMotorVelocity_mms - leftMotorVelocity_mms) * (1.0 / FPS) / self.wheelSpacing_mm

        self.angle_rad = deltaAngle_rad + self.angle_rad
        self.position_px = self.position_px + delta_px
        self.angularVelocity_rpm = abs(deltaAngle_rad / (1.0 / FPS)) * 9.5493

        headingAngle_deg = math.degrees(-self.angle_rad)
        self.headingVector.from_polar((HeadingVectorLength_px, headingAngle_deg))

        return (leftMotorVelocity_mms, rightMotorVelocity_mms, self.angularVelocity_rpm)
    
    def CalculateSpinAngle(self):
        spinTangentialWheelVelocity_mms = self.maximumTangentialWheelVelocity_mms * self.spinThrottle
        angularVelocity_rads = spinTangentialWheelVelocity_mms / (self.wheelSpacing_mm / 2)

        return math.degrees(angularVelocity_rads * (1.0 / FPS))
    
    def CalculateSteeringThrottle(self, startWheelPosition_mm, translationThrottle):
        # pygame .angle_to() seems to get confused by (0, 0) vectors
        if translationThrottle == 0: return 0

        rotatedSteeringVector = self.steeringVector.rotate(math.degrees(self.angle_rad))

        spinAngle_deg = self.CalculateSpinAngle()
        steerAngle_deg = (360 + startWheelPosition_mm.angle_to(rotatedSteeringVector)) / 2

        if steerAngle_deg > 90: steerAngle_deg = -steerAngle_deg
        
        extraAngle_deg = steerAngle_deg - spinAngle_deg

        extraWheelArcLength_mm = (self.wheelSpacing_mm / 2) * math.radians(extraAngle_deg)
        extraWheelTangentialVelocity_mms = extraWheelArcLength_mm / (1.0 / FPS)
        steeringThrottle = extraWheelTangentialVelocity_mms / self.maximumTangentialWheelVelocity_mms
        steeringThrottle = steeringThrottle * translationThrottle

        return steeringThrottle
    
    def Update(self, steeringVector):
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
        
        return self.UpdatePosition()
    
    def Draw(self, surface):
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
    
    def GetStats(self):
        stats = [   f'Left Motor Throttle: {self.leftMotorThrottle}',
                    f'Right Motor Throttle: {self.rightMotorThrottle}',
                    f'Angular Velocity (RPM): {self.angularVelocity_rpm}' ]
        
        return stats


def DrawStats(stats):
    for i in range(0, len(stats)):
        stat_position_px = [StatsRootLocation_px[0], StatsRootLocation_px[1]]
        stat_position_px[1] = stat_position_px[1] + (i * StatsTextHeight_px)
        StatsFont.render_to(DisplaySurface, stat_position_px, stats[i], StatsTextColour)

def DrawChart(config):
    axs[0].clear()
    axs[1].clear()
    axs[2].clear()

    axs[0].set_title('Left Wheel Velocity') 
    axs[0].set_xlabel('Time (s)')
    axs[0].set_ylabel('Tangential Velocity (mm/s)')

    axs[1].set_title('Right Wheel Velocity') 
    axs[1].set_xlabel('Time (s)')
    axs[1].set_ylabel('Tangential Velocity (mm/s)')

    axs[2].set_title('Robot Angular Velocity') 
    axs[2].set_xlabel('Time (s)')
    axs[2].set_ylabel('Angular Velocity (RPM)')

    axs[0].grid(True)
    axs[1].grid(True)
    axs[2].grid(True)
    
    xs.append(pygame.time.get_ticks() / 1000)

    LeftWheelData.append(config[0])
    RightWheelData.append(config[1])
    AngularVelocityData.append(config[2])

    if len(xs) >= MaximumReadings:
        xs.pop(0)
        LeftWheelData.pop(0)
        RightWheelData.pop(0)
        AngularVelocityData.pop(0)
    
    axs[0].plot(xs, LeftWheelData, color='red')
    axs[1].plot(xs, RightWheelData, color='green')
    axs[2].plot(xs, AngularVelocityData, color='blue')

    plt.draw()
    plt.pause(0.001)


# Main
joystick = Joystick()
robot = Robot(RobotDiamater_mm, RobotWheelSpacing_mm, RobotWheelDiameter_mm, RobotMotorReduction, RobotMotorKv, RobotBatteryCells, RobotCellVoltage_V, RobotSpinThrottle)

while True:     
    for event in pygame.event.get():              
        if event.type == QUIT:
            pygame.quit()
            sys.exit()
    
    steeringVector = joystick.Update()
    config = robot.Update(steeringVector)
    stats = robot.GetStats()

    DisplaySurface.fill(ArenaColour)
    joystick.Draw(DisplaySurface)
    robot.Draw(DisplaySurface)
    #DrawStats(stats)
    DrawChart(config)
    
    pygame.display.update()
    Clock.tick(FPS)