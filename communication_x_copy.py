
import math
import sim
import numpy as np
import threading as th
import readchar

runSim = True

#Speed of the two wheels that we're going to control with this script
leftVelocity = 0
rightVelocity = 0
velocity = 0

# this is the step size by which the user can incremment and decrement speeds
v_increment = 0.1 

#These help us with the task of printing object locations to the console
objectleft = False
objectright = False
objectfront = False

#how close do we have to get to an object before we say anything?
obj_min_thr = 0.003 

#and when are we happy that we've cleared the object again?
obj_max_thr = 0.03

#when are we sure that the error could be neglected?
min_error = 1e-5

sim.simxFinish(-1) 
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) 

#Object distance (we're going to read this from a sensor)
sensor_readings = np.nan

#Set the initial velocity of the epuck to 0
velocities = sim.simxPackFloats([0, 0])
sim.simxSetStringSignal(clientID=clientID, signalName="Motors", signalValue=velocities, operationMode=sim.simx_opmode_blocking)

class PID:
    def __init__(self, KP, KI, KD):
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.integratedError = 0
        self.last = None

    def update(self, target, value, dt):
        error = target - value

        self.integratedError = self.integratedError + self.KI * error * dt

        if self.last == None:
            self.last = value

        derivative = (value - self.last) / dt

        self.last = value

        P = self.KP * error * dt
        I = self.integratedError
        D = -self.KD * derivative

        # print("PID", P, I, D)

        return P + I + D

# for some reason values in the range 128..255
# get encoded as -128..-1
def fixPixelComponent(value):
    if value < 0:
        value += 256
    return value

def leftHalf(resolution, image):
    result = []
    width = resolution[0]
    height = resolution[1]
    for y in range(0, height):
        for x in range(0, width//2):
            pixel = (y*width+x)*3
            result.append(image[pixel])
            result.append(image[pixel+1])
            result.append(image[pixel+2])
    return ((width//2, height), result)

def rightHalf(resolution, image):
    result = []
    width = resolution[0]
    height = resolution[1]
    for y in range(0, height):
        for x in range(width//2, width):
            pixel = (y*width+x)*3
            result.append(image[pixel])
            result.append(image[pixel+1])
            result.append(image[pixel+2])
    return ((width//2, height), result)

def averageColor(resolution, image, row):
    r = 0
    g = 0
    b = 0
    width = resolution[0]
    offs = row * width * 3
    for i in range(0, width):
        r += fixPixelComponent(image[offs+3*i])
        g += fixPixelComponent(image[offs+3*i+1])
        b += fixPixelComponent(image[offs+3*i+2])
    return (r/width, g/width, b/width)

def greenness(resolution, image):
    r = 0
    g = 0
    b = 0
    height = resolution[1]
    for i in range(0, height):
        color = averageColor(resolution, image, i)
        r += color[0]
        g += color[1]
        b += color[2]
    return (g - (r + g + b) / 3) / (height * 256)

def greennessRow(resolution, image, row):
    color = averageColor(resolution, image, row)
    return (color[1] - (color[0]+color[1]+color[2]) / 3) / 256

def slope(resolution, image):
    height = resolution[1]
    top = greennessRow(resolution, image, 0)
    bottom = greennessRow(resolution, image, height-1)
    return bottom - top

if clientID != -1:
    pidLeft = PID(4.0, 0.1, 0.01)
    pidRight = PID(4.0, 0.1, 0.01)
#    pidLeft = PID(0.2, 0.0, 0.0)
#    pidRight = PID(0.2, 0.0, 0.0)

    targetVelocity = 3.0

    targetLeft = targetVelocity
    targetRight = targetVelocity

    return_code,sensorFront = sim.simxGetObjectHandle(clientID,"Vision_sensor", sim.simx_opmode_blocking)
    if return_code != 0:
        print ('Failed getting front vision sensor')
        quit()

    return_code,sensorBack = sim.simxGetObjectHandle(clientID,"Vision_sensor0", sim.simx_opmode_blocking)
    if return_code != 0:
        print ('Failed getting back vision sensor')
        quit()

    return_code,sensorLeft = sim.simxGetObjectHandle(clientID,"Vision_sensor1", sim.simx_opmode_blocking)
    if return_code != 0:
        print ('Failed getting left vision sensor')
        quit()

    return_code,sensorRight = sim.simxGetObjectHandle(clientID,"Vision_sensor2", sim.simx_opmode_blocking)
    if return_code != 0:
        print ('Failed getting right vision sensor')
        quit()

    print('setting sensor parameters')

    for sensor in [sensorFront, sensorBack, sensorLeft, sensorRight]:
        sim.simxSetObjectIntParameter(clientID, sensor, 1002, 20, sim.simx_opmode_blocking) # resolution x
        sim.simxSetObjectIntParameter(clientID, sensor, 1003, 20, sim.simx_opmode_blocking) # resolution y
        sim.simxSetObjectFloatParameter(clientID, sensor, 1000, 0.028, sim.simx_opmode_blocking) # near clip
        sim.simxSetObjectFloatParameter(clientID, sensor, 1005, 0.075, sim.simx_opmode_blocking) # ortho size

    print('running')

    #this loops runs until the user presses "t"
    while runSim:
        ###THIS IS THE COMMUNICATION WITH COPPELIASIM - WE JUST SEND THE DESIRED VELOCITIES AND READ WHAT THE SENSORS SAY
        leftVelocity += pidLeft.update(targetLeft, leftVelocity, 0.1)
        rightVelocity += pidRight.update(targetRight, rightVelocity, 0.1)

        #We send the desired velocities for both motors to the simulator by packing the floats into something that can be sent over a string signal
        velocities = sim.simxPackFloats([leftVelocity, rightVelocity])
        sim.simxSetStringSignal(clientID=clientID, signalName="Motors", signalValue=velocities, operationMode=sim.simx_opmode_blocking)

        return_code, resolutionFront, imageFront = sim.simxGetVisionSensorImage(clientID, sensorHandle=sensorFront, options=0, operationMode=sim.simx_opmode_blocking)
        if return_code != 0:
            print('Failed to get front image')
            quit()

        return_code, resolutionBack, imageBack = sim.simxGetVisionSensorImage(clientID, sensorHandle=sensorBack, options=0, operationMode=sim.simx_opmode_blocking)
        if return_code != 0:
            print('Failed to get back image')
            quit()

        return_code, resolutionLeft, imageLeft = sim.simxGetVisionSensorImage(clientID, sensorHandle=sensorLeft, options=0, operationMode=sim.simx_opmode_blocking)
        if return_code != 0:
            print('Failed to get left image')
            quit()

        return_code, resolutionRight, imageRight = sim.simxGetVisionSensorImage(clientID, sensorHandle=sensorRight, options=0, operationMode=sim.simx_opmode_blocking)
        if return_code != 0:
            print('Failed to get Right image')
            quit()

        resolutionFrontLeft, imageFrontLeft = leftHalf(resolutionFront, imageFront)
        resolutionFrontRight, imageFrontRight = rightHalf(resolutionFront, imageFront)

        slopeFrontLeft = slope(resolutionFrontLeft, imageFrontLeft)
        slopeFrontRight = -slope(resolutionFrontRight, imageFrontRight)

        slopeLeft = slope(resolutionLeft, imageLeft)
        slopeRight = -slope(resolutionRight, imageRight)

        # main error signal, based on computed slope of the path relative to the robot
        signal = slopeFrontLeft + slopeFrontRight + slopeLeft + slopeRight

        greennessFrontLeft = greenness(resolutionFrontLeft, imageFrontLeft)
        greennessFrontRight = greenness(resolutionFrontRight, imageFrontRight)
        greennessLeft = greenness(resolutionLeft, imageLeft)
        greennessRight = greenness(resolutionRight, imageRight)

        # small error signal, based on how much greenness there is to the left vs to the right of the robot
        # meant to keep the robot in the middle of the path with the help of the integrator part of the PID
        # by accumulating this small error over time
        signal += 0.05 * (greennessFrontLeft + greennessLeft - greennessFrontRight - greennessRight)

        abs_signal = abs(signal)

        # damp the speed when the error signal is large, to give the robot time to maneuver sharp turns
        damping = 1
        if abs_signal > 0.05:
            damping = max(0.2, 1 / math.exp(abs_signal * 6))
        dampedVelocity = targetVelocity * damping

        correction = 2.5 * abs_signal

        if signal > 0:
            targetLeft = max(0, dampedVelocity - correction)
            targetRight = dampedVelocity
        else:
            targetLeft = dampedVelocity
            targetRight = max(0, dampedVelocity - correction)

        # print("damped velocity", dampedVelocity, "targetLeft", targetLeft, "targetRight", targetRight, "signal", signal)

    #Set the initial velocity of the epuck to 0
    velocities = sim.simxPackFloats([0, 0])
    sim.simxSetStringSignal(clientID=clientID, signalName="Motors", signalValue=velocities, operationMode=sim.simx_opmode_blocking)

    #End connection
    sim.simxGetPingTime(clientID)
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
