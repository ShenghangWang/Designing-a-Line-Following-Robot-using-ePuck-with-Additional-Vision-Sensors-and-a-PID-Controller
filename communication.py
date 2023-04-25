
import sim
import numpy as np
import threading as th
import readchar

runSim = True

#Speed of the two wheels that we're going to control with this script
leftVelocity = 0
rightVelocity = 0

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

#This thread reads keys from the user - it's how we increase and decrese the speed of the robot
def th_key_capture():
    global runSim

    global leftVelocity
    global rightVelocity

    user_input = ""

    while user_input != "t":
        user_input = readchar.readchar()

        #We stop when the user presses t
        if user_input == "t":
            runSim = False

        #Stop the epuck (both speeds set to 0) when the user presses v
        if user_input == "v":
            leftVelocity = 0
            rightVelocity = 0

        #increment or decrement the speed of the left wheel
        elif user_input == "q":
            leftVelocity = 20#leftVelocity + v_increment
        elif user_input == "a":
            leftVelocity = leftVelocity - v_increment


         #increment or decrement the speed of the right wheel
        elif user_input == "p":
            rightVelocity = rightVelocity + v_increment
        elif user_input == "l":
            rightVelocity = rightVelocity - v_increment

print ('Program started')

print('To control the e-puck:')
print('q/a to increase/decrease the left motor velocity')
print('p/l to increase/decrease the right motor velocity')
print('v to set the speed to 0')
print('To quit: press t')

#Just a thread to capture key presses
th.Thread(target=th_key_capture, args=(), name='th_key_capture', daemon=True).start()


sim.simxFinish(-1) 
clientID=sim.simxStart('127.0.0.1',19997,True,True,5000,5) 

#Object distance (we're going to read this from a sensor)
sensor_readings = np.nan

#Set the initial velocity of the epuck to 0
velocities = sim.simxPackFloats([0, 0])
sim.simxSetStringSignal(clientID=clientID, signalName="Motors", signalValue=velocities, operationMode=sim.simx_opmode_blocking)


if clientID != -1:

    #this loops runs until the user presses "t"
    while runSim:


        ###THIS IS THE COMMUNICATION WITH COPPELIASIM - WE JUST SEND THE DESIRED VELOCITIES AND READ WHAT THE SENSORS SAY


        #We send the desired velocities for both motors to the simulator by packing the floats into something that can be sent over a string signal
        velocities = sim.simxPackFloats([leftVelocity, rightVelocity])
        sim.simxSetStringSignal(clientID=clientID, signalName="Motors", signalValue=velocities, operationMode=sim.simx_opmode_blocking)


        #next, we try to read what the sensors say. This signal is set in CoppeliaSim. If the return code is not 0, this didn't work, 
        #don't try to do something with the data either. NOTE we use blocking mode here, so this will wait until a value can be read or 
        #attempting to read times out.
        return_code, return_value = sim.simxGetStringSignal(clientID=clientID, signalName="Sensors", operationMode=sim.simx_opmode_blocking)


        #### 



        if return_code == 0:
            sensor_readings = sim.simxUnpackFloats(return_value)

            #we don't do anything with the information we got, just print info about where an object might be. 
            #Since we don't want to spam the console (this will be triggered at every time step), we use the booleans to keep track of what we did
            if not objectfront:

                #to detect an object at the front, we take the average values of both front sensors (not necessarily ideal)
                if (sensor_readings[2] + sensor_readings[3])/2 < obj_min_thr:
                    print("Object straight ahead")
                    objectfront = True
            else:
                if (sensor_readings[2] + sensor_readings[3])/2 > obj_max_thr:
                    print("Object front cleared")
                    objectfront = False

            if not objectleft:
                if sensor_readings[0] < obj_min_thr:
                    print("Object to the left")
                    objectleft = True
            else:
                if sensor_readings[0] > obj_max_thr:
                    print("Object left cleared")
                    objectleft = False    

            if not objectright:
                if sensor_readings[5] < obj_min_thr:
                    print("Objet to the right")
                    objectright = True
            else:
                if sensor_readings[5] > obj_max_thr:
                    print("Objet right cleared")
                    objectright = False


    #Set the initial velocity of the epuck to 0
    velocities = sim.simxPackFloats([0, 0])
    sim.simxSetStringSignal(clientID=clientID, signalName="Motors", signalValue=velocities, operationMode=sim.simx_opmode_blocking)



    #End connection
    sim.simxGetPingTime(clientID)
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
