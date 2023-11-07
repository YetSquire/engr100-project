"""
University of Michigan
ENG100-400
"""

import sys, platform
from pathlib import Path
if platform.system() == 'Darwin':
    airsim_install = '$HOME/AirSim'
else:
    airsim_install = 'C:\\AirSim'
sys.path.append(str(Path(airsim_install) / 'PythonClient'))
sys.path.append(str(Path(airsim_install) / 'PythonClient' / 'multirotor'))

############### import a few useful libraries ###########

#import setup_path
import time
import numpy as np
import math
import matplotlib.pyplot as plt

############### establish the link to AirSim ###########

import airsim              # import AirSim API
import E100_functions      # import drone simulator library

dt = E100_functions.dt()  
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

#### Copy and paste the above in your own flight controller #####

initX, initY = E100_functions.get_XY(client);
initZ = E100_functions.get_barometer(client);

currentX = initX
currentY = initY
currentZ = initZ

print(initX);
print(" ");
print(initY);
print(" ");
print(initZ);
print(" ");

K_P = 1.2
K_I = 0.5
K_D = 0.5
#############

alpha = .35
lastLPF = 0
currentLPF = 0
integration_term = 0
throttle = 0.625  # initialize Throttle
wind_flag = 1
error_old = 0

alt_log = []

#throttle keeps z constant
#pitch is from building (existing wind concept)
#roll is 

alpha_alt = 0.7
alpha_lidar = 1.0
start = time.time()

def stabilize(error_old, integration_term, was_moving_right):
    currentY, currentX = E100_functions.get_XY(client); #swapped because 
      #possibly change to a variable later
    error= abs(20-currentY)
    
    differential_term = (error - error_old)/dt    
    if K_P*error + K_I*integration_term + K_D*differential_term <0:
        roll = 0
    else:
        roll = K_P*error + K_I*integration_term + K_D*differential_term
        if was_moving_right:
            roll*=-2
        else: roll*=2
    if roll >= 1:
            if was_moving_right:
                roll*=-2
            else:
                roll*=2
    else: 
        E100_functions.set_quadcopter(client, roll,0,0,throttle)
    return error

def moveLeft():        
        E100_functions.set_quadcopter(client,-10,0,0,throttle)
    
def moveRight():
        E100_functions.set_quadcopter(client,10,0,0,throttle)


while True:
    now = time.time()
    if now - start < 5: 
        E100_functions.set_quadcopter(client,0,0,0,0.7)
    else:
        currentY, currentX = E100_functions.get_XY(client); #swapped because 
        currentZ = E100_functions.get_barometer(client);
        yVel, xVel, zVel = E100_functions.get_linear_velocity(client)
        alt_log.append(currentY)
        plt.plot(alt_log)
        targetDist = 20
        startStab = 10
        if abs(currentY - initY) < startStab:
            moveRight()
        elif abs(currentY - initY) < targetDist:
            if yVel > 0: t = True
            holder = stabilize(error_old, integration_term, t)
            error_old = holder
            integration_term += holder*dt
        elif yVel != 0:
            t = False
            if yVel > 0: 
                t = True
                t = False
            holder = stabilize(error_old, integration_term, t)
            error_old = holder
            integration_term += holder*dt
        else: 
            E100_functions.set_quadcopter(client,0,0,0,0.6)
            



"""target_alt = 100
K_P = 1
K_I = 0.
K_D = 2
alt_integration_term = 0
alt_flag = 1
throttle = 0.5  # initialize Throttle
##############################################
target_front_dist = 10
pitch_rate = 0
pitch_K_P = 1
pitch_K_I = 0.0
pitch_K_D = 0.75
pitch_integration_term = 0
pitch_flag = 1
desired_pitch = 0
##############################################
target_right_dist = 5
roll_K_P = 0.5
roll_K_I = 0.001
roll_K_D = 0.65
roll_integration_term = 0
roll_flag = 1
desired_roll = 0 
desired_yaw = 0

start = time.time()

altitude_sensor_flag = 1
Lidar_sensor_flag = 1

x_pos = []
y_pos = []
lat_error = []

while True:
           
    now = time.time()
    if now - start > 42:
        target_alt = 0
    if now - start > 45:   
        break
    
    #control signal 
    E100_functions.set_quadcopter(client,desired_roll,desired_pitch,desired_yaw,throttle)
    
    x,y = E100_functions.get_XY(client)
    x_pos.append(x)
    y_pos.append(y)
   
    #################### get sensor readings #####################
    if altitude_sensor_flag==1:
        altitude_n1=0
        altitude_sensor_flag=0
    if Lidar_sensor_flag == 1:
        front1, right1, left1, back1 = E100_functions.get_lidars(client)
               
    altitude_n0 = E100_functions.get_barometer(client) # read quadcopter's altitude WITH BAROMETER- ANDY
    altitude = alpha_alt*altitude_n0 + (1-alpha_alt)*altitude_n1  # apply low-pass filter
    altitude_n1 = altitude
    
    roll, pitch, yaw = E100_functions.get_orientation(client) # read quadcopter's attitude
    
    front, right, left, back = E100_functions.get_lidars(client)    # read LIDAR readings WITH THE UNDERSTANDING WE LATER REWRITE RIGHT AND LEFT
    
    
    left = initY;
    right = initY - 2*target_overall;
    
    front = alpha_lidar*front + (1-alpha_lidar)*front1
    right = alpha_lidar*right + (1-alpha_lidar)*right1
    left = alpha_lidar*left + (1-alpha_lidar)*left1
    
    
    
    front1 = front
    right1 = right
    left1 = left
    
    if alt_flag == 1:
        error_old = target_alt-altitude
        alt_flag = 0
    else:
        error_old = error    
    error= target_alt-altitude
    alt_integration_term += error*dt
    alt_differential_term = (error - error_old)/dt  
    throttle = K_P*error + K_I*alt_integration_term + K_D*alt_differential_term
    if throttle < 0:
        throttle = 0
    elif throttle > 1:
        throttle = 1       
    
    ##############################################################
    #front dist hold part(PID controller+activation function)####
    ##############################################################     
    if pitch_flag == 1:
        pitch_error_old = target_front_dist - front
        pitch_flag = 0
    else:
        pitch_error_old = pitch_error    
    pitch_error= target_front_dist - front
    pitch_integration_term += pitch_error*dt
    pitch_differential_term = (pitch_error - pitch_error_old)/dt
    desired_pitch = pitch_K_P*pitch_error + pitch_K_I*pitch_integration_term + pitch_K_D*pitch_differential_term
    desired_pitch = math.degrees(0.15*np.tanh(desired_pitch))  # use tanh function to limit the maximum and minimum of the pitch value
    
    ##############################################################
    #center dist hold part(PID controller+activation function)####
    ##############################################################    
    if roll_flag == 1:
        roll_error_old = right - left
        roll_flag = 0
    else:
        roll_error_old = roll_error    
    roll_error=  right - left
    roll_integration_term += roll_error*dt
    roll_differential_term = (roll_error - roll_error_old)/dt    
    desired_roll = roll_K_P*roll_error + roll_K_I*roll_integration_term + roll_K_D*roll_differential_term
    desired_roll = math.degrees(0.2*np.tanh(0.5*desired_roll))
    desired_yaw = 0
    
    lat_error.append(right - left)
    plt.plot(lat_error)


plt.scatter(x_pos,y_pos)"""

#### Copy and paste the following in your own flight controller #####    

############### release the link to AirSim ###########
client.armDisarm(False)
client.enableApiControl(False)






















