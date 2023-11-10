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

# import setup_path
import time
import random
import numpy as np
import math
import matplotlib.pyplot as plt

############### establish the link to AirSim ###########

import airsim  # import AirSim API
import E100_functions  # import drone simulator library

dt = E100_functions.dt()
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

#### Copy and paste the above in your own flight controller #####

initX, initY = E100_functions.get_XY(client)
initZ = E100_functions.get_barometer(client)

currentX = initX
currentY = initY
currentZ = initZ

print(initX)
print(initY)
print(initZ)

K_PX = 1
K_IX = 0.2
K_DX = 1

K_PY = 1.5
K_IY = 0
K_DY = 1.5

K_PZ = 2
K_IZ = 0.2
K_DZ = 1
#############

errorX = 0
errorY = 0
errorZ = 0

integrationX = 0
integrationY = 0
integrationZ = 0

targetX = 0
targetY = 0
targetZ = 0

alt_log = []

# throttle keeps z constant
# pitch is from building (existing wind concept)
# roll is

alpha_alt = 0.7
alpha_lidar = 0.6
start = time.time()


#TODO: in order, plotting, finetune variable, stabilizeX, wind, stabilize after wind, motion path
#DIMENSIONS between Unreal and Airsim are about 1:1

#set building to loc: -1800, 5000, 850, set size to 20, 20, 50
#drone to -3000, 5000, z doesn't matter so long as < 10000
#LOCATION is based on cm

#Questions for next week: a good way to prevent more overshooting, completely stabilize height, and should we implement alpha control? Good way to show wind naturally??

def stabilizeAll(errorOldX, integrationX, targetX, errorOldY, integrationY, targetY, errorOldZ, integrationZ, targetZ):
    pitch, errorX = stabilizeX(errorOldX, integrationX, targetX)
    roll, errorY = stabilizeY(errorOldY, integrationY, targetY)
    throttle, errorZ = stabilizeZ(errorOldZ, integrationZ, targetZ)
    
    print(roll)
    print(pitch)
    print(throttle)
    
    #random gusts of wind
    #if random.randrange(-10,10) > 8: wind()
    
    if throttle > 1: throttle = 1
    if throttle < 0: throttle = 0
    E100_functions.set_quadcopter(client, roll, pitch, 0, throttle)
    return errorX, errorY, errorZ

def stabilizeX(error_old, integration_term, target_dist):
    currentX, r, l, b = E100_functions.get_lidars(client);
    
    #difference between front and front1???
    #currentX = alpha_lidar*currentX + (1-alpha_lidar)*currentX
    error = target_dist - currentX
    if abs(error) > 20: error = 0
    #optional stop subroutine
    differential_term = (error - error_old) / dt

    pitch = K_PX * error + K_IX * integration_term + K_DX * differential_term
    if np.isnan(pitch): pitch = 0
    return pitch, error

def stabilizeY(error_old, integration_term, target_dist):
    currentY, currentX = E100_functions.get_XY(client);  # swapped because
    error = target_dist - currentY

    differential_term = (error - error_old) / dt

    roll = K_PY * error + K_IY * integration_term + K_DY * differential_term
    return roll, error 


def stabilizeZ(error_old, integration_term, target_dist):
    currentZ = E100_functions.get_barometer(client);
    error = target_dist - currentZ

    differential_term = (error - error_old) / dt

    throttle = K_PZ * error + K_IZ * integration_term + K_DZ * differential_term
    return throttle, error

def wind():
    print("blew")
    E100_functions.set_wind(client,random.randrange(-10,10),random.randrange(-10,10),random.randrange(-10,10))

while True:
    now = time.time()
    currentY, currentX = E100_functions.get_XY(client)  # swapped because
    currentZ = E100_functions.get_barometer(client)
    yVel, xVel, zVel = E100_functions.get_linear_velocity(client)
    
    targetX = 2
    targetY = 0
    targetZ = initZ + 50
    
    errorX, errorY, errorZ = stabilizeAll(errorX, integrationX, targetX,
                                          errorY, integrationY, targetY,
                                          errorZ, integrationZ, targetZ)
    integrationX += errorX*dt
    integrationY += errorY*dt
    integrationZ += errorZ*dt