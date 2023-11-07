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
print(" ")
print(initY)
print(" ")
print(initZ)
print(" ")

K_PX = 2
K_IX = 0.4
K_DX = 1.5

K_PY = 1.5
K_IY = 0.4
K_DY = 1.5

K_PZ = 2
K_IZ = 0.6
K_DZ = 0.5
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
alpha_lidar = 1.0
start = time.time()


#TODO: in order, plotting, finetune variable, stabilizeX, wind, stabilize after wind, motion path


def stabilizeAll(errorOldX, integrationX, targetX, errorOldY, integrationY, targetY, errorOldZ, integrationZ, targetZ):
    errorX = 0
    roll, errorY = stabilizeY(errorOldY, integrationY, targetY)
    throttle, errorZ = stabilizeZ(errorOldZ, integrationZ, targetZ)
    # future stabilizeX

    E100_functions.set_quadcopter(client, roll, 0, 0, throttle)
    return errorX, errorY, errorZ


def stabilizeY(error_old, integration_term, target_dist):
    currentY, currentX = E100_functions.get_XY(client);  # swapped because
    # possibly change to a variable later
    error = target_dist - currentY

    differential_term = (error - error_old) / dt

    roll = K_PY * error + K_IY * integration_term + K_DY * differential_term

    roll *= 2
    return roll, error


def stabilizeZ(error_old, integration_term, target_dist):
    currentZ = E100_functions.get_barometer(client);
    error = target_dist - currentZ

    differential_term = (error - error_old) / dt

    throttle = K_PZ * error + K_IZ * integration_term + K_DZ * differential_term

    return throttle, error


while True:
    now = time.time()
    currentY, currentX = E100_functions.get_XY(client)  # swapped because
    currentZ = E100_functions.get_barometer(client)
    yVel, xVel, zVel = E100_functions.get_linear_velocity(client)
    
    targetY = -10
    targetZ = initZ
    
    errorX, errorY, errorZ = stabilizeAll(errorX, integrationX, targetX,
                                          errorY, integrationY, targetY,
                                          errorZ, integrationZ, targetZ)
    integrationX += errorX*dt
    integrationY += errorY*dt
    integrationZ += errorZ*dt