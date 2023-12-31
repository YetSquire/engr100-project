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

initY, initX = E100_functions.get_XY(client)
#initX, r, l, b = E100_functions.get_lidars(client);
lastZ = initZ = E100_functions.get_barometer(client)

currentX = initX
currentY = initY
currentZ = initZ

print(initX)
print(initY)
print(initZ)

K_PX = 2
K_IX = 0.2
K_DX = 2

K_PY = 1.5
K_IY = 0
K_DY = 1.5

K_PZ = 0.5
K_IZ = 0.1
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

alpha = 0.85

#############
windOn = True
startedLeft = True
buildingHeight = 40
buildingWidth = 20
windowHeight = 10
windowInterval = 5
done = False

y_log = []
z_log = []
wind_y_log = []
wind_z_log = []

x_log = []
wind_x_log = []





start = time.time()


#TODO: in order, plotting, finetune variable, stabilizeX, wind, stabilize after wind, motion path
#DIMENSIONS between Unreal and Airsim are about 1:1

#set building to loc: -1800, 5000, 2500, set size to 20, 20, 50
#drone to -3300, 4000, 1000
#LOCATION is based on cm

#Questions for next week: a good way to prevent more overshooting, completely stabilize height, and should we implement alpha control? Good way to show wind naturally??
def wait(begin):
    '''now = time.time()
    while now-begin < 3:
        now = time.time()
        E100_functions.set_quadcopter(client, 0, 0, 0, 0.6)'''
    #print("waited")

def stabilizeAll(errorOldX, integrationX, targetX, errorOldY, integrationY, targetY, errorOldZ, integrationZ, targetZ, lastZ):
    pitch, errorX = stabilizeX(errorOldX, integrationX, targetX)
    roll, errorY = stabilizeY(errorOldY, integrationY, targetY)
    throttle, errorZ, lastZ = stabilizeZ(errorOldZ, integrationZ, targetZ, lastZ)
    currentY, currentX = E100_functions.get_XY(client);
    
    #print(roll)
    #print(pitch)
    #print(throttle)
    #print(currentX)
    #print(currentY)
    #random gusts of wind
    
    if throttle > 1: throttle = 1
    if throttle < 0: throttle = 0
    E100_functions.set_quadcopter(client, roll, pitch, 0, throttle)
    x_log.append(currentX)
    y_log.append(currentY)
    return errorX, errorY, errorZ, lastZ

def stabilizeX(error_old, integration_term, target_dist):
    #currentX, r, l, b = E100_functions.get_lidars(client);
    currentY, currentX = E100_functions.get_XY(client);
    
    #difference between front and front1???
    #currentX = alpha_lidar*currentX + (1-alpha_lidar)*currentX
    error = currentX - target_dist
    #optional stop subroutine
    differential_term = (error - error_old) / dt

    pitch = K_PX * error + K_IX * integration_term + K_DX * differential_term
    #if np.isnan(pitch): pitch = 0
    if abs(error) < 0.1: pitch = 0.1
    #print(error)
    #if abs(error) > 20: pitch = 0
    #ask profs
    
    
    return pitch, error

def stabilizeY(error_old, integration_term, target_dist):
    currentY, currentX = E100_functions.get_XY(client);  # swapped because
    error = target_dist - currentY

    differential_term = (error - error_old) / dt

    roll = K_PY * error + K_IY * integration_term + K_DY * differential_term
    return roll, error 


def stabilizeZ(error_old, integration_term, target_dist, lastZ):
    currentZ = E100_functions.get_barometer(client);
    z_log.append(currentZ)
    currentZ = alpha*currentZ + (1-alpha)*lastZ
    lastZ = currentZ
    
    error = target_dist - currentZ

    differential_term = (error - error_old) / dt

    throttle = K_PZ * error + K_IZ * integration_term + K_DZ * differential_term
    if throttle >= 1:
        throttle = 1
    if throttle < 0:
        throttle = 0
    return throttle, error, lastZ

def wind():
    print("blew")
    currentY, currentX = E100_functions.get_XY(client);
    currentZ = E100_functions.get_barometer(client);
    wind_x_log.append(currentX)
    wind_y_log.append(currentY)
    wind_z_log.append(currentZ)
    E100_functions.set_wind(client,random.randrange(-10,10),random.randrange(-10,10),random.randrange(-10,10))
    
    

while True:
    now = time.time()
    
    targetX = initX
    
    #targetY and targetZ are relative to where the drone originally started- 0,0, in a sense
    zHold = initZ
    while lastZ < buildingHeight and not done:
        currentY, currentX = E100_functions.get_XY(client)  # swapped because
        zHold += windowHeight
        targetY = currentY
        targetZ = zHold
        iterZ = E100_functions.get_barometer(client) #the height at the beginning of the window climb
        
        while lastZ < iterZ + windowHeight - 3:
            if windOn and random.randrange(-20,10) > 8: wind()
            errorX, errorY, errorZ, lastZ = stabilizeAll(errorX, integrationX, targetX,
                                              errorY, integrationY, targetY,
                                              errorZ, integrationZ, targetZ,
                                              lastZ)
            integrationX += errorX*dt
            integrationY += errorY*dt
            integrationZ += errorZ*dt
        if startedLeft: yHold = 0
        if not startedLeft: yHold = buildingWidth
        if startedLeft:
           while currentY < buildingWidth - windowInterval:
               iterY, unused = E100_functions.get_XY(client)
               startedLeft = False
               sent = time.time()
               wait(sent)
               while currentY < iterY + windowInterval:
                   if windOn and random.randrange(-20,10) > 8: wind()
                   currentY, currentX = E100_functions.get_XY(client)
                   targetY = iterY + windowInterval
                   errorX, errorY, errorZ, lastZ = stabilizeAll(errorX, integrationX, targetX,
                                                     errorY, integrationY, targetY,
                                                     errorZ, integrationZ, targetZ,
                                                     lastZ)
                   integrationX += errorX*dt
                   integrationY += errorY*dt
                   integrationZ += errorZ*dt
               sent = time.time()
               wait(sent)
        else:
            while currentY > windowInterval:
                iterY, unused = E100_functions.get_XY(client)
                startedLeft = True
                sent = time.time()
                wait(sent)
                while currentY > iterY - windowInterval:
                    if windOn and random.randrange(-20,10) > 8: wind()
                    currentY, currentX = E100_functions.get_XY(client)
                    targetY = iterY - windowInterval
                    errorX, errorY, errorZ, lastZ = stabilizeAll(errorX, integrationX, targetX,
                                                      errorY, integrationY, targetY,
                                                      errorZ, integrationZ, targetZ,
                                                      lastZ)
                    integrationX += errorX*dt
                    integrationY += errorY*dt
                    integrationZ += errorZ*dt
                sent = time.time()
                wait(sent)
                    
        if buildingHeight - lastZ < 5: 
            done = True
            #plt.scatter(y_log, z_log)
            #plt.scatter(wind_y_log, wind_z_log)
            plt.scatter(x_log, z_log)
            plt.scatter(wind_x_log, wind_z_log)
                    
                    
                    #ok so currently, the drone flies above the building at the very end for some reason
                    #most of it looks ok before that, needs fine-tuning
                   
    #print("ended")
    errorX, errorY, errorZ, lastZ = stabilizeAll(errorX, integrationX, 0,
                                          errorY, integrationY, 0,
                                          errorZ, integrationZ, 0,
                                          lastZ)
    integrationX += errorX*dt
    integrationY += errorY*dt
    integrationZ += errorZ*dt
    
