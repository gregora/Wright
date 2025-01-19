import numpy as np
from math import cos, sin, pi
import matplotlib.pyplot as plt

from misc import *

import time

import tqdm

from Airframe import Airframe
from Visualization import Visualization

import pygame

airframe = Airframe.from_json("rc.json")
visualization = Visualization(record=True, fps=100)

attitudes = []
positions = []

velocities = []

airframe.x_i[2, 0] = -1.5

airframe.v_i[0, 0] = 10
airframe.v_i[1, 0] = 0
airframe.v_i[2, 0] = -1

#airframe.attitude[0, 0] = 0.3
airframe.attitude[1, 0] = 0.2
airframe.attitude[2, 0] = 0.0

# turn off the motor
#airframe.motors[0]["Thrust"] = 0
#airframe.motors[0]["Torque"] = 0

dt = 0.0005

N = 120_000


for i in tqdm.tqdm(range(N)):

    airframe.physics(dt)

    if(np.isnan(airframe.attitude).any()):
        break

    if i % 20 == 0:

        attitudes.append(airframe.attitude.copy())
        positions.append(airframe.x_i.copy())
        velocities.append(airframe.v_i.copy())


        eul, w_b = airframe.sensor_data()

        eul = eul * 180 / pi

        # control law imitation
        P = 0.35 / 90
        D = 0.01 / (4 * 3.14)

        airframe.surfaces[7]["Angle"] = (eul[1, 0] - 10) * P + w_b[1, 0]*D

        airframe.surfaces[5]["Angle"] = - (eul[0,0] - 0)*P - w_b[0,0]*D
        airframe.surfaces[6]["Angle"] =   (eul[0,0] - 0)*P - w_b[0,0]*D




        visualization.update(i*dt, attitudes[-1]*180/pi)

        

        pygame.key.get_pressed()

        # left arrow
        if pygame.key.get_pressed()[pygame.K_LEFT]:
            airframe.surfaces[5]["Angle"] = -0.35
            airframe.surfaces[6]["Angle"] = 0.35
        # right arrow
        elif pygame.key.get_pressed()[pygame.K_RIGHT]:
            airframe.surfaces[5]["Angle"] = 0.35
            airframe.surfaces[6]["Angle"] = -0.35
        #else:
        #   airframe.surfaces[5]["Angle"] = 0
        #   airframe.surfaces[6]["Angle"] = 0

        # up arrow
        if pygame.key.get_pressed()[pygame.K_UP]:
            airframe.surfaces[7]["Angle"] = 0.35
        elif pygame.key.get_pressed()[pygame.K_DOWN]:
            airframe.surfaces[7]["Angle"] = -0.35
        #else:
        #    airframe.surfaces[7]["Angle"] = 0

visualization.close()



attitudes = np.array(attitudes)
positions = np.array(positions)
velocities = np.array(velocities)

T = np.linspace(0, dt*50*len(attitudes), len(attitudes))

plt.subplot(2, 2, 1)

plt.plot(T, attitudes[:, 0], label="Roll")
plt.plot(T, attitudes[:, 1], label="Pitch")
plt.plot(T, attitudes[:, 2], label="Yaw")

plt.legend()

plt.subplot(2, 2, 2)

plt.plot(T, positions[:, 0], label="X")
plt.plot(T, positions[:, 1], label="Y")
plt.plot(T, positions[:, 2], label="Z")

plt.legend()

plt.subplot(2, 2, 3)

plt.plot(T, velocities[:, 0], label="Vx")
plt.plot(T, velocities[:, 1], label="Vy")
plt.plot(T, velocities[:, 2], label="Vz")

plt.legend()

plt.show()

