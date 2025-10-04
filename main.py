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

#print(f"Mass          : {airframe.m:.2f} kg")
#print(f"Center of mass: {airframe.cm[0, 0]:.2f} m")

FPS = 30

visualization = Visualization(record=False, fps=FPS)

attitudes = []
positions = []

velocities = []

commands = []

airframe.x_i[2, 0] = -1.5

airframe.v_i[0, 0] = 10
airframe.v_i[1, 0] = 0
airframe.v_i[2, 0] = -1

airframe.attitude[0, 0] = 0.0
airframe.attitude[1, 0] = 0.2
airframe.attitude[2, 0] = 0

# turn off the motor
#airframe.motors["Motor"]["Thrust"] = 0
#airframe.motors["Motor"]["Torque"] = 0

dt = 0.005
T = 15 # seconds

N = int(T / dt)


for i in tqdm.tqdm(range(N)):

    airframe.physics(dt)

    if(np.isnan(airframe.attitude).any()):
        break

    if i % int(1 / (dt * FPS)) == 0:

        # add random wind
        # airframe.v_i += np.random.randn(3, 1) * 0.1


        positions.append(airframe.x_i.copy())
        velocities.append(airframe.v_i.copy())

        visualization.history.append(airframe.x_i.copy()[:,0])
        visualization.update(i*dt, airframe.x_i, airframe.attitude*180/pi)

        
        pygame.key.get_pressed()

        # left arrow
        if pygame.key.get_pressed()[pygame.K_LEFT]:
            airframe.surfaces["Left Aileron"]["Angle"] = -0.35 / 2
            airframe.surfaces["Right Aileron"]["Angle"] = 0.35 / 2
        # right arrow
        elif pygame.key.get_pressed()[pygame.K_RIGHT]:
            airframe.surfaces["Left Aileron"]["Angle"] = 0.35 / 2
            airframe.surfaces["Right Aileron"]["Angle"] = -0.35 / 2
        else:
           airframe.surfaces["Left Aileron"]["Angle"] = 0
           airframe.surfaces["Right Aileron"]["Angle"] = 0

        # up arrow
        if pygame.key.get_pressed()[pygame.K_UP]:
            airframe.surfaces["Elevator"]["Angle"] = 0.35 / 2
        elif pygame.key.get_pressed()[pygame.K_DOWN]:
            airframe.surfaces["Elevator"]["Angle"] = -0.35 / 2
        else:
            airframe.surfaces["Elevator"]["Angle"] = 0

        eul, w_b = airframe.sensor_data()

        eul = eul * 180 / pi

        eul_noise = np.random.normal(0, 2, size = (3, 1))   # angle nosise in deg
        w_b_noise = np.random.normal(0, 0.1, size = (3, 1)) # angular velocity noise in rad / s

        eul += eul_noise
        w_b += w_b_noise

        # control law imitation
        P = 4.0 / 90            * 0.35
        D = 0.1 / (4 * 3.14)    * 0.35

        airframe.surfaces["Elevator"]["Angle"]      =   (eul[1, 0] - 10) * P + w_b[1, 0]*D

        airframe.surfaces["Left Aileron"]["Angle"]  = - (eul[2, 0] - 0)*P - w_b[0,0]*D
        airframe.surfaces["Right Aileron"]["Angle"] =   (eul[2, 0] - 0)*P - w_b[0,0]*D

        if airframe.surfaces["Elevator"]["Angle"] > 0.35:
            airframe.surfaces["Elevator"]["Angle"] = 0.35
        elif airframe.surfaces["Elevator"]["Angle"] < -0.35:
            airframe.surfaces["Elevator"]["Angle"] = -0.35

        if airframe.surfaces["Left Aileron"]["Angle"] > 0.35:
            airframe.surfaces["Left Aileron"]["Angle"] = 0.35
        elif airframe.surfaces["Left Aileron"]["Angle"] < -0.35:
            airframe.surfaces["Left Aileron"]["Angle"] = -0.35

        if airframe.surfaces["Right Aileron"]["Angle"] > 0.35:
            airframe.surfaces["Right Aileron"]["Angle"] = 0.35
        elif airframe.surfaces["Right Aileron"]["Angle"] < -0.35:
            airframe.surfaces["Right Aileron"]["Angle"] = -0.35

        attitudes.append(eul)
        commands.append([airframe.surfaces["Left Aileron"]["Angle"], airframe.surfaces["Elevator"]["Angle"], airframe.surfaces["Rudder"]["Angle"]])

visualization.close()



attitudes = np.array(attitudes)
positions = np.array(positions)
velocities = np.array(velocities)
commands = np.array(commands)

T = np.linspace(0, dt*int(1 / (dt * FPS))*len(attitudes), len(attitudes))

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

plt.subplot(2, 2, 4)

plt.plot(T, commands[:, 0], label="Ailerons")
plt.plot(T, commands[:, 1], label="Elevator")
plt.plot(T, commands[:, 2], label="Rudder")

plt.legend()


plt.show()

