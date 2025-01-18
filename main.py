import numpy as np
from math import cos, sin, pi
import matplotlib.pyplot as plt

from misc import *

import time

import tqdm

from Airframe import Airframe
from Visualization import Visualization

airframe = Airframe.from_json("rc.json")
visualization = Visualization(record=True, fps=100)

attitudes = []
positions = []

velocities = []


airframe.v_i[0, 0] = 10


airframe.attitude[0, 0] = 0.3
airframe.attitude[1, 0] = -0.06
airframe.attitude[2, 0] = 0.0



dt = 0.001

N = 60_000


for i in tqdm.tqdm(range(N)):

    airframe.physics(dt)


    attitudes.append(airframe.attitude.copy())
    positions.append(airframe.x_i.copy())
    velocities.append(airframe.v_i.copy())

    if i % 10 == 0:
        visualization.update(i*dt, attitudes[-1]*180/pi)
        time.sleep(0.005)

visualization.close()



attitudes = np.array(attitudes)
positions = np.array(positions)
velocities = np.array(velocities)

T = np.linspace(0, dt*N, N)

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

