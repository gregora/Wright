import numpy as np
from math import cos, sin, pi
import matplotlib.pyplot as plt

from misc import *

import tqdm

from Airframe import Airframe



surfaces = [
    {
        "Name": "Left Wing",
        "Position": np.array([[0], [-0.5], [0]]),
        "Area": 0.0576,
        "Type": "Positive",
        "Vertical": False
    },
    {
        "Name": "Right Wing",
        "Position": np.array([[0], [0.5], [0]]),
        "Area": 0.0576,
        "Type": "Positive",
        "Vertical": False
    },
    {
        "Name": "Horizontal Stabilizer",
        "Position": np.array([[-0.5], [0], [0]]),
        "Area": 0.03,
        "Type": "Negative",
        "Vertical": False
    },
    {
        "Name": "Vertical Stabilizer",
        "Position": np.array([[-0.5], [0], [-0.2]]),
        "Area": 0.0075,
        "Type": "Symetric",
        "Vertical": True
    }
]


masses = [
    {
        "Name": "Battery",
        "Position": np.array([[0.15], [0], [0.03]]),
        "Mass": 0.147
    },
    {
        "Name": "Motor",
        "Position": np.array([[0.35], [0], [-0.02]]),
        "Mass": 0.064
    },
    {
        "Name": "ESC",
        "Position": np.array([[0.25], [0], [-0.04]]),
        "Mass": 0.038
    },
    {
        "Name": "Flight Controller",
        "Position": np.array([[-0.30], [0], [0]]),
        "Mass": 0.07
    },
    {
        "Name": "Receiver",
        "Position": np.array([[0.0], [0], [0]]),
        "Mass": 0.02
    },
    {
        "Name": "Left Wing",
        "Position": np.array([[0], [-0.5], [0]]),
        "Mass": 0.01
    },
    {
        "Name": "Right Wing",
        "Position": np.array([[0], [0.5], [0]]),
        "Mass": 0.01
    },
    {
        "Name": "Horizontal Stabilizer",
        "Position": np.array([[-0.5], [0], [0]]),
        "Mass": 0.03
    },
    {
        "Name": "Vertical Stabilizer",
        "Position": np.array([[-0.5], [0], [-0.2]]),
        "Mass": 0.03
    },
    {
        "Name": "Fuselage",
        "Position": np.array([[0], [0], [0.1]]),
        "Mass": 0.1
    },
    {
        "Name": "Nose",
        "Position": np.array([[0.5], [0], [0]]),
        "Mass": 0.000
    }
]


airframe = Airframe(surfaces, masses)


attitudes = []
positions = []

velocities = []


airframe.v_i[0, 0] = 10


airframe.attitude[0, 0] = 0.0
airframe.attitude[1, 0] = -0.06
airframe.attitude[2, 0] = 0.1


dt = 0.001

N = 60_000

for i in tqdm.tqdm(range(N)):

    airframe.physics(dt)


    attitudes.append(airframe.attitude.copy())
    positions.append(airframe.x_i.copy())
    velocities.append(airframe.v_i.copy())

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
