import numpy as np
from math import cos, sin, pi
import matplotlib.pyplot as plt

from vedo import *
from misc import *

import tqdm

from Airframe import Airframe

airframe = Airframe.from_json("rc.json")

attitudes = []
positions = []

velocities = []


airframe.v_i[0, 0] = 10


airframe.attitude[0, 0] = 0.1
airframe.attitude[1, 0] = -0.06
airframe.attitude[2, 0] = 0.0

mesh = Mesh("CAD/plane.obj")
vedo_plt = Plotter(bg='beige', bg2='lb', axes=0, offscreen=True, interactive=1)
# show axes

vedo_plt += mesh
vedo_plt += __doc__

video = Video("video.mp4", duration=60, backend='ffmpeg') # backend='opencv'

cam1= dict(pos=(5.805, 17.34, -0.8418),
           focalPoint=(3.133, 1.506, -3.132),
           viewup=(-0.3099, 0.1871, -0.9322),
           distance=16.22,
           clippingRange=(12.35, 21.13))


dt = 0.001

N = 60_000

for i in tqdm.tqdm(range(N)):

    airframe.physics(dt)


    attitudes.append(airframe.attitude.copy())
    positions.append(airframe.x_i.copy())
    velocities.append(airframe.v_i.copy())

    if i % 40 == 0:
        mesh.rotate_x(airframe.attitude[0, 0] * 180 / pi)
        mesh.rotate_z(airframe.attitude[1, 0] * 180 / pi)
        mesh.rotate_y(airframe.attitude[2, 0] * 180 / pi)

        vedo_plt.show()
        video.add_frame()

        mesh.rotate_y(-airframe.attitude[2, 0] * 180 / pi)
        mesh.rotate_z(-airframe.attitude[1, 0] * 180 / pi)
        mesh.rotate_x(-airframe.attitude[0, 0] * 180 / pi)


#video.action(cameras=[cam1])
video.close()



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

