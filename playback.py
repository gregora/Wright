import numpy as np
import pandas as pd

from math import cos, sin, pi
import matplotlib.pyplot as plt

from misc import *

import time

import tqdm

from Visualization import Visualization

import pygame

import sys

visualization = Visualization(record=False, fps=100)

file = "playbacks/playback1.csv"

if len(sys.argv) > 1:
    file = sys.argv[1]

data = pd.read_csv(file)

ground_height = 300  # change if needed
yaw_offset = -90     # change if needed

data["Time"] = data["Time"] - data["Time"][0]

data["Altitude"] = data["Altitude"] - ground_height
data["Latitude"]  = (data["Latitude"]  - data["Latitude"][0])  * 40_075 * 1000 / 360
data["Longitude"] = (data["Longitude"] - data["Longitude"][0]) * 40_075 * 1000 / 360

v_i = np.zeros((3, 1))
x = np.zeros((3, 1))

t = 0
dt = 0

Pk = np.eye(6) * 10

x_gps = None

while True:
    time_start = time.time()

    # interpolate current state
    yaw = np.interp(t, data["Time"], data["Yaw"])
    pitch = np.interp(t, data["Time"], data["Pitch"])
    roll = np.interp(t, data["Time"], data["Roll"])
                     
    latitude = np.interp(t, data["Time"], data["Latitude"])
    longitude = np.interp(t, data["Time"], data["Longitude"])
    altitude = np.interp(t, data["Time"], data["Altitude"])

    a_bx = np.interp(t, data["Time"], data["ax"])
    a_by = np.interp(t, data["Time"], data["ay"])
    a_bz = np.interp(t, data["Time"], data["az"])

    x_gps_prev = x_gps
    x_gps = np.array([[latitude], [longitude], [-altitude]]) # x points to north, y points to east, z points down

    eul_gyro = np.array([[yaw], [-pitch], [-roll]])
    eul = np.array([[yaw + yaw_offset], [pitch], [roll]])
    eul = np.array([[yaw + yaw_offset], [pitch], [roll]])

    R = ZYX2R(eul_gyro*pi/180)
    a_i = R @ np.array([[a_bx], [a_by], [a_bz]]) # body frame acceleration to inertial frame
    
    # axis remapping
    a_i[1, 0] = -a_i[1, 0] # y is inverted in bno055
    a_i[2, 0] = -a_i[2, 0] # z is inverted in bno055


    if x.all() == 0.0:
        x = x_gps
    
    #x = x_gps
    x = x + dt * v_i
    v_i = v_i + dt * a_i

    Fk = np.array([
        [1, 0, 0, dt,  0,  0],
        [0, 1, 0,  0, dt,  0],
        [0, 0, 1,  0,  0, dt],
        [0, 0, 0,  1,  0,  0],
        [0, 0, 0,  0,  1,  0],
        [0, 0, 0,  0,  0,  1] 
    ])

    Hk = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 1, 0, 0, 0]
    ])

    Qk = np.diag([100, 100, 100, 100, 100, 100])
    Rk = np.eye(3) * 10


    Pk = Fk @ Pk @ Fk.T + Qk

    if x_gps_prev is not None and x_gps_prev.all() != x_gps.all():
        yk = x_gps - x
        Sk = Hk @ Pk @ Hk.T + Rk
        Kk = Pk @ Hk.T @ np.linalg.inv(Sk)
        x = x + Kk[:3, :] @ yk
        v_i = v_i + Kk[3:, :] @ yk
        Pk = (np.eye(6) - Kk @ Hk) @ Pk

    #x = x_gps
    x_hist = x.copy()
    visualization.history.append(x_hist)
    visualization.update(t, x_hist, R2XYZ(ZYX2R(eul*pi/180))*180/pi)

    time_end = time.time()

    t += (time_end - time_start) * 1000
    dt = time_end - time_start # used for kalman filter