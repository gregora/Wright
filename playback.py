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

ground_height = 300

data["Time"] = data["Time"] - data["Time"][0]

data["Altitude"] = data["Altitude"] - ground_height
data["Latitude"]  = (data["Latitude"]  - data["Latitude"][0])  * 40_075 * 1000 / 360
data["Longitude"] = (data["Longitude"] - data["Longitude"][0]) * 40_075 * 1000 / 360

t = 0

while True:
    time_start = time.time()

    # interpolate current state
    yaw = np.interp(t, data["Time"], data["Yaw"])
    pitch = np.interp(t, data["Time"], data["Pitch"])
    roll = np.interp(t, data["Time"], data["Roll"])
                     
    latitude = np.interp(t, data["Time"], data["Latitude"])
    longitude = np.interp(t, data["Time"], data["Longitude"])
    altitude = np.interp(t, data["Time"], data["Altitude"])

    x = np.array([[latitude], [longitude], [-altitude]])
    eul = np.array([[yaw], [pitch], [roll]])


    visualization.update(t, x, R2XYZ(ZYX2R(eul*pi/180))*180/pi)

    time_end = time.time()

    t += (time_end - time_start) * 1000
