import numpy as np
import pandas as pd

from math import cos, sin, pi
import matplotlib.pyplot as plt

from misc import *

import time

import tqdm

from Visualization import Visualization

import pygame

visualization = Visualization(record=False, fps=100)

data = pd.read_csv("playbacks/playback2.csv")

ground_height = 300

data["Time"] = data["Time"] - data["Time"][0]
data["Latitude"] = data["Latitude"] - data["Latitude"][0]
data["Longitude"] = data["Longitude"] - data["Longitude"][0]
data["Altitude"] = data["Altitude"] - ground_height

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
