import numpy as np
from math import cos, sin, tan, pi, atan2, asin


def getR(attitude):
    # Returns the rotation matrix from the euler angles
    # Roll - Pitch - Yaw representation
    # Inertial frame -> Body frame

    roll = attitude[0, 0]
    pitch = attitude[1, 0]
    yaw = attitude[2, 0]

    R1 = np.array([[1, 0, 0],
                   [0, cos(roll), -sin(roll)],
                   [0, sin(roll), cos(roll)]])

    R2 = np.array([[cos(pitch), 0, sin(pitch)],
                    [0, 1, 0],
                    [-sin(pitch), 0, cos(pitch)]])

    R3 = np.array([[cos(yaw), -sin(yaw), 0],
                     [sin(yaw), cos(yaw), 0],
                     [0, 0, 1]])

    R = R1 @ R2 @ R3

    return R

def getdEul(attitude, w_b):
    # Returns the time derivative of the euler angles
    # Roll - Pitch - Yaw representation

    roll = attitude[0, 0]
    pitch = attitude[1, 0]
    yaw = attitude[2, 0]

    dEul = np.array([[1, sin(roll) * tan(pitch), cos(roll) * tan(pitch)],
                     [0, cos(roll), -sin(roll)],
                     [0, sin(roll) / cos(pitch), cos(roll) / cos(pitch)]]) @ w_b

    return dEul

def R2ZYX(R):
    # Returns the euler angles from the rotation matrix
    # Yaw - Pitch - Roll representation

    pitch = -asin(R[2, 0])

    if cos(pitch) != 0:
        roll = atan2(R[2, 1] / cos(pitch), R[2, 2] / cos(pitch))
        yaw = atan2(R[1, 0] / cos(pitch), R[0, 0] / cos(pitch))
    else:
        roll = 0
        yaw = atan2(R[1, 0], R[0, 0])

    return np.array([[roll], [pitch], [yaw]])


def symetricC(alpha):
    Cl = 0.5 * alpha / 5 * 180 / pi
    Cd = (alpha * 180 / pi)**2 * 0.00017 + 0.01

    return Cl, Cd

def positiveC(alpha):
    Cl = 0.2 + 0.5 * alpha / 5 * 180 / pi
    Cd = (alpha * 180 / pi)**2 * 0.00017 + 0.01

    return Cl, Cd

def negativeC(alpha):
    Cl = -0.2 + 0.5 * alpha / 5 * 180 / pi
    Cd = (alpha * 180 / pi)**2 * 0.00017 + 0.01

    return Cl, Cd

def wrapToPi(angle):
    return (angle + pi) % (2 * pi) - pi
