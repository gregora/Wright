import numpy as np
from math import cos, sin, tan, pi, atan2, asin


def XYZ2R(attitude):
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

def R2XYZ(R):
    # Returns the euler angles from the rotation matrix
    # Roll - Pitch - Yaw representation

    roll = atan2(R[2, 1], R[2, 2])
    pitch = -asin(R[2, 0])
    yaw = atan2(R[1, 0], R[0, 0])

    return np.array([[roll], [pitch], [yaw]])

def getdXYZ(attitude, w_b):
    # Returns the time derivative of the euler angles
    # Roll - Pitch - Yaw representation

    roll = attitude[0, 0]
    pitch = attitude[1, 0]
    yaw = attitude[2, 0]

    dEul = np.array([[1, sin(roll) * tan(pitch), cos(roll) * tan(pitch)],
                     [0, cos(roll), -sin(roll)],
                     [0, sin(roll) / cos(pitch), cos(roll) / cos(pitch)]]) @ w_b

    return dEul


def ZYX2R(zyx):
    # Returns the rotation matrix from the euler angles
    # Yaw - Pitch - Roll representation

    roll = zyx[2, 0]
    pitch = zyx[1, 0]
    yaw = zyx[0, 0]

    R1 = np.array([[cos(yaw), -sin(yaw), 0],
                     [sin(yaw), cos(yaw), 0],
                     [0, 0, 1]])
    
    R2 = np.array([[cos(pitch), 0, sin(pitch)],
                    [0, 1, 0],
                    [-sin(pitch), 0, cos(pitch)]])

    R3 = np.array([[1, 0, 0],
                   [0, cos(roll), -sin(roll)],
                   [0, sin(roll), cos(roll)]])

    R = R1 @ R2 @ R3

    return R

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

    return np.array([[yaw], [pitch], [roll]])


def normalizeQuat(q):
    # Normalizes a quaternion
    q = q / np.linalg.norm(q)

    return q


def quat2R(q):
    # Converts a quaternion to a rotation matrix
    w = q[0, 0]
    x = q[1, 0]
    y = q[2, 0]
    z = q[3, 0]

    R = np.array([[1 - 2 * (y**2 + z**2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                    [2 * (x * y + z * w), 1 - 2 * (x**2 + z**2), 2 * (y * z - x * w)],
                    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x**2 + y**2)]])

    return R

def R2quat(R):
    # Converts a rotation matrix to a quaternion
    q = np.zeros((4, 1))
    q[0, 0] = 0.5 * np.sqrt(1 + R[0, 0] + R[1, 1] + R[2, 2]) # w
    q[1, 0] = (R[2, 1] - R[1, 2]) / (4 * q[0, 0]) # x
    q[2, 0] = (R[0, 2] - R[2, 0]) / (4 * q[0, 0]) # y
    q[3, 0] = (R[1, 0] - R[0, 1]) / (4 * q[0, 0]) # z

    return normalizeQuat(q)


def dQuat(q, w_b, dt):
    # Returns the time derivative of a quaternion given the angular velocity in body frame

    Omega = np.array([[0, -w_b[0, 0], -w_b[1, 0], -w_b[2, 0]],
                      [w_b[0, 0], 0, w_b[2, 0], -w_b[1, 0]],
                      [w_b[1, 0], -w_b[2, 0], 0, w_b[0, 0]],
                      [w_b[2, 0], w_b[1, 0], -w_b[0, 0], 0]])
    dq = dt * 0.5 * Omega @ q

    return dq


def parsePolarTxt(filename):
    # Reads a txt and returns a numpy array


    f = open(filename, 'r')

    data = f.readlines()

    data = data[12:]

    array = []

    for line in data:
        line = line.strip()
        line = line.split(" ")

        line = [float(i) for i in line if i != '']

        array.append(line)

    array = np.array(array)

    f.close()

    return array


NACA0006 = parsePolarTxt('Airfoils/xf-naca0006-il-1000000.txt')
NACA2415 = parsePolarTxt('Airfoils/xf-naca2415-il-1000000.txt')

# NACA 0006 airfoil
def symetricC(alpha):
    # Small angle approximation
    #Cl = 0.5 * alpha / 5 * 180 / pi
    #Cd = (alpha * 180 / pi)**2 * 0.00017 + 0.01


    # Interpolation
    alpha = alpha * 180 / pi

    Cl = np.interp(alpha, NACA0006[:, 0], NACA0006[:, 1])
    Cd = np.interp(alpha, NACA0006[:, 0], NACA0006[:, 2])

    # optimistic linear region
    #if abs(alpha) > 10:
    #    Cl = alpha * 0.07
    #    Cd = abs(alpha) * 0.012

    return Cl, Cd

# NACA 2415 airfoil
def positiveC(alpha):
    # Small angle approximation
    #Cl = 0.2 + 0.5 * alpha / 5 * 180 / pi
    #Cd = (alpha * 180 / pi)**2 * 0.00017 + 0.01


    # Interpolation
    alpha = alpha * 180 / pi
    Cl = np.interp(alpha, NACA2415[:, 0], NACA2415[:, 1])
    Cd = np.interp(alpha, NACA2415[:, 0], NACA2415[:, 2])

    return Cl, Cd

# NACA 2415 airfoil, turned upside down
def negativeC(alpha):
    # Small angle approximation
    #Cl = -0.2 + 0.5 * alpha / 5 * 180 / pi
    #Cd = (alpha * 180 / pi)**2 * 0.00017 + 0.01

    # Interpolation
    alpha = alpha * 180 / pi

    Cl = -np.interp(-alpha, NACA2415[:, 0], NACA2415[:, 1])
    Cd = -np.interp(-alpha, NACA2415[:, 0], NACA2415[:, 2])

    return Cl, Cd


def wrapToPi(angle):
    return (angle + pi) % (2 * pi) - pi
