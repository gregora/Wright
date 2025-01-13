import numpy as np
from math import cos, sin, pi
import matplotlib.pyplot as plt

def getR(attitude):
    # Returns the rotation matrix from the euler angles

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

def symetricC(alpha):
    Cl = alpha / 10 * 180 / pi
    Cd = (alpha * 180 / pi)**2 * 0.00017 + 0.01

    return Cl, Cd

def positiveC(alpha):
    Cl = 0.2 + alpha / 10 * 180 / pi
    Cd = (alpha * 180 / pi)**2 * 0.00017 + 0.01

    return Cl, Cd

def negativeC(alpha):
    Cl = -0.2 - alpha / 10 * 180 / pi
    Cd = (alpha * 180 / pi)**2 * 0.00017 + 0.01

    return Cl, Cd


x_i = np.zeros((3, 1)) # inertial position
attitude = np.zeros((3, 1)) # attitude as euler angles (Roll - Pitch - Yaw)

v_i = np.zeros((3, 1)) # inertial velocities
w_i = np.zeros((3, 1)) # inertial angular velocities


dt = 0.01

v_i[0, 0] = 10
attitude[1, 0] = 0
w_i[0, 0] = -0.0


surfaces = [
    {
        "Name": "Left Wing",
        "Position": np.array([[0], [-0.5], [0]]),
        "Area": 0.5,
        "Type": "Positive",
        "Vertical": False
    },
    {
        "Name": "Right Wing",
        "Position": np.array([[0], [0.5], [0]]),
        "Area": 0.5,
        "Type": "Positive",
        "Vertical": False
    },
    {
        "Name": "Horizontal Stabilizer",
        "Position": np.array([[-0.5], [0], [0]]),
        "Area": 0.1,
        "Type": "Negative",
        "Vertical": False
    },
    {
        "Name": "Vertical Stabilizer",
        "Position": np.array([[-0.5], [0], [0.2]]),
        "Area": 0.1,
        "Type": "Symetric",
        "Vertical": True
    }
]



for i in range(10):

    R = getR(attitude)

    v_b = R.T @ v_i
    w_b = R.T @ w_i


    torque = np.array([[0], [0], [0]])
    force = np.array([[0], [0], [0]])

    for surface in surfaces:

        # calculate surface velocity
        v_s = np.cross(w_b[:, 0], surface["Position"][:, 0])
        v_s = np.array([[v_s[0]], [v_s[1]], [v_s[2]]])
        v_s = - v_s + v_b

        # from surface velocity calculate local alpha and beta
        alpha_s = -np.arctan2(v_s[2, 0], v_s[0, 0])
        beta_s = np.arctan2(v_s[1, 0], v_s[0, 0])

        print(f"Surface: {surface['Name']}")
        print(f"Alpha: {alpha_s}")
        print(f"Beta: {beta_s}")

        if surface["Vertical"]:
            alpha_s, beta_s = beta_s, alpha_s

        if surface["Type"] == "Symetric":
            Cl, Cd = symetricC(alpha_s)
        elif surface["Type"] == "Positive":
            Cl, Cd = positiveC(alpha_s)
        elif surface["Type"] == "Negative":
            Cl, Cd = negativeC(alpha_s)

        lift = 0.5 * 1.225 * v_s[0, 0]**2 * surface["Area"] * Cl
        drag = 0.5 * 1.225 * v_s[0, 0]**2 * surface["Area"] * Cd

        print(f"Lift: {lift}")
        print(f"Drag: {drag}")
 



    break