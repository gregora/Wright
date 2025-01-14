import numpy as np
from math import cos, sin, pi
import matplotlib.pyplot as plt

from misc import *

x_i = np.zeros((3, 1)) # inertial position
attitude = np.zeros((3, 1)) # attitude as euler angles (Roll - Pitch - Yaw)

v_i = np.zeros((3, 1)) # inertial velocities
w_i = np.zeros((3, 1)) # inertial angular velocities


g_i = np.array([[0], [0], [9.81]]) # gravity vector in inertial frame



v_i[0, 0] = 10
attitude[1, 0] = 0.05
w_i[0, 0] = 0


surfaces = [
    {
        "Name": "Left Wing",
        "Position": np.array([[0], [-0.5], [0]]),
        "Area": 0.05,
        "Type": "Positive",
        "Vertical": False
    },
    {
        "Name": "Right Wing",
        "Position": np.array([[0], [0.5], [0]]),
        "Area": 0.05,
        "Type": "Positive",
        "Vertical": False
    },
    {
        "Name": "Horizontal Stabilizer",
        "Position": np.array([[-0.5], [0], [0]]),
        "Area": 0.0685,
        "Type": "Negative",
        "Vertical": False
    },
    {
        "Name": "Vertical Stabilizer",
        "Position": np.array([[-0.5], [0], [0.2]]),
        "Area": 0.01,
        "Type": "Symetric",
        "Vertical": True
    }
]


masses = [
    {
        "Name": "Battery",
        "Position": np.array([[0.25], [0], [0]]),
        "Mass": 0.147
    },
    {
        "Name": "Motor",
        "Position": np.array([[0.35], [0], [0]]),
        "Mass": 0.064
    },
    {
        "Name": "ESC",
        "Position": np.array([[0.25], [0], [0]]),
        "Mass": 0.038
    },
    {
        "Name": "Flight Controller",
        "Position": np.array([[-0.30], [0], [0]]),
        "Mass": 0.07
    },
    {
        "Name": "Receiver",
        "Position": np.array([[0.25], [0], [0]]),
        "Mass": 0.02
    },
    {
        "Name": "Left Wing",
        "Position": np.array([[0], [-0.5], [0]]),
        "Mass": 0.2
    },
    {
        "Name": "Right Wing",
        "Position": np.array([[0], [0.5], [0]]),
        "Mass": 0.2
    },
    {
        "Name": "Horizontal Stabilizer",
        "Position": np.array([[-0.5], [0], [0]]),
        "Mass": 0.05
    },
    {
        "Name": "Vertical Stabilizer",
        "Position": np.array([[-0.5], [0], [0.2]]),
        "Mass": 0.05
    },
    {
        "Name": "Fuselage",
        "Position": np.array([[0], [0], [0]]),
        "Mass": 0.3
    },
    {
        "Name": "Nose",
        "Position": np.array([[0.5], [0], [0]]),
        "Mass": 0.15
    }
]


m = 0 # total mass
I = np.zeros((3, 3)) # total inertia matrix

cm = np.array([[0.0], [0.0], [0.0]])

for mass in masses:
    m += mass["Mass"]

    I += mass["Mass"] * np.array([[mass["Position"][1, 0]**2 + mass["Position"][2, 0]**2, -mass["Position"][0, 0] * mass["Position"][1, 0], -mass["Position"][0, 0] * mass["Position"][2, 0]],
                                  [-mass["Position"][0, 0] * mass["Position"][1, 0], mass["Position"][0, 0]**2 + mass["Position"][2, 0]**2, -mass["Position"][1, 0] * mass["Position"][2, 0]],
                                  [-mass["Position"][0, 0] * mass["Position"][2, 0], -mass["Position"][1, 0] * mass["Position"][2, 0], mass["Position"][0, 0]**2 + mass["Position"][1, 0]**2]])

    cm += mass["Position"] * mass["Mass"]

cm /= m



print(f"Total Mass: {m}")
print(f"Total Inertia: {I}")
print(f"Center of Mass: {cm}")


attitudes = []
positions = []

velocities = []



dt = 0.01

#exit()

for i in range(200):

    R = getR(attitude)

    v_b = R @ v_i
    w_b = R @ w_i

    torque_b = np.array([[0.0], [0.0], [0.0]])
    force_b = np.array([[0.0], [0.0], [0.0]])

    for surface in surfaces:

        # calculate surface velocity
        v_s = np.cross(w_b[:, 0], surface["Position"][:, 0])
        v_s = np.array([[v_s[0]], [v_s[1]], [v_s[2]]])
        v_s = - v_s + v_b

        # from surface velocity calculate local alpha and beta
        alpha_s = np.arctan2(v_s[2, 0], v_s[0, 0])
        beta_s = np.arctan2(v_s[1, 0], v_s[0, 0])

        if surface["Vertical"]:
            alpha_s, beta_s = beta_s, alpha_s

        if surface["Type"] == "Symetric":
            Cl, Cd = symetricC(-alpha_s)
        elif surface["Type"] == "Positive":
            Cl, Cd = positiveC(-alpha_s)
        elif surface["Type"] == "Negative":
            Cl, Cd = negativeC(-alpha_s)

        lift = 0.5 * 1.225 * v_s[0, 0]**2 * surface["Area"] * Cl
        drag = 0.5 * 1.225 * v_s[0, 0]**2 * surface["Area"] * Cd

        #alpha_s = -alpha_s
        drag_vector = -np.array([[cos(alpha_s)], [0], [-sin(alpha_s)]]) * drag
        lift_vector = -np.array([[sin(alpha_s)], [0], [cos(alpha_s)]]) * lift

        if surface["Vertical"]:
            drag_vector = -np.array([[cos(alpha_s)], [-sin(alpha_s)], [0]]) * drag
            lift_vector = -np.array([[sin(alpha_s)], [cos(alpha_s)], [0]]) * lift


        force_b += drag_vector + lift_vector

        torque_s = np.array([np.cross(surface["Position"][:, 0], lift_vector[:, 0] + drag_vector[:, 0])]).T
        
        torque_b += torque_s

        print(f"Surface: {surface['Name']}")
        print(f"Alpha: {alpha_s}")
        print(f"Beta: {beta_s}")


        print(f"Lift: {lift}")
        print(f"Drag: {drag}")

        print(f"Drag Vector: {drag_vector}")
        print(f"Lift Vector: {lift_vector}")

        print(f"Torque S: {torque_s}") 
        print()




    g_b = R @ g_i
    torque_b += np.array([np.cross(cm[:, 0], m * g_b[:, 0])]).T

    print(f"Gravity: {g_b}")
    print(f"Gravity Torque: {np.array([np.cross(cm[:, 0], m * g_b[:, 0])]).T}")

    a_b = force_b / m + g_b
    alpha_b = np.linalg.inv(I) @ torque_b


    v_b += a_b * dt
    w_b += alpha_b * dt

    v_i = R.T @ v_b
    w_i = R.T @ w_b
    
    x_i += v_i * dt
    attitude += getdEul(attitude, w_b) * dt

    
    print(f"Force B: {force_b}")
    print(f"Torque B: {torque_b}")

    print(f"Velocity: {v_i}")
    print(f"Angular Velocity: {w_i}")

    print(f"Position: {x_i}")
    print(f"Attitude: {attitude}")
    print()
    print()

    attitudes.append(attitude.copy())
    positions.append(x_i.copy())
    velocities.append(v_i.copy())

attitudes = np.array(attitudes)
positions = np.array(positions)
velocities = np.array(velocities)

plt.plot(attitudes[:, 0], label="Roll")
plt.plot(attitudes[:, 1], label="Pitch")
plt.plot(attitudes[:, 2], label="Yaw")

plt.legend()
plt.show()

plt.plot(positions[:, 0], label="X")
plt.plot(positions[:, 1], label="Y")
plt.plot(positions[:, 2], label="Z")

plt.legend()
plt.show()

plt.plot(velocities[:, 0], label="Vx")
plt.plot(velocities[:, 1], label="Vy")
plt.plot(velocities[:, 2], label="Vz")

plt.legend()

plt.show()




