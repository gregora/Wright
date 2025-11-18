import numpy as np
from math import cos, sin, pi

from misc import *

import json


class Airframe:

    # using NED coordinate system
    # x - north
    # y - east
    # z - down

    x_i = np.zeros((3, 1)) # inertial position
    attitude = np.zeros((3, 1)) # attitude as euler angles (Roll - Pitch - Yaw)
    quaternion = np.zeros((4, 1)) # attitude as quaternion representation (w x y z)
    quaternion[0, 0] = 1.0

    v_i = np.zeros((3, 1)) # inertial velocities
    w_i = np.zeros((3, 1)) # inertial angular velocities

    g_i = np.array([[0], [0], [9.81]]) # gravity vector in inertial frame

    surfaces = [] # list of surfaces
    masses = [] # list of point masses
    motors = [] # list of motors

    m = 0 # total mass
    I = np.zeros((3, 3)) # total inertia matrix in body frame

    cm = np.array([[0.0], [0.0], [0.0]]) # center of mass


    def __init__(self, surfaces = [], masses = [], motors = []):
        self.surfaces = surfaces
        self.masses = masses
        self.motors = motors

        self.compute_fixed_values()

    def compute_fixed_values(self): # compute mass, inertia matrix and center of mass

        self.m = 0
        self.I = np.zeros((3, 3))

        self.cm = np.array([[0.0], [0.0], [0.0]])

        for mass in self.masses.values():
            self.m += mass["Mass"]

            self.I += mass["Mass"] * np.array([[mass["Position"][1, 0]**2 + mass["Position"][2, 0]**2, -mass["Position"][0, 0] * mass["Position"][1, 0], -mass["Position"][0, 0] * mass["Position"][2, 0]],
                                        [-mass["Position"][0, 0] * mass["Position"][1, 0], mass["Position"][0, 0]**2 + mass["Position"][2, 0]**2, -mass["Position"][1, 0] * mass["Position"][2, 0]],
                                        [-mass["Position"][0, 0] * mass["Position"][2, 0], -mass["Position"][1, 0] * mass["Position"][2, 0], mass["Position"][0, 0]**2 + mass["Position"][1, 0]**2]])

            self.cm += mass["Position"] * mass["Mass"]

        self.cm /= self.m

    def physics(self, dt):

        R = quat2R(self.quaternion)

        # THIS HAS TO BE R.T BECAUSE YOU NEED TO CALCULATE HOW VECTOR WOULD LOOK IN BODY FRAME
        # NOT HOW TO TRANSFORM THE VECTOR FROM INERTIAL TO BODY FRAME
        v_b = R.T @ self.v_i
        w_b = R.T @ self.w_i

        torque_b = np.array([[0.0], [0.0], [0.0]])
        force_b = np.array([[0.0], [0.0], [0.0]])


        for surface_name, surface in self.surfaces.items():

            # calculate surface velocity
            v_s = np.cross(w_b[:, 0], surface["Position"][:, 0] - self.cm[:, 0])
            v_s = np.array([[v_s[0]], [v_s[1]], [v_s[2]]])
            v_s = v_s + v_b

            # from surface velocity calculate local alpha and beta
            alpha_s = np.arctan2(v_s[2, 0], v_s[0, 0])
            beta_s =  np.arctan2(v_s[1, 0], v_s[0, 0])

            if surface["Vertical"]:
                alpha_s, beta_s = beta_s, alpha_s

            if "Angle" not in surface.keys():
                surface["Angle"] = 0.0

            alpha_s += surface["Angle"]

            if surface["Type"] == "Symetric":
                Cl, Cd = symetricC(alpha_s)
            elif surface["Type"] == "Positive":
                Cl, Cd = positiveC(alpha_s)
            elif surface["Type"] == "Negative":
                Cl, Cd = negativeC(alpha_s)

            drag_vector = np.zeros((3, 1))
            lift_vector = np.zeros((3, 1))

            if not surface["Vertical"]:
                lift = 0.5 * 1.225 * (v_s[0, 0]**2 + v_s[2, 0]**2) * surface["Area"] * Cl
                drag = 0.5 * 1.225 * (v_s[0, 0]**2 + v_s[2, 0]**2) * surface["Area"] * Cd

                drag_vector = -np.array([[ cos(alpha_s)], [0], [-sin(alpha_s)]]) * drag
                lift_vector = -np.array([[ sin(alpha_s)], [0], [ cos(alpha_s)]]) * lift


            if surface["Vertical"]:
                lift = 0.5 * 1.225 * (v_s[0, 0]**2 + v_s[1, 0]**2) * surface["Area"] * Cl
                drag = 0.5 * 1.225 * (v_s[0, 0]**2 + v_s[1, 0]**2) * surface["Area"] * Cd

                drag_vector = -np.array([[ cos(alpha_s)], [-sin(alpha_s)], [0]]) * drag
                lift_vector = -np.array([[ sin(alpha_s)], [ cos(alpha_s)], [0]]) * lift

            #drag_vector = np.zeros((3, 1))
            #lift_vector = np.zeros((3, 1))

            force_b += drag_vector + lift_vector

            torque_s = np.array([np.cross(surface["Position"][:, 0] - self.cm[:, 0], lift_vector[:, 0] + drag_vector[:, 0])]).T
            
            torque_b += torque_s

            
            print(f"Surface: {surface_name}")
            print(f"Global velocity: {self.v_i.T}")
            print(f"Local veloctiy: {v_b.T}")
            print()

            print(f"Alpha: {alpha_s * 180 / pi:.2f} deg")
            print(f"w_i: {self.w_i.T}")


            print(f"Lift: {lift}")
            print(f"Drag: {drag}")

            print(f"Drag Vector: {drag_vector.T}")
            print(f"Lift Vector: {lift_vector.T}")

            print(f"Torque: {torque_s.T}")

            print()
            

        for motor in self.motors.values():
            force_b += motor["Thrust"] * np.array([[1], [0], [0]])
            torque_b += motor["Torque"] * np.array([[1], [0], [0]])

        #exit()
        # force of gravity
        g_b = R.T @ self.g_i
        force_b += self.m * g_b

        w_b += np.linalg.inv(self.I) @ (torque_b - np.cross(w_b[:, 0], (self.I @ w_b)[:, 0]).reshape((3, 1))) * dt

        force_i = R @ force_b


        a_i = force_i / self.m

        
        #print(f"Angular acceleration in inertial frame: {alpha_i.T}")

        self.v_i += a_i * dt        
        self.w_i = R @ w_b
        
        print(w_b)

        self.x_i += self.v_i * dt
        self.attitude = R2ZYX(R)
        self.attitude = wrapToPi(self.attitude)

        self.quaternion += dQuat(self.quaternion, w_b, dt)
        self.quaternion = normalizeQuat(self.quaternion)

        """
        print(f"Gravity: {g_b}")
        print(f"Gravity Torque: {np.array([np.cross(cm[:, 0], m * g_b[:, 0])]).T}")
        
        print(f"Force i: {force_i}")
        print(f"Torque i: {torque_i}")
        print()
        print()
        """

    def sensor_data(self):
        R = XYZ2R(self.attitude)

        w_b = R.T @ self.w_i.copy()

        return R2ZYX(R).copy(), w_b.copy()

    def from_json(file):
        with open(file, 'r') as f:
            data = json.load(f)

        
        surfaces = data["Surfaces"]
        masses = data["Masses"]
        motors = data["Motors"]

        for s in surfaces.values():
            s["Position"] = np.array(s["Position"])
        
        for m in masses.values():
            m["Position"] = np.array(m["Position"])

        for m in motors.values():
            m["Position"] = np.array(m["Position"])

        

        return Airframe(surfaces, masses, motors)