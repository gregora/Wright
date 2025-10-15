import numpy as np
from math import cos, sin, pi

from misc import *

import json


class Airframe:
    x_i = np.zeros((3, 1)) # inertial position
    attitude = np.zeros((3, 1)) # attitude as euler angles (Roll - Pitch - Yaw)
    quaternion = np.zeros((4, 1)) # attitude as quaternion

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
            self.attitude = wrapToPi(self.attitude)

            R = XYZ2R(self.attitude)

            # THIS HAS TO BE R.T BECAUSE YOU NEED TO CALCULATE HOW VECTOR WOULD LOOK IN BODY FRAME
            # NOT HOW TO TRANSFORM THE VECTOR FROM INERTIAL TO BODY FRAME
            v_b = R.T @ self.v_i
            w_b = R.T @ self.w_i

            torque_b = np.array([[0.0], [0.0], [0.0]])
            force_b = np.array([[0.0], [0.0], [0.0]])


            for surface in self.surfaces.values():

                # calculate surface velocity
                v_s = np.cross(w_b[:, 0], surface["Position"][:, 0])
                v_s = np.array([[v_s[0]], [v_s[1]], [v_s[2]]])
                v_s = v_s + v_b

                # from surface velocity calculate local alpha and beta
                alpha_s = np.arctan2(v_s[2, 0], v_s[0, 0])
                beta_s = np.arctan2(v_s[1, 0], v_s[0, 0])

                if surface["Vertical"]:
                    alpha_s, beta_s = beta_s, alpha_s

                alpha_s += surface["Angle"]

                if surface["Type"] == "Symetric":
                    Cl, Cd = symetricC(alpha_s)
                elif surface["Type"] == "Positive":
                    Cl, Cd = positiveC(alpha_s)
                elif surface["Type"] == "Negative":
                    Cl, Cd = negativeC(alpha_s)

                if not surface["Vertical"]:
                    lift = 0.5 * 1.225 * (v_s[0, 0]**2 + v_s[2, 0]**2) * surface["Area"] * Cl
                    drag = 0.5 * 1.225 * (v_s[0, 0]**2 + v_s[2, 0]**2) * surface["Area"] * Cd

                    drag_vector = -np.array([[cos(alpha_s)], [0], [-sin(alpha_s)]]) * drag
                    lift_vector = -np.array([[sin(alpha_s)], [0], [cos(alpha_s)]]) * lift

                if surface["Vertical"]:
                    lift = 0.5 * 1.225 * (v_s[0, 0]**2 + v_s[1, 0]**2) * surface["Area"] * Cl
                    drag = 0.5 * 1.225 * (v_s[0, 0]**2 + v_s[1, 0]**2) * surface["Area"] * Cd

                    drag_vector = -np.array([[cos(alpha_s)], [-sin(alpha_s)], [0]]) * drag
                    lift_vector = -np.array([[sin(alpha_s)], [cos(alpha_s)], [0]]) * lift

            
                force_b += drag_vector + lift_vector

                torque_s = np.array([np.cross(surface["Position"][:, 0], lift_vector[:, 0] + drag_vector[:, 0])]).T
                
                torque_b += torque_s

                """
                print(f"Surface: {surface['Name']}")
                print(f"Alpha: {alpha_s}")
                print(f"Beta: {beta_s}")


                print(f"Lift: {lift}")
                print(f"Drag: {drag}")

                print(f"Drag Vector: {drag_vector}")
                print(f"Lift Vector: {lift_vector}")

                print(f"Torque: {torque_s}")

                print()
                """

            for motor in self.motors.values():
                force_b += motor["Thrust"] * np.array([[1], [0], [0]])
                torque_b += motor["Torque"] * np.array([[1], [0], [0]])

                torque_b += np.array([np.cross(motor["Position"][:, 0], motor["Thrust"] * np.array([[1], [0], [0]])[:, 0])]).T

            # force of gravity
            g_b = R.T @ self.g_i
            torque_b += np.array([np.cross(self.cm[:, 0], self.m * g_b[:, 0])]).T

            force_b += self.m * g_b


            force_i = R @ force_b
            torque_i = R @ torque_b

            I_c = R @ self.I @ R.T # first calculate where vector would be in body frame, multiply by I and transform back
            L_c = I_c @ self.w_i

            a_i = force_i / self.m

            # because center of body is not in the center of mass, we have to take into account the acceleration of the center of mass 
            torque_i = torque_i - np.array([np.cross(self.cm[:, 0], force_i[:, 0])]).T
            # calculate angular acceleration from torque 
            alpha_i = np.linalg.inv(I_c) @ (torque_i - np.array([np.cross(self.w_i[:, 0], L_c[:, 0])]).T)


            self.v_i += a_i * dt
            self.w_i += alpha_i * dt
            
            
            self.x_i += self.v_i * dt
            self.attitude += getdXYZ(self.attitude, self.w_i) * dt
            self.attitude = wrapToPi(self.attitude)

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