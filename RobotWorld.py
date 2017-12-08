# File that defines the world and its parameters, the robot and its
# parameters (lengths, motor strengths, etc), and the relations beween the
# robot and world

import numpy as np

class World:
    def __init__(self):
        # Width and height of canvas in centimeters
        self.CANVAS_WIDTH = 60.0
        self.CANVAS_HEIGHT = 45.0

        # Base of robot with respect to bottom left corner of canvas
        self.ROBOT_BASE_X = 30.0
        self.ROBOT_BASE_Y = -10.0

class RobotArm:
    def __init__(self, w):
        self.current_theta = np.zeros(5)
        self.world = w
        # Length of first link
        self.L1 = 40.0
        # Length of second link
        self.L2 = 30.0
        # Distance along +z between tool origin and joint 5 axis (along +y)
        self.L3 = 5.0
        # Height from ground to tool origin at reference frame
        self.H = 10.0
        # End 1 of brush with respect to tool frame
        self.QB1 = np.asarray([0.0, 5.0, 0.0, 1.0])
        # End 2 of brush with respect to tool frame
        self.QB2 = np.asarray([0.0, -5.0, 0.0, 1.0])

    def get_current_theta(self):
        return self.current_joints

    def set_current_theta(self, theta):
        self.current_theta = theta
