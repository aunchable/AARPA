# File that defines the world and its parameters, the robot and its
# parameters (lengths, motor strengths, etc), and the relations beween the
# robot and world


class World:
    def __init__(self):
        self.CANVAS_WIDTH =
        self.CANVAS_HEIGHT =
        self.ROBOT_BASE_X =
        self.ROBOT_BASE_Y =

class RobotArm:
    def __init__(self, w):
        self.current_joints = np.zeros(5)
        self.world = w
        self.L1 =
        self.L2 =
        self.L3 =
