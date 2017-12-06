# This file is the main simulator which has a RobotWorld, a
# SimulatedTimelineGenerator to generate random timelines for the joint
# variables, and then runs the forward dynamics calculations on these
# timelines and plots the resulting paths that would be drawn

def forwards_kinematics_position(robotArm, simulated_timeline, delta_t):
    return positions

def forwards_dynamics_velocities(robotArm, simulated_timeline, delta_t):
    return velocities

def plot_forwards_simulation_results(positions_velocities):
    return

world = RobotWorld.World()
robotArm = RobotWorld.RobotArm(world)

delta_t = 0.01
simulated_timeline = SimulatedTimelineGenerator.generate_random_timeline(robotArm)
positions = forwards_kinematics_position(robotArm, simulated_timeline, delta_t)
velocities = forwards_dynamics_velocities(robotArm, simulated_timeline, delta_t)
