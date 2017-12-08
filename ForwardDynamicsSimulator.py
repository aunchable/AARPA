# This file is the main simulator which has a RobotWorld, a
# SimulatedTimelineGenerator to generate random timelines for the joint
# variables, and then runs the forward dynamics calculations on these
# timelines and plots the resulting paths that would be drawn

import numpy as np
import matplotlib.pyplot as plt

import RobotWorld
import SimulatedTimelineGenerator

def g_st(r, theta):
    [t1, t2, t3, t4, t5] = theta
    g = np.zeros(shape=(4, 4))

    # Useful quantities
    c123 = np.cos(t1 + t2 + t3)
    s123 = np.sin(t1 + t2 + t3)
    c5 = np.cos(t5)
    s5 = np.sin(t5)

    # Rotational part
    g[0][0] = c123 * c5
    g[0][1] = -s123
    g[0][2] = c123 * s5
    g[1][0] = s123 *c5
    g[1][1] = c123
    g[1][2] = s123 * s5
    g[2][0] = -s5
    g[2][1] = 0
    g[2][2] = c5

    # Positional part
    g[0][3] = -r.L1 * np.sin(t1) - r.L2 * np.sin(t1 + t2) - r.L3 * c123 * s5
    g[1][3] = r.L1 * np.cos(t1) + r.L2 * np.cos(t1 + t2) - r.L3 * s123 * s5
    g[2][3] = r.H + r.L3 + t4 - r.L3 * c5
    g[3][3] = 1

    return g


def Js_st(r, theta):
    [t1, t2, t3, t4, t5] = theta
    J = np.zeros(shape=(6, 5))

    # Useful quantities
    c1 = np.cos(t1)
    s1 = np.sin(t1)
    c12 = np.cos(t1 + t2)
    s12 = np.sin(t1 + t2)
    c123 = np.cos(t1 + t2 + t3)
    s123 = np.sin(t1 + t2 + t3)

    J[0][0] = 0
    J[0][1] = r.L1 * c1
    J[0][2] = r.L1 * c1 + r.L2 * c12
    J[0][3] = 0
    J[0][4] = -(r.H + r.L3 + t4) * c123

    J[1][0] = 0
    J[1][1] = r.L1 * s1
    J[1][2] = r.L1 * s1 + r.L2 * s12
    J[1][3] = 0
    J[1][4] = -(r.H + r.L3 + t4) * s123

    J[2][0] = 0
    J[2][1] = 0
    J[2][2] = 0
    J[2][3] = 1
    J[2][4] = r.L2 * np.sin(t3) + r.L1 * np.sin(t2 + t3)

    J[3][0] = 0
    J[3][1] = 0
    J[3][2] = 0
    J[3][3] = 0
    J[3][4] = -s123

    J[4][0] = 0
    J[4][1] = 0
    J[4][2] = 0
    J[4][3] = 0
    J[4][4] = c123

    J[5][0] = 1
    J[5][1] = 1
    J[5][2] = 1
    J[5][3] = 0
    J[5][4] = 0

    return J


def twist_skewsym(twist):
    pos_vec = twist[:3]
    omega_vec = twist[3:]
    omega_skewsym = np.array([[0,-omega_vec[2],omega_vec[1]],
                              [omega_vec[2],0,-omega_vec[0]],
                              [-omega_vec[1],omega_vec[0],0]])
    skewsym = np.zeros(shape=(4,4))
    skewsym[:3, :3] = omega_skewsym
    skewsym[:3, 3] = pos_vec
    return skewsym


def forwards_kinematics_position(robotArm, timeline, delta_t):
    if len(timeline) == 0:
        return [], [], []

    robot_to_canvas = [robotArm.world.ROBOT_BASE_X,
                       robotArm.world.ROBOT_BASE_Y,
                       0.0]

    t_pos = []
    q1_pos = []
    q2_pos = []
    timestamps = []

    g = g_st(robotArm, timeline[0][1:])
    t_pos.append(g[:3, 3] + robot_to_canvas)
    q1_pos.append(np.dot(g, robotArm.QB1)[:3] + robot_to_canvas)
    q2_pos.append(np.dot(g, robotArm.QB2)[:3] + robot_to_canvas)
    timestamps.append(0.0)

    last_t = 0.0
    for timestep in timeline[1:]:
        if timestep[0] >= last_t + delta_t:
            g = g_st(robotArm, timestep[1:])
            t_pos.append(g[:3, 3] + robot_to_canvas)
            q1_pos.append(np.dot(g, robotArm.QB1)[:3] + robot_to_canvas)
            q2_pos.append(np.dot(g, robotArm.QB2)[:3] + robot_to_canvas)
            timestamps.append(timestep[0])
            last_t = timestep[0]

    timestamps = np.asarray(timestamps)
    t_pos = np.asarray(t_pos)
    q1_pos = np.asarray(q1_pos)
    q2_pos = np.asarray(q2_pos)
    return t_pos, q1_pos, q2_pos, timestamps


def forwards_dynamics_velocities(robotArm, timeline, t_pos, timestamps, delta_t):
    if len(timeline) <= 2:
        return [], [], []

    t_vel = [[0.0, 0.0, 0.0]]

    J = Js_st(robotArm, timeline[1][1:])
    dtheta = ((timeline[2][1:] - timeline[0][1:]) /
              (timeline[2][0] - timeline[0][0]))
    Vs_st = np.dot(J, dtheta)
    idx = np.argmin(np.abs(timestamps-timeline[1][0]))
    t_pos_ext = np.array([t_pos[idx][0], t_pos[idx][1], t_pos[idx][2], 1.0])
    t_vel.append(np.dot(twist_skewsym(Vs_st), t_pos_ext)[:3])

    last_t = timeline[1][0]
    for i, timestep in enumerate(timeline[:-1]):
        if timestep[0] >= last_t + delta_t:
            J = Js_st(robotArm, timestep[1:])
            dtheta = ((timeline[i+1][1:] - timeline[i-1][1:]) /
                      (timeline[i+1][0] - timeline[i-1][0]))
            Vs_st = np.dot(J, dtheta)
            idx = np.argmin(np.abs(timestamps-timestep[0]))
            t_pos_ext = np.array([t_pos[idx][0], t_pos[idx][1], t_pos[idx][2], 1.0])
            t_vel.append(np.dot(twist_skewsym(Vs_st), t_pos_ext)[:3])
            last_t = timestep[0]

    t_vel.append(t_vel[-1])

    t_vel = np.asarray(t_vel)

    return t_vel


def plot_forwards_simulation_results(timeline, t_pos, q1_pos, q2_pos, t_vel):
    # Timeline plot
    plt.subplot(5, 1, 1)
    plt.plot(timeline[:, 0], timeline[:, 1], 'k-')
    plt.title('Simulated Joint Variable Timeline')
    plt.ylabel('Theta 1')
    plt.subplot(5, 1, 2)
    plt.plot(timeline[:, 0], timeline[:, 2], 'k-')
    plt.ylabel('Theta 2')
    plt.subplot(5, 1, 3)
    plt.plot(timeline[:, 0], timeline[:, 3], 'k-')
    plt.ylabel('Theta 3')
    plt.subplot(5, 1, 4)
    plt.plot(timeline[:, 0], timeline[:, 4], 'k-')
    plt.ylabel('Theta 4')
    plt.subplot(5, 1, 5)
    plt.plot(timeline[:, 0], timeline[:, 5], 'k-')
    plt.ylabel('Theta 5')
    plt.xlabel('Time (s)')
    plt.savefig('plots/timeline4.png')
    plt.close()

    # Position plot
    plt.scatter(t_pos[:, 0], t_pos[:, 1])
    plt.scatter(q1_pos[:, 0], q1_pos[:, 1], c='k')
    plt.scatter(q2_pos[:, 0], q2_pos[:, 1], c='k')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.xlim([-100, 100])
    plt.ylim([-100, 100])
    plt.gca().set_aspect('equal', adjustable='box')
    plt.title('Position over time')
    plt.savefig('plots/position4.png')
    plt.close()

    # Velocity plot
    plt.figure()
    s = 30
    Q = plt.quiver(t_pos[:, 0][1::s], t_pos[:, 1][1::s], t_vel[:, 0][1::s], t_vel[:, 1][1::s])
    qk = plt.quiverkey(Q, 0.9, 0.9, 1, 'cm/s', labelpos='E', coordinates='figure')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.xlim([-100, 100])
    plt.ylim([-100, 100])
    plt.gca().set_aspect('equal', adjustable='box')
    plt.title('Linear velocity over time')
    plt.savefig('plots/velocity4.png')
    plt.close()
    return


world = RobotWorld.World()
robotArm = RobotWorld.RobotArm(world)

delta_t = 0.01
simulated_timeline = SimulatedTimelineGenerator.generate_linear_timeline(
                        robotArm, [True, True, False, False, False])
# simulated_timeline = SimulatedTimelineGenerator.generate_piecewise_linear_timeline(
#                         robotArm, [True, True, True, False, False], time_length=10)
t_pos, q1_pos, q2_pos, timestamps = forwards_kinematics_position(robotArm, simulated_timeline, delta_t)
t_vel = forwards_dynamics_velocities(robotArm, simulated_timeline, t_pos, timestamps, delta_t)

print(simulated_timeline[:5])
print(t_pos[:5])
print(q1_pos[:5])
print(q2_pos[:5])
print(t_vel[:5])

plot_forwards_simulation_results(simulated_timeline, t_pos, q1_pos, q2_pos, t_vel)
