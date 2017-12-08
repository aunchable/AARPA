# Generates simulated timelines for joint variables for running the
# ForwardsDynamicsSimulator to see how the path will behave

import numpy as np

TIMELINE_DT = 0.01

def generate_random_timeline(r):
    return


def generate_linear_timeline(r, theta_idx, time_length=5):
    timeline = []
    delta = np.asarray([TIMELINE_DT, 0.0, 0.0, 0.0, 0.0, 0.0])
    for j in range(len(theta_idx)):
        if theta_idx[j]:
            delta[j + 1] = np.random.rand() * TIMELINE_DT
    for i in range(int(time_length / TIMELINE_DT)):
        timeline.append(i * delta)
    timeline = np.asarray(timeline)
    return timeline
