# Generates simulated timelines for joint variables for running the
# ForwardsDynamicsSimulator to see how the path will behave

import numpy as np

TIMELINE_DT = 0.01

def generate_piecewise_linear_timeline(r, theta_idx, time_length=5):
    timeline = [[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]]
    delta = np.asarray([TIMELINE_DT, 0.0, 0.0, 0.0, 0.0, 0.0])
    for i in range(int(time_length / TIMELINE_DT)):
        if i % int(1 / TIMELINE_DT) == 0:
            for j in range(len(theta_idx)):
                if theta_idx[j]:
                    delta[j + 1] = (np.random.rand() - 0.25) * TIMELINE_DT
        timeline.append(timeline[-1] + delta)
    timeline = np.asarray(timeline)

    # timeline[:,5] = np.pi * 0.15 * np.ones(len(timeline))

    return timeline


def generate_linear_timeline(r, theta_idx, time_length=5):
    timeline = []
    delta = np.asarray([TIMELINE_DT, 0.0, 0.0, 0.0, 0.0, 0.0])
    for j in range(len(theta_idx)):
        if theta_idx[j]:
            delta[j + 1] = np.random.rand() * TIMELINE_DT
    for i in range(int(time_length / TIMELINE_DT)):
        timeline.append(i * delta)
    timeline = np.asarray(timeline)

    # timeline[:,3] = np.pi * 0.4 * np.ones(len(timeline))
    # timeline[:,5] = np.pi * 0.15 * np.ones(len(timeline))

    return timeline
