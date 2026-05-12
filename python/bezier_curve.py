import numpy as np
import matplotlib.pyplot as plt
import math
import time


def visualize(bezier_line, bezier_control_points):
    plt.figure()
    plt.plot(bezier_line[:, 0], bezier_line[:, 1], color='red')
    plt.scatter(bezier_control_points[:, 0], bezier_control_points[:, 1])
    plt.show()


def recursive_bezier(pts, t):
    while True:
        recursive_pts = np.empty(shape=(0, 2))
        for i in np.arange(0, pts.shape[0] - 1):
            pt = t * pts[i] + (1 - t) * pts[i+1]
            recursive_pts = np.append(recursive_pts, np.expand_dims(pt, axis=0), axis=0)

        pts = recursive_pts
        if len(recursive_pts) == 1:
            print(recursive_pts)
            break

    return recursive_pts[0]


if __name__ == '__main__':
    control_points = np.array([(1.0, 0.5), (0.0, 2.0), (3.9, 1.9)])
    recursive_bezier_line = []

    for t in np.arange(0, 1.01, 0.01):
        recursive_bezier_line.append(recursive_bezier(control_points, t))
    bezier_lines = np.stack(recursive_bezier_line, axis=0)
    visualize(bezier_lines, control_points)
