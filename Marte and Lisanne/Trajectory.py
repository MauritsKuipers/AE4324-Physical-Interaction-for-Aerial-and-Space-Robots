# We are going to trace a small circle in the global x-z plane

import numpy as np
import matplotlib.pyplot as plt

def trajectory_positions():
    # a circle discretized into 20 points
    angles = np.linspace(0,360,20)

    # the ee positions
    x = []
    y = []
    z = []

    radius = 10
    for i in angles:
        x.append(30 + radius + radius*np.cos(i*np.pi/180))
        y.append(0)
        z.append(30 + radius + radius*np.sin(i*np.pi/180))

    # plotting the points to check the shape
    plt.scatter(x,z)
    plt.show()
    return [x,y,z]

