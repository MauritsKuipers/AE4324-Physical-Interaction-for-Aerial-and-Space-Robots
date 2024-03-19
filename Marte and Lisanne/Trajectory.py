# We are going to trace a small circle in the global x-z plane

import numpy as np
import matplotlib.pyplot as plt

def trajectory_positions():
    # a circle discretized into 20 points
    angles = np.linspace(0,360,5)

    # the ee positions
    x = []
    y = []
    z = []

    radius = 20
    for i in angles:
        x.append(radius + radius*np.cos(i*np.pi/180))
        y.append(50)
        z.append(radius + radius*np.sin(i*np.pi/180))

    # plotting the points to check the shape
    plt.scatter(x,z)
    plt.show()
    return [x,y,z]

