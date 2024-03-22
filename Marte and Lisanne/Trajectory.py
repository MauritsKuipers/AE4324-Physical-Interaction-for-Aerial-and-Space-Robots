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

    radius = 50
    """
    for i in angles:
        x.append(50 + radius + radius*np.cos(i*np.pi/180))
        y.append(-100)
        z.append(30 + radius + radius*np.sin(i*np.pi/180))
    """
    x_array = np.arange(-120, 120, 5)
    for x_value in x_array:
        x.append(x_value)
        z.append(np.sqrt(120**2 - x_value**2)+150)
        y.append(-100)
    for x_value in np.invert(x_array):
        x.append(x_value)
        z.append(-np.sqrt(120**2 - x_value**2)+150)
        y.append(-100)
    # plotting the points to check the shape
    plt.scatter(x,z)
    plt.show()
    return [x,y,z]

