# We are going to trace a small circle in the global x-z plane

import numpy as np
import matplotlib.pyplot as plt

# a circle discretized into 20 points
angles = np.linspace(0,360,20)

# the ee positions
x = []
y = []
z = []
for i in angles:
    x.append(50 + 50*np.cos(i*np.pi/180))
    y.append(0)
    z.append(50 + 50*np.sin(i*np.pi/180))

# plotting the points to chech the shape
plt.scatter(x,z)
plt.show()

