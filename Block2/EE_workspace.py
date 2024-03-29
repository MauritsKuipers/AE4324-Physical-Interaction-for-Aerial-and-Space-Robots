from JohnyKinematics import Johny
import matplotlib.pyplot as plt
import matplotlib
from math import pi, sqrt
import numpy as np


## Initate Robotic Arm Class ##
robot = Johny()

## Sweep over all ##
hist_max, hist_min = robot.EE_workspace()

### Data Structuring ###
ee_coordinates_max = list(zip(hist_max["EE"]["x"], hist_max["EE"]["y"], hist_max["EE"]["z"]))
ee_coordinates_min = list(zip(hist_min["EE"]["x"], hist_min["EE"]["y"], hist_min["EE"]["z"]))

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Extract x, y, z coordinates from ee_coordinates_max/min
ee_x_max, ee_y_max, ee_z_max = zip(*ee_coordinates_max)
ee_x_min, ee_y_min, ee_z_min = zip(*ee_coordinates_min)

print("here 1")
# Plot the points
ax.scatter(ee_x_max, ee_y_max, ee_z_max, c='r', alpha=0.1)
ax.scatter(ee_x_min, ee_y_min, ee_z_min, c='b', alpha=0.1)
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('End Effector Workspace')
ax.legend()


plt.show()



