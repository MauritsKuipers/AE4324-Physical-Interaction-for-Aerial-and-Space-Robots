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
ax.scatter(ee_x_max, ee_y_max, ee_z_max, c='r', alpha=0.1, label='End Effector Points')
ax.scatter(ee_x_min, ee_y_min, ee_z_min, c='b', alpha=0.1, label='End Effector Points')
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
ax.set_title('End Effector Workspace')
ax.legend()


plt.show()









# ## Initate Robotic Arm Class ##
# robot = Johny()
#
# ## Sweep over all ##
# hist = robot.sweep_over_all()
#
# ### Data Structuring ###
# ee_coordinates = list(zip(hist["EE"]["x"], hist["EE"]["y"], hist["EE"]["z"]))
#
# # Create a 3D plot
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
#
# # Extract x, y, z coordinates from ee_coordinates
# ee_x, ee_y, ee_z = zip(*ee_coordinates)
# print("here 1")
# # Plot the points
# ax.scatter(ee_x, ee_y, ee_z, c='r', alpha=0.1, label='End Effector Points')
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.set_title('End Effector Workspace')
# ax.legend()

# Show the plot
plt.show()


# ## This was to attempt to have just the outer shell of the limits
# from JohnyKinematics import Johny
# import matplotlib.pyplot as plt
# from scipy.spatial import ConvexHull
# import numpy as np
#
# ## Initiate Robotic Arm Class ##
# robot = Johny()
#
# ## Sweep over all ##
# hist = robot.sweep_over_all()
#
# ### Data Structuring ###
# ee_coordinates = np.array(list(zip(hist["EE"]["x"], hist["EE"]["y"], hist["EE"]["z"])))  # Convert to numpy array
#
# # Calculate convex hull
# hull = ConvexHull(ee_coordinates)
#
# # Create a 3D plot
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
#
# # Plot the convex hull perimeter
# for simplex in hull.simplices:
#     ax.plot(ee_coordinates[simplex, 0], ee_coordinates[simplex, 1], ee_coordinates[simplex, 2], 'k-')
#
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.set_title('End Effector Workspace Perimeter')
#
# # Show the plot
# plt.show()


# ## Initate Robotic Arm Class ##
# robot = Johny()
#
# ## Sweep over all ##
# hist= robot.sweep_over_ang0_for_all_ang0()
#
# ### Data Structuring ###
# ee_coordinates = list(zip(hist["EE"]["x"], hist["EE"]["y"], hist["EE"]["z"]))
#
# # Create a 3D plot
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
#
# # Extract x, y, z coordinates from ee_coordinates
# ee_x, ee_y, ee_z = zip(*ee_coordinates)
#
# print("here 1")
# # Plot the points
# ax.scatter(ee_x, ee_y, ee_z, c='r', alpha=0.1, label='End Effector Points')
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.set_title('End Effector Workspace')
# ax.legend()
#
# # Show the plot
# plt.show()


# from JohnyKinematics import Johny
# import matplotlib.pyplot as plt
# from scipy.spatial import ConvexHull, Delaunay
# import numpy as np
#
# ## Initiate Robotic Arm Class ##
# robot = Johny()
#
# ## Sweep over all ##
# hist = robot.sweep_over_all()
#
# ### Data Structuring ###
# ee_coordinates = np.array(list(zip(hist["EE"]["x"], hist["EE"]["y"], hist["EE"]["z"])))  # Convert to numpy array
#
# # Calculate convex hull
# hull = ConvexHull(ee_coordinates)
#
# # Calculate Delaunay triangulation
# tri = Delaunay(ee_coordinates[hull.vertices])
#
# # Create a 3D plot
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
#
# # Plot the convex hull edges
# for simplex in tri.simplices:
#     ax.plot(ee_coordinates[hull.vertices[simplex], 0],
#             ee_coordinates[hull.vertices[simplex], 1],
#             ee_coordinates[hull.vertices[simplex], 2],
#             'k-', alpha = 0.1)
#
# ax.set_xlabel('X')
# ax.set_ylabel('Y')
# ax.set_zlabel('Z')
# ax.set_title('End Effector Workspace Outer Shell')
#
# # Show the plot
# plt.show()
