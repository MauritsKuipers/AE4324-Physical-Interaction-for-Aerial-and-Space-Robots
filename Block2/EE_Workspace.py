from JohnyKinematics import Johny
import matplotlib.pyplot as plt




## Initiate Robotic Arm Class ##
robot = Johny()

## Sweep for the Workspace
hist_max, hist_min = robot.EE_workspace()

## Data Structure ##
ee_coordinates_max = list(zip(hist_max["EE"]["x"], hist_max["EE"]["y"], hist_max["EE"]["z"]))
ee_coordinates_min = list(zip(hist_min["EE"]["x"], hist_min["EE"]["y"], hist_min["EE"]["z"]))

## Create 3D Plot ##
fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

## Extract the Coordinates ##
ee_x_max, ee_y_max, ee_z_max = zip(*ee_coordinates_max)
ee_x_min, ee_y_min, ee_z_min = zip(*ee_coordinates_min)

ax.scatter(ee_x_max, ee_y_max, ee_z_max, c="b", label="Maximum End Effector Points")
# ax.scatter(ee_x_min, ee_y_min, ee_z_min, c="r", label="Minimum End Effector Points")
ax.set_xlabel("x")
ax.set_ylabel("y")
ax.set_zlabel("z")
ax.set_title("EE Workspace")
ax.legend()

plt.show()