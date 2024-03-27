import numpy as np
import matplotlib.pyplot as plt
from Transformation import transform_B20, transform_021, transform_122, transform_223, transform_32EE


def rad(angle_in_deg):
    return np.deg2rad(angle_in_deg)

def j0_pos(th0, xy_only=True):
    pos = transform_B20(th0, 45) @ np.matrix([[0], [0], [0], [1]])
    if xy_only:
        # will only plot y and z values (output only [1:3] values of EE position in normal array format)
        return np.array(pos[1:3]).flatten()
    else:
        return np.array(pos[0:3]).flatten()
def j1_pos(th0, th1, xy_only=True):
    pos = (transform_B20(th0, 45) @
          transform_021(th1, 20) @ np.matrix([[0], [0], [0], [1]])
           )
    if xy_only:
        # will only plot y and z values (output only [1:3] values of EE position in normal array format)
        return np.array(pos[1:3]).flatten()
    else:
        return np.array(pos[0:3]).flatten()
def j2_pos(th0, th1, th2, xy_only=True):
    pos = (transform_B20(th0, 45) @
          transform_021(th1, 20) @
          transform_122(th2, 95) @ np.matrix([[0], [0], [0], [1]])
           )
    if xy_only:
        # will only plot y and z values (output only [1:3] values of EE position in normal array format)
        return np.array(pos[1:3]).flatten()
    else:
        return np.array(pos[0:3]).flatten()

def j3_pos(th0, th1, th2, th3, xy_only=True):
    pos = (transform_B20(th0, 45) @
          transform_021(th1, 20) @
          transform_122(th2, 95) @
          transform_223(th3,105) @ np.matrix([[0], [0], [0], [1]])
           )

    if xy_only:
        # will only plot y and z values (output only [1:3] values of EE position in normal array format)
        return np.array(pos[1:3]).flatten()
    else:
        return np.array(pos[0:3]).flatten()
def EE_pos(th0, th1, th2, th3, thEE, xy_only=True):
    pos = (transform_B20(th0, 45) @
           transform_021(th1, 20) @
           transform_122(th2, 95) @
           transform_223(th3, 105) @
           transform_32EE(thEE, 75) @ np.matrix([[0], [0], [0], [1]]))
    if xy_only:
        # will only plot y and z values (output only [1:3] values of EE position in normal array format)
        return np.array(pos[1:3]).flatten()
    else:
        return np.array(pos[0:3]).flatten()


def robot_plot(th0, th1, th2, th3, thEE, xy_only=True):
    if xy_only:
        xs = [0,
              j0_pos(th0, xy_only)[0],
              j1_pos(th0, th1, xy_only)[0],
              j2_pos(th0, th1, th2, xy_only)[0],
              j3_pos(th0, th1, th2, th3, xy_only)[0],
              EE_pos(th0, th1, th2, th3, thEE, xy_only)[0]]
        ys = [0,
              j0_pos(th0, xy_only)[1],
              j1_pos(th0, th1, xy_only)[1],
              j2_pos(th0, th1, th2, xy_only)[1],
              j3_pos(th0, th1, th2, th3, xy_only)[1],
              EE_pos(th0, th1, th2, th3, thEE, xy_only)[1]]
        joints = ["B", 0, 1, 2, 3, "EE"]

        plt.plot(xs, ys, '-o')

        for i, txt in enumerate(joints):
            plt.annotate(txt, (xs[i], ys[i]))
    else:
        pass

###################################################
# INPUTS:
plot_2D = False

th0_min, th0_max = [rad(-90), rad(90)]
th1_min, th1_max = [rad(-55), rad(90)]
th2_min, th2_max = [rad(45), rad(150)]
th3_min, th3_max = [rad(-80), rad(100)]
"""
th0_min, th0_max = [rad(-90), rad(90)]
th1_min, th1_max = [rad(-180), rad(180)]
th2_min, th2_max = [rad(0), rad(0)]
th3_min, th3_max = [rad(0), rad(0)]
"""
###################################################

if not plot_2D:
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

# theta 4 is always 0
theta4 = 0

###################################################
# MAX REACH:
###################################################
if plot_2D:
    # start position
    theta0 = 0
    # end position:
    theta0_end = 0
    # number of times to plot theta0
    n = 1
else:
    # start position:
    theta0 = th0_min
    # end position:
    theta0_end = th0_max
    # number of times to plot theta0
    n = 30

EE_position_max_list = []

if plot_2D:
    # plot ground:
    plt.plot([-300, 200], [0, 0], color='k')
    plt.xlabel("y")
    plt.ylabel("z")


for theta0 in np.linspace(theta0, theta0_end, n):
    # start position:
    theta1 = th1_min
    theta2 = th2_min
    theta3 = th3_min

    # trajectory 1:
    robot_plot(theta0, theta1, theta2, theta3, theta4, plot_2D)
    for theta3 in np.linspace(th3_min, 0, 10):
        EE_position_max_list.append(EE_pos(theta0, theta1, theta2, theta3, theta4, plot_2D))
    # trajectory 2:
    #robot_plot(theta0, theta1, theta2, theta3, theta4, plot_2D)
    for theta1 in np.linspace(th1_min, th1_max, 30):
        # theta 2 and 3 at end position of previous trajectory
        EE_position_max_list.append(EE_pos(theta0, theta1, theta2, theta3, theta4, plot_2D))
    # trajectory 3:
    #robot_plot(theta0, theta1, theta2, theta3, theta4, plot_2D)
    for theta2 in np.linspace(th2_min, th2_max, 20):
        EE_position_max_list.append(EE_pos(theta0, theta1, theta2, theta3, theta4, plot_2D))
    # trajectory 4:
    #robot_plot(theta0, theta1, theta2, theta3, theta4, plot_2D)
    for theta3 in np.linspace(0, th3_max, 10):
        EE_position_max_list.append(EE_pos(theta0, theta1, theta2, theta3, theta4, plot_2D))

    #robot_plot(theta0, theta1, theta2, theta3, theta4, plot_2D)

EE_position_max_list = np.array(EE_position_max_list)
if plot_2D:
    plt.plot(EE_position_max_list[:,0], EE_position_max_list[:,1], color="r")
else:
    xs = EE_position_max_list[:,0]
    ys = EE_position_max_list[:,1]
    zs = EE_position_max_list[:,2]

    ax.scatter(xs, ys, zs)
    ax.scatter(100,-100,100, color="red")

###################################################
# MIN REACH:
###################################################

if plot_2D:
    # start position
    theta0 = 0
    # end position:
    theta0_end = 0
    # number of times to plot theta0
    n = 1
else:
    # start position:
    theta0 = th0_min
    # end position:
    theta0_end = th0_max
    # number of times to plot theta0
    n = 30

EE_position_min_list = []

# for theta0 in np.linspace(theta0, theta0_end, n):
#
#     # start position:
#     theta1 = th1_min
#     theta2 = th2_min
#     theta3 = 0
#
#     # trajectory 1:
#     # robot_plot(theta0, theta1, theta2, theta3, theta4, plot_2D)
#     for theta3 in np.linspace(theta3, th3_max, 10):
#         EE_position_min_list.append(EE_pos(theta0, theta1, theta2, theta3, theta4, plot_2D))
#
#     # trajectory 2:
#     # robot_plot(theta0, theta1, theta2, theta3, theta4, plot_2D)
#     for theta2 in np.linspace(th2_min, th2_max, 30):
#         EE_position_min_list.append(EE_pos(theta0, theta1, theta2, theta3, theta4, plot_2D))
#
#     # trajectory 3:
#     # robot_plot(theta0, theta1, theta2, theta3, theta4, plot_2D)
#     for theta1 in np.linspace(th1_min, th1_max, 20):
#         EE_position_min_list.append(EE_pos(theta0, theta1, theta2, theta3, theta4, plot_2D))
#
# EE_position_min_list = np.array(EE_position_min_list)
# if plot_2D:
#     plt.plot(EE_position_min_list[:,0], EE_position_min_list[:,1], color="r")
#
# else:
#     xs = EE_position_min_list[:,0]
#     ys = EE_position_min_list[:,1]
#     zs = EE_position_min_list[:,2]
#
#     ax.scatter(xs, ys, zs)
#
#     ax.set_xlabel("x")
#     ax.set_ylabel("y")
#     ax.set_zlabel("z")

plt.show()