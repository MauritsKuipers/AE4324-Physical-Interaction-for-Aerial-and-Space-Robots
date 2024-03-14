import numpy as np
import matplotlib.pyplot as plt
from Transformation import transform_B20, transform_021, transform_122, transform_223, transform_32EE

def joint_plot(theta0, theta1, theta2, theta3, EE_plot_pos=None):
    # vector to be multiplied:
    vector = np.matrix([[0], [0], [0], [1]])

    thetaEE = 0     # doesn't matter just math idk

    # joint positions:
    j0_pos = (transform_B20(theta0, 45) @ vector)

    j1_pos = (transform_B20(theta0, 45) @
            transform_021(theta1, 20) @ vector)

    j2_pos = (transform_B20(theta0, 45) @
            transform_021(theta1, 20) @
            transform_122(theta2, 95) @ vector)

    j3_pos = (transform_B20(theta0, 45) @
            transform_021(theta1, 20) @
            transform_122(theta2, 95) @
            transform_223(theta3,105) @ vector)

    EE_pos = (transform_B20(theta0, 45) @
            transform_021(theta1, 20) @
            transform_122(theta2, 95) @
            transform_223(theta3,105) @
            transform_32EE(thetaEE,75) @ vector)

    # joint coordinates:
    xs = [0, j0_pos[0,0], j1_pos[0,0], j2_pos[0,0], j3_pos[0,0], EE_pos[0,0]]
    ys = [0, j0_pos[1,0], j1_pos[1,0], j2_pos[1,0], j3_pos[1,0], EE_pos[1,0]]
    zs = [0, j0_pos[2,0], j1_pos[2,0], j2_pos[2,0], j3_pos[2,0], EE_pos[2,0]]

    # plot the joints and links:
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    ax.plot(xs, ys, zs, '-o')

    # label the joints:
    joints = ["B", 0, 1, 2, 3, "EE"]
    for i, txt in enumerate(joints):
        ax.text(xs[i], ys[i], zs[i], '%s' % (str(txt)), size=10, zorder=1, color='k')

    # label the axes
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

    # for scaling the robot in space properly:
    ax.set_xlim3d(-300, 300)
    ax.set_ylim3d(-300, 300)
    ax.set_zlim3d(0, 600)
    if EE_plot_pos:
        x, y, z = EE_plot_pos
        ax.scatter(x, y, z, color="red")
    plt.show()