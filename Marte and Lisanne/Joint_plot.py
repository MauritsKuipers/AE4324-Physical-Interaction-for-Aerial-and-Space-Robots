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

    # plot axis limits:
    all_joint_positions = xs + ys + zs
    margin = 1.1
    lim = max([abs(element) for element in all_joint_positions]) * margin

    # for scaling the robot in space properly (z = 0 always ground):
    ax.set_xlim3d(-lim, lim)
    ax.set_ylim3d(-lim, lim)
    ax.set_zlim3d(0, 2*lim)

    # Add target EE position (for inverse kinematics verification)
    if EE_plot_pos:
        x, y, z = EE_plot_pos
        ax.scatter(x, y, z, color="red")
    plt.show()
    print(EE_pos)
"""
t0 = np.deg2rad(45)
t1 = np.deg2rad(-13.1)
t2 = np.deg2rad(136.3)
t3 = np.pi/2 - t2 - t1


L1 = 95
L2 = 105
new_t1 = t1 + 2 * L2 * np.arcsin(np.sin(t2)/np.sqrt(L1**2 + L2 ** 2 + 2 * L1 * L2 * np.cos(t2)))
joint_plot(t0, new_t1,    -t2,    -t3, [100,-100,100])
joint_plot(t0, t1,          t2,     t3, [100,-100,100])
"""

t0 = np.deg2rad(0)
t1 = np.deg2rad(30)
t2 = np.deg2rad(45)
t3 = np.pi/2 - t2 - t1

L1 = 95
L2 = 105
new_t1 = t1 + 2 * L2 * np.arcsin(np.sin(t2)/np.sqrt(L1**2 + L2 ** 2 + 2 * L1 * L2 * np.cos(t2)))
joint_plot(t0, new_t1,    -t2,    -t3)
joint_plot(t0, t1,          t2,     t3)
