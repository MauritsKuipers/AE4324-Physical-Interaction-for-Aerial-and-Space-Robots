import numpy as np
import matplotlib.pyplot as plt
from Joint_plot import joint_plot

def rad(angle_in_deg):
    return np.deg2rad(angle_in_deg)

def joint_angles_projection(ee_position, theta_ee):

    # "unpack" position vector
    x, y, z = ee_position

    # look at joints within arm plane:
    theta_0, x_ee, y_ee, z_ee = vertical_projection(x, y, z)

    L_12 = 95  # [mm]
    L_23 = 105  # [mm]
    L_3ee = 75  # [mm]

    x_3 = x_ee + L_3ee * np.cos(theta_ee)  # [mm]
    z_3 = z_ee + L_3ee * np.sin(theta_ee)  # [mm]
    L_13 = np.sqrt(z_3 ** 2 + x_3 ** 2)  # [mm]

    theta_1 = np.arccos((L_12 ** 2 + L_13 ** 2 - L_23 ** 2) / (2 * L_12 * L_13)) + np.arctan2(z_3, x_3) - np.pi / 2

    alpha_2 = np.arccos((L_12 ** 2 + L_23 ** 2 - L_13 ** 2) / (2 * L_12 * L_23))  # [rad]

    theta_2 = -(np.pi - alpha_2)  # [rad]
    theta_3 = np.pi / 2 + theta_ee - theta_1 - theta_2  # [rad]

    return theta_0, theta_1, theta_2, theta_3

def vertical_projection(x_global_ee, y_global_ee, z_global_ee):

    global_theta_0 = np.arctan2(y_global_ee,x_global_ee) + np.deg2rad(90)
    planar_x_ee = -np.sqrt(x_global_ee**2 + y_global_ee**2)
    planar_y_ee = 0
    planar_z_ee = z_global_ee - 45 - 20
    return global_theta_0, planar_x_ee, planar_y_ee, planar_z_ee

def try_EE_position(position, joint_limited=True):
    global th0_min, th1_min, th2_min, th3_min, th0_max, th1_max, th2_max, th3_max

    # iterate through orientations of EE gripper with respect to the horizontal
    for thEE in rad(np.arange(-90, 90, 0.5)):
        print(thEE)
        # first check for errors (position is out of workspace for non-restricted joints)
        try:
            theta_0, theta_1, theta_2, theta_3 = joint_angles_projection(position, thEE)

        except:
            print("error")
            continue

        th0_in_limits = bool(th0_min <= theta_0 <= th0_max)
        th1_in_limits = bool(th1_min <= theta_1 <= th1_max)
        th2_in_limits = bool(th2_min <= theta_2 <= th2_max)
        th3_in_limits = bool(th3_min <= theta_3 <= th3_max)

        # check if the angles are within their physical limits
        if within_joint_limits:
            joint_plot(theta_0, theta_1, theta_2, theta_3)
            print(f"EE position: {position} is in the robot workspace!")
            return
        elif not joint_limited:
            # When the joints aren't restricted
            joint_plot(theta_0, theta_1, theta_2, theta_3)
            print(f"EE position: {position} is in the robot UNRESTRICTED workspace!")
            return
        else:
            continue

    print(f"EE position: {position} is NOT in the robot workspace... :(")


th0_min, th0_max = [rad(-90), rad(90)]
th1_min, th1_max = [rad(-55), rad(90)]
th2_min, th2_max = [rad(45), rad(150)]
th3_min, th3_max = [rad(-80), rad(100)]

plt.close()

# assignment positions:
# assume that in assignment they take positive y value away from pcb
# position 1
pos1 = [100, -100, 100]
# position 2
pos2 = [200, -100, 300]
# position 3
pos3 = [0, 0, 300]
# position 4
pos4 = [0, 0, 70]

try_EE_position(pos1, True)

