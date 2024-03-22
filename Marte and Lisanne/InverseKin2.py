import numpy as np
import matplotlib.pyplot as plt
from Joint_plot import joint_plot
from Trajectory import trajectory_positions

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

    if L_13 > (L_12 + L_23):
        raise ValueError

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
        # first check for errors (position is out of workspace for non-restricted joints)
        try:
            theta_0, theta_1, theta_2, theta_3 = joint_angles_projection(position, thEE)

        except:
            continue

        # check if the angles are within their physical limits
        th0_in_limits = bool(th0_min <= theta_0 <= th0_max)
        th1_in_limits = bool(th1_min <= theta_1 <= th1_max)
        th2_in_limits = bool(th2_min <= theta_2 <= th2_max)
        th3_in_limits = bool(th3_min <= theta_3 <= th3_max)

        # todo: easier way and quicker way of writing this
        if not th0_in_limits:
            # no way of fixing this
            within_joint_limits = False
        elif not th1_in_limits or not th2_in_limits or not th3_in_limits:
            # check elbow up / down is within the limits
            L1 = 95
            L2 = 105

            theta_1_new = theta_1 + 2 * np.arcsin(L2 * np.sin(theta_2)/np.sqrt(L1 ** 2 + L2 ** 2 + 2 * L1 * L2 * np.cos(theta_2)))
            theta_2_new = -theta_2
            theta_3_new = np.pi/2 - theta_2_new - theta_1_new + thEE

            if (th1_min <= theta_1_new <= th1_max and th2_min <= theta_2_new <= th2_max and th3_min <= theta_3_new <= th3_max):

                theta_1 = theta_1_new
                theta_2 = theta_2_new
                theta_3 = theta_3_new

                within_joint_limits = True
            else:
                # reversing the elbow didn't get it within limits, so the position is not reachable with joint limits
                within_joint_limits = False
        else:
            # this means all angles are within limits at first try (no need for elbow reversal)
            within_joint_limits = True

        # Plot the robot (or not) and output if the plot is plotting a constrained or unconstrained robot.
        if within_joint_limits:
            #joint_plot(theta_0, theta_1, theta_2, theta_3)

            #print(f"EE position: {position} is in the robot workspace!")
            print(f"theta_EE: {np.rad2deg(thEE)} \n"
                  f"theta_0: {np.rad2deg(theta_0)} \n"
                  f"theta_1: {np.rad2deg(theta_1)} \n"
                  f"theta_2: {np.rad2deg(theta_2)} \n"
                  f"theta_3: {np.rad2deg(theta_3)}")
            return theta_0, theta_1, theta_2, theta_3
        elif not joint_limited:
            # When the joints aren't restricted
            # joint_plot(theta_0, theta_1, theta_2, theta_3)
            print(f"EE position: {position} is in the robot UNRESTRICTED workspace!")
            return theta_0, theta_1, theta_2, theta_3
        else:
            continue

    print(f"EE position: {position} is NOT in the robot workspace... :(")

if __name__ == "__main__":
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
    #
    pos5 = [0, -150, 50]

    #try_EE_position(pos5, True)

def trajectory_joint_angles(xyz_positions, speed=2):
    """ Takes list with form: [[x1, x2, ..., xn], [y1, y2, ..., yn], [z1, z2, ..., zn]]
    and outputs corresponding joint angles with time list. """
    x = xyz_positions[0]
    y = xyz_positions[1]
    z = xyz_positions[2]

    th0_list = []
    th1_list = []
    th2_list = []
    th3_list = []

    for i in range(len(x)):
        #print('position', i)
        th0, th1, th2, th3 = try_EE_position((x[i], y[i], z[i]), True)
        th0_list.append(th0)
        th1_list.append(th1)
        # Robot flips theta 2 angle
        th2_list.append(-th2)
        th3_list.append(th3)
        # note: theta 2 also has an offset but will be added in other file
    t = np.ndarray.tolist(np.arange(len(x)+1)[1:] * speed)

    return {"time": t, "theta_0": th0_list, "theta_1": th1_list, "theta_2": th2_list, "theta_3": th3_list}

#trajectory_joint_angles(trajectory_positions())

# try_EE_position([100, -100, 100])
print(trajectory_joint_angles([[0], [0], [300]]))
# {'time': [2], 'theta_0': [0.7853981633974483], 'theta_1': [0.15083233291732956], 'theta_2': [-2.40719150909267], 'theta_3': [-1.379926596913827]}
# {'time': [2], 'theta_0': [1.5707963267948966], 'theta_1': [-0.6819964922774115], 'theta_2': [-1.2888814029541067], 'theta_3': [-0.6068849106766951]}
{'time': [2,
  4,
  6,
  8,
  10,
  12,
  14,
  16,
  18,
  20,
  22,
  24,
  26,
  28,
  30,
  32,
  34,
  36,
  38,
  40],
 'theta_0': [1.5707963267948966,
  1.5707963267948966,
  1.5707963267948966,
  1.5707963267948966,
  1.5707963267948966,
  1.5707963267948966,
  1.5707963267948966,
  1.5707963267948966,
  1.5707963267948966,
  1.5707963267948966,
  1.5707963267948966,
  1.5707963267948966,
  1.5707963267948966,
  1.5707963267948966,
  1.5707963267948966,
  1.5707963267948966,
  1.5707963267948966,
  1.5707963267948966,
  1.5707963267948966,
  1.5707963267948966],
 'theta_1': [-0.9482557055134944,
  -0.9512052692805266,
  -0.9590812790204204,
  -0.9571000165102772,
  -0.9531788452433303,
  -0.9537190913023696,
  -0.953067697615896,
  -0.956473087623118,
  -0.9581071938126486,
  -0.9527183316956847,
  -0.9495260432354042,
  -0.9490881915121747,
  -0.858761830881742,
  -0.8075291691731219,
  -0.777749534395145,
  -0.7773565243469567,
  -0.7994964539505425,
  -0.8484651863261994,
  -0.901817063944407,
  -0.9482557055134944],
 'theta_2': [2.6035891397338897,
  2.5715157821771952,
  2.5424409758945656,
  2.5142388805896654,
  2.493738332580646,
  2.4857404710621145,
  2.489066113433589,
  2.5047198435153497,
  2.5286646271149147,
  2.556371751269577,
  2.5869253712116675,
  2.6171098957235923,
  2.615907131565505,
  2.6175098897207776,
  2.616430842854018,
  2.616287883865217,
  2.6157647123811376,
  2.617974246259471,
  2.614925092887102,
  2.6035891397338897],
 'theta_3': [1.1808266002703902,
  1.2245761678540885,
  1.2789802763965552,
  1.340107694231199,
  1.4003203022731294,
  1.4524916411505586,
  1.4921478363924687,
  1.5148060813578166,
  1.518675342727697,
  1.503032648976014,
  1.4692867405736432,
  1.4212110718185456,
  1.358267414126115,
  1.2967053480022508,
  1.2418248213111187,
  1.1979415389518733,
  1.1682447624797083,
  1.1539174371572305,
  1.157958590587977,
  1.1808266002703902]}
