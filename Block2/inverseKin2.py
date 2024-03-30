import numpy as np
import matplotlib.pyplot as plt
# from Joint_plot import joint_plot
from JohnyKinematics import Johny

robot = Johny()

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
            """
            print(f"theta_EE: {np.rad2deg(thEE)} \n"
                  f"theta_0: {np.rad2deg(theta_0)} \n"
                  f"theta_1: {np.rad2deg(theta_1)} \n"
                  f"theta_2: {np.rad2deg(theta_2)} \n"
                  f"theta_3: {np.rad2deg(theta_3)}")
            """
            return theta_0, theta_1, theta_2, theta_3
        elif not joint_limited:
            # When the joints aren't restricted
            # joint_plot(theta_0, theta_1, theta_2, theta_3)
            print(f"EE position: {position} is in the robot UNRESTRICTED workspace!")
            return theta_0, theta_1, theta_2, theta_3
        else:
            continue

    print(f"EE position: {position} is NOT in the robot workspace... :(")



def trajectory_joint_angles(xyz_positions, speed=1):
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


def joint_plot(theta0, theta1, theta2, theta3, EE_plot_pos=None):
    # vector to be multiplied:
    vector = np.matrix([[0], [0], [0], [1]])

    thetaEE = 0     # doesn't matter just math idk

    # joint positions:

    # j0_pos = (transform_B20(theta0, 45) @ vector)
    #
    # j1_pos = (transform_B20(theta0, 45) @
    #         transform_021(theta1, 20) @ vector)
    #
    # j2_pos = (transform_B20(theta0, 45) @
    #         transform_021(theta1, 20) @
    #         transform_122(theta2, 95) @ vector)
    #
    # j3_pos = (transform_B20(theta0, 45) @
    #         transform_021(theta1, 20) @
    #         transform_122(theta2, 95) @
    #         transform_223(theta3,105) @ vector)
    #
    # EE_pos = (transform_B20(theta0, 45) @
    #         transform_021(theta1, 20) @
    #         transform_122(theta2, 95) @
    #         transform_223(theta3,105) @
    #         transform_32EE(thetaEE,75) @ vector)
    robot.ang0 = theta0 #+ np.pi/2
    robot.ang1 = theta1
    robot.ang2 = theta2
    robot.ang3 = theta3

    locations = robot.get_frame_origins_in_ground_frame()
    j0_pos = locations["J0"]
    j1_pos = locations["J1"]
    j2_pos = locations["J2"]
    j3_pos = locations["J3"]
    EE_pos = locations["EE"]
    print("j0_po: ", j0_pos)
    print("j1_po: ", j1_pos)
    print("j2_po: ", j2_pos)
    print("j3_po: ", j3_pos)
    print("EE_pos: ", EE_pos)
    # joint coordinates:
    xs = [0, j0_pos["x"], j1_pos["x"], j2_pos["x"], j3_pos["x"], EE_pos["x"]]
    ys = [0, j0_pos["y"], j1_pos["y"], j2_pos["y"], j3_pos["y"], EE_pos["y"]]
    zs = [0, j0_pos["z"], j1_pos["z"], j2_pos["z"], j3_pos["z"], EE_pos["z"]]
    print("xs: ", xs)
    print("ys: ", ys)
    print("zs: ", zs)
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
        print("Test")
        x, y, z = EE_plot_pos
        ax.scatter(x/1000, y/1000, z/1000, color="red")
    plt.show()


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
    theta0, theta1, theta2, theta3 = try_EE_position(pos1, False)
    print("Theta0 :", theta0)
    print("Theta1 :", theta1)
    print("Theta2 :", theta2)
    print("Theta3 :", theta3)
    joint_plot(theta0, theta1, theta2, theta3, EE_plot_pos=None)
    # position 2
    pos2 = [200, -100, 300]
    # theta0, theta1, theta2, theta3 = try_EE_position(pos2, True)
    # joint_plot(theta0, theta1, theta2, theta3)
    # position 3
    pos3 = [0, 0, 300]
    theta0, theta1, theta2, theta3 = try_EE_position(pos3, True)
    joint_plot(theta0, theta1, theta2, theta3, EE_plot_pos=None)
    # position 4
    pos4 = [0, 0, 70]
    # theta0, theta1, theta2, theta3 = try_EE_position(pos4, True)
    # joint_plot(theta0, theta1, theta2, theta3)
    # position 5
    pos5 = [0, -150, 50]
    # theta0, theta1, theta2, theta3 = try_EE_position(pos5, True)
    # joint_plot(theta0, theta1, theta2, theta3, EE_plot_pos=pos5)







def trajectory_plotting(traj):

    for i in range(len(traj["time"])):
        theta0 = traj["theta_0"][i]
        theta1 = traj["theta_1"][i]
        theta2 = traj["theta_2"][i]
        theta3 = traj["theta_3"][i]






    return

#trajectory_joint_angles(trajectory_positions())

# try_EE_position([100, -100, 100])
# {'time': [2], 'theta_0': [0.7853981633974483], 'theta_1': [0.15083233291732956], 'theta_2': [-2.40719150909267], 'theta_3': [-1.379926596913827]}
# trajectory = trajectory_joint_angles(trajectory_positions())
# print("Trajectory Dictionary length: ", len(trajectory))
# print(trajectory_joint_angles(trajectory_positions()))


# {'time': [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89, 90, 91, 92, 93, 94, 95, 96],
# 'theta_0': [-0.8760580505981936, -0.8550527371260164, -0.8329812666744316, -0.8097835725701668, -0.7853981633974483, -0.759762754875771, -0.7328151017865068, -0.7044940642422177, -0.6747409422235529, -0.6435011087932843, -0.6107259643892089, -0.5763752205911836, -0.540419500270584, -0.5028432109278609, -0.46364760900080615, -0.4228539261329407, -0.3805063771123649, -0.3366748193867273, -0.2914567944778672, -0.24497866312686423, -0.19739555984988089, -0.14888994760949736, -0.09966865249116208, -0.04995839572194272, 0.0, 0.04995839572194272, 0.09966865249116186, 0.14888994760949714, 0.19739555984988066, 0.244978663126864, 0.291456794477867, 0.33667481938672705, 0.3805063771123649, 0.4228539261329407, 0.46364760900080615, 0.5028432109278607, 0.540419500270584, 0.5763752205911836, 0.6107259643892086, 0.6435011087932844, 0.6747409422235526, 0.7044940642422176, 0.7328151017865066, 0.7597627548757707, 0.7853981633974483, 0.8097835725701668, 0.8329812666744316, 0.8550527371260165, 0.8719394577410081, 0.8507256330207998, 0.8284337764208256, 0.8050034942546529, 0.7803730800666359, 0.7544801838344056, 0.7272626879966902, 0.6986598247214632, 0.6686135679278209, 0.6370703292756835, 0.6039829782529978, 0.5693131911006619, 0.53303411017749, 0.49513326346840403, 0.4556156532112243, 0.41450687458478597, 0.3718560738485812, 0.3277385067805554, 0.28225742198149106, 0.23554498072086338, 0.18776194651359335, 0.13909594148207116, 0.08975817418995047, 0.03997868712328989, -0.009999666686665298, -0.05992815512120786, -0.10955952677394443, -0.15865526218640147, -0.20699219421982118, -0.25436805855326594, -0.30060567004239536, -0.3455555805817121, -0.3890972310552785, -0.4311387407187821, -0.4716155678623277, -0.510488321916776, -0.5477400137159023, -0.5833730069938561, -0.6174058917515728, -0.6498704494119476, -0.6808088289158278, -0.7102710074866865, -0.7383125725172279, -0.7649928327109103, -0.7903732467283024, -0.8145161449044873, -0.8374837126116268, -0.8593372010553888],
# 'theta_1': [0.17500878185123758, 0.1637150655261963, 0.15505974991842542, 0.15372328606991026, 0.14975565813314096, 0.15293623000572043, 0.14774123970936293, 0.1466313914405486, 0.14893875187958727, 0.14726333542544356, 0.14818811110023455, 0.1515295555326146, 0.14280193668400276, 0.12171919684967425, 0.1025204595019098, 0.0851599783142889, 0.06961176385259815, 0.05586461026754774, 0.04391803036092057, 0.03377881560066309, 0.025458065099783767, 0.018968613991351635, 0.014322850144332433, 0.011530944881067207, 0.010599541260236212, 0.011530944881067207, 0.014322850144332433, 0.018968613991351635, 0.025458065099783767, 0.03377881560066309, 0.04391803036092057, 0.05586461026754774, 0.06961176385259815, 0.0851599783142889, 0.1025204595019098, 0.12171919684967425, 0.14280193668400276, 0.1515295555326146, 0.14818811110023455, 0.14726333542544356, 0.14893875187958727, 0.1466313914405486, 0.14774123970936293, 0.15293623000572043, 0.14975565813314096, 0.15372328606991026, 0.15505974991842542, 0.1637150655261963, 0.19060342583333512, 0.20420117751741262, 0.21731954952790744, 0.23535768592069406, 0.1971115723747392, 0.15452230818250623, 0.12241415869555095, 0.09260988773662016, 0.06689377453252021, -0.10957668153615163, -0.17062019974182085, -0.21054610621595327, -0.23547270519927777, -0.2570133797675074, -0.2696944705154234, -0.28076483248824236, -0.28958299320961967, -0.29539344597880124, -0.2994950757541295, -0.30233234763178984, -0.30446950388310334, -0.3064597864032228, -0.30650020563620206, -0.3077152193072932, -0.3079966292812699, -0.3073406065954103, -0.30685369319373823, -0.30541816932544563, -0.30386505460289825, -0.3012956338766868, -0.29799736817917744, -0.2930452902035805, -0.2863641503552521, -0.2765724256702651, -0.26477756020666643, -0.24816831303503495, -0.22489466709263173, -0.19290980954646653, -0.15027540857754662, -0.06815718727168019, 0.08036196269669516, 0.11475355054698211, 0.14092716152107831, 0.1688023833323502, 0.21647936861600536, 0.22810352107027, 0.2122021891306085, 0.2005258093733513],
# 'theta_2': [-2.0115761033531383, -1.7814758291976922, -1.6887483955126819, -1.6142172161387902, -1.5541328749056125, -1.495918815111336, -1.4511884194883184, -1.4074650220690756, -1.364173284157771, -1.3282592495626098, -1.2926051184895726, -1.2569241419740207, -1.2374138325639237, -1.2345154304865482, -1.2319299319315045, -1.2296374272854271, -1.2276217214100276, -1.2258695984990746, -1.2243702845653206, -1.2231150494776881, -1.2220969103115251, -1.2213104103381898, -1.2207514561982724, -1.220417201361883, -1.22030596788635, -1.220417201361883, -1.2207514561982724, -1.2213104103381898, -1.2220969103115251, -1.2231150494776881, -1.2243702845653206, -1.2258695984990746, -1.2276217214100276, -1.2296374272854271, -1.2319299319315045, -1.2345154304865482, -1.2374138325639237, -1.2569241419740207, -1.2926051184895726, -1.3282592495626098, -1.364173284157771, -1.4074650220690756, -1.4511884194883184, -1.495918815111336, -1.5541328749056125, -1.6142172161387902, -1.6887483955126819, -1.7814758291976922, -2.108819604451997, -2.2519265708396703, -2.334464834113567, -2.3967600224130945, -2.4586000978178033, -2.5094540371949052, -2.5511137573374443, -2.586748189568187, -2.6178142752905176, -2.615418389190921, -2.6155456368439904, -2.614616128492458, -2.6167574966515468, -2.613919934970734, -2.617562698375063, -2.6174848558141424, -2.613667491089491, -2.6122833551250633, -2.613703281898818, -2.611818153375514, -2.612983599281849, -2.6173017551452387, -2.6119294961464403, -2.6161826014345166, -2.6171645552614913, -2.614873605741902, -2.615767224824703, -2.613398343605112, -2.6141811151360623, -2.611710088703874, -2.6122841060597874, -2.6156921674388323, -2.61564438318555, -2.617963652558587, -2.6165510900369338, -2.6166954164416327, -2.617470074734589, -2.617586129309072, -2.615333046325433, -2.616382195238447, -2.60599085946021, -2.5730675672671386, -2.5349787226357803, -2.4906053454384285, -2.4347146024895996, -2.3733928406778966, -2.304559682192446, -2.207717715161294],
# 'theta_3': [-1.3837334292869843, -1.3954121803456747, -1.3900225399125816, -1.3926947130299197, -1.3897292676797741, -1.3957823035778787, -1.3894901489583618, -1.388290134570163, -1.3909389883977552, -1.388256122388337, -1.3884333520299772, -1.391000404986692, -1.3802157692479264, -1.3562346273362225, -1.3344503914334143, -1.314797405599716, -1.2972334852626257, -1.2817342087666224, -1.2682883149262412, -1.2568938650783512, -1.2475549754113089, -1.2402790243295414, -1.2350743063426048, -1.2319481462429502, -1.2309055091465861, -1.2319481462429502, -1.2350743063426048, -1.2402790243295414, -1.2475549754113089, -1.2568938650783512, -1.2682883149262412, -1.2817342087666224, -1.2972334852626257, -1.314797405599716, -1.3344503914334143, -1.3562346273362225, -1.3802157692479264, -1.391000404986692, -1.3884333520299772, -1.388256122388337, -1.3909389883977552, -1.388290134570163, -1.3894901489583618, -1.3957823035778787, -1.3897292676797741, -1.3926947130299197, -1.3900225399125816, -1.3954121803456747, -1.3918518192482807, -1.3827502583805704, -1.3824137848052738, -1.3929339394178146, -1.3118081461569089, -1.2240797124820897, -1.1550914667979286, -1.0911084579899673, -1.0353719066883964, -0.6732459930607233, -0.5163366136484355, -0.39694138248302563, -0.3130696278389885, -0.22760486777014444, -0.17493330912669947, -0.12015187329310151, -0.06388311654721446, -0.0217819427737187, 0.007079699007769613, 0.03798203818864898, 0.05640704105357097, 0.0628058139702723, 0.08567178472199322, 0.08263369310500801, 0.08193314925201012, 0.08356807608573957, 0.07346089734129524, 0.06566760843262176, 0.045878429659180764, 0.028326742845214303, -0.0017254789881233634, -0.04499220338265175, -0.08653214401758447, -0.14227636937546695, -0.19629190361727034, -0.26540535475343074, -0.3505401828085919, -0.4524542650090132, -0.5713753993340388, -0.7679891709325506, -1.0631966178021786, -1.1344780835391675, -1.1923760199616784, -1.2544176809153433, -1.3509236783698295, -1.3884925316121075, -1.3822978575267402, -1.3872259121179624]}


