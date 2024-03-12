import numpy as np

theta_ee = 0    #[rad]
z_ee = 100      #[mm]
y_ee = -100      #[mm]

def theta_0(x_global_ee, y_global_ee):
    theta_0 = np.arctan2(y_global_ee/x_global_ee) + np.pi/2
    return theta_0
def vertical_projection(x_global_ee, y_global_ee, z_global_ee, theta_0):
    x_ee = 0
    y_ee = -(x_global_ee/np.cos(np.pi/2 + theta_0))
    z_ee = z_global_ee
    return x_ee, y_ee, z_ee

def joint_angles_projected(y_ee, z_ee, theta_ee):

    L_12 = 95       #[mm]
    L_23 = 105      #[mm]
    L_3ee = 75      #[mm]

    y_3 = y_ee + L_3ee*np.cos(theta_ee)     #[mm]
    z_3 = z_ee - L_3ee*np.sin(theta_ee)     #[mm]
    L_13 = np.sqrt(z_3**2+y_3**2)           #[mm]

    theta_1 = np.arccos((L_12**2+L_13**2-L_23**2)/(2*L_12*L_13)) + np.arcsin(y_3/L_13)     #[rad]

    alpha_2 = np.arccos((L_12**2+L_23**2-L_13**2)/(2*L_12*L_23))     #[rad]
    alpha_3 = np.arccos((L_13**2+L_23**2-L_12**2)/(2*L_13*L_23))     #[rad]

    theta_2 = np.pi - alpha_2     #[rad]
    theta_3 = np.pi/2 + theta_ee - theta_1 - theta_2     #[rad]

    return theta_1, theta_2, theta_3
