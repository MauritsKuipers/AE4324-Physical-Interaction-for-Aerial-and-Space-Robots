import numpy as np
from Joint_plot import joint_plot
def vertical_projection(x_global_ee, y_global_ee, z_global_ee):
    theta_0 = np.arctan2(y_global_ee,x_global_ee) + np.deg2rad(90)
    x_ee = -np.sqrt(x_global_ee**2 + y_global_ee**2)
    y_ee = 0
    z_ee = z_global_ee - 45 - 20
    return theta_0, x_ee, y_ee, z_ee

def joint_angles_projection(x_ee, z_ee, theta_ee):

    L_12 = 95       #[mm]
    L_23 = 105      #[mm]
    L_3ee = 75      #[mm]

    x_3 = x_ee + L_3ee*np.cos(theta_ee)     #[mm]
    z_3 = z_ee + L_3ee*np.sin(theta_ee)     #[mm]
    L_13 = np.sqrt(z_3**2+x_3**2)           #[mm]

    theta_1 = np.arccos((L_12 ** 2 + L_13 ** 2 - L_23 ** 2) / (2 * L_12 * L_13)) + np.arctan2(z_3, x_3) - np.pi/2

    alpha_2 = np.arccos((L_12**2+L_23**2-L_13**2)/(2*L_12*L_23))     #[rad]
#   alpha_3 = np.arccos((L_13**2+L_23**2-L_12**2)/(2*L_13*L_23))     #[rad]

    theta_2 = -(np.pi - alpha_2)     #[rad]
    theta_3 = np.pi/2 + theta_ee - theta_1 - theta_2     #[rad]

    return theta_1, theta_2, theta_3
"""
x = 0
y = -95*np.sin(np.pi/4)
z = 65+95*np.sin(np.pi/4)+105+75
thetaee = np.deg2rad(-90)
"""
"""
x = 0 #-y
y = -105*np.sin(np.pi/4)#x
z = 65+95+105*np.cos(np.pi/4) + 75#-z
thetaee = np.deg2rad(-45)
"""
"""
x = -242.7081528017131
z = 74.14213562373097
y = 0
thetaee = np.deg2rad(45)
"""
x=100
y=-100
z=100
thetaee = np.deg2rad(-90)

theta_0, x_ee, y_ee, z_ee = vertical_projection(x,y,z)
theta_1, theta_2, theta_3 = joint_angles_projection(x_ee, z_ee, thetaee)

joint_plot(theta_0, theta_1, theta_2, theta_3, (x,y,z))
