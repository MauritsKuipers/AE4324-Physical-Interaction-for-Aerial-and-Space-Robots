import numpy as np
import matplotlib.pyplot as plt
# Change: rotate around z so that link is along x
import sympy as sm


def transform_B20(theta_0, link_B20):
    # first -90° rotation about y axis, then theta_0 about x, I am not sure if we should take the transpose of both...? We now did only for the latter one
    # T01y = np.array([[0,        0,      -1],
    #                  [0,        1,      0],
    #                  [1,        0,      0]]
    #                 )
    #
    # T01Rx = np.array(([1,   0,                  0],
    #                   [0,   np.cos(theta_0),    -np.sin(theta_0)],
    #                   [0,   np.sin(theta_0),    np.cos(theta_0)])
    #                  )
    # TB01R = T01y @ T01Rx
    #
    # T01 = np.vstack(
    #     [T01R, [0, 0, 0]]
    # )
    #
    # T01 = np.hstack(
    #     [T01, [[link_021],
    #            [0],
    #            [0],
    #            [1]]
    #      ]
    # )

    TB0 = np.array([[0,     -np.sin(theta_0),   -np.cos(theta_0),   0],
                    [0,     np.cos(theta_0),    -np.sin(theta_0),   0],
                    [1,     0,                  0,                  link_B20],
                    [0,     0,                  0,                  1]]
                   )
    return TB0


def transform_021(theta_1, link_021):
    # First 90° around y, then theta_1 around x, same here, we only transpose the second matrix
    T01y = np.array([[0,        0,      1],
                     [0,        1,      0],
                     [-1,       0,      0]]
                    )

    T01Rx = np.array(([1,       0,      0],
                     [0,        np.cos(theta_1),    -np.sin(theta_1)],
                     [0,        np.sin(theta_1),    np.cos(theta_1)])
                    )
    T01R = T01y @ T01Rx

    # add position:
    T01 = np.vstack(
        [T01R, [0, 0, 0]]
    )

    T01 = np.hstack(
        [T01,  [[link_021],
                [0],
                [0],
                [1]]
         ]
    )
    return T01


def transform_122(theta_2, link_122):
    # rotate about y axis with theta_2

    T12 = np.array([[1,     0,                  0,                  0],
                    [0,     np.cos(theta_2),      -np.sin(theta_2),     0],
                    [0,     np.sin(theta_2),      np.cos(theta_2),      link_122],
                    [0,     0,                  0,                  1]]
                   )
    return T12


def transform_223(theta_3, link_223):
    # rotate about y axis with theta_3

    T23 = np.array([[1,     0,                  0,                  0],
                    [0,     np.cos(theta_3),    -np.sin(theta_3),   0],
                    [0,     np.sin(theta_3),    np.cos(theta_3),    link_223],
                    [0,     0,                  0,                  1]]
                   )
    return T23


def transform_32EE(theta_4, link_32EE):
    T34 = np.array([[1,     0,                  0,                  0],
                    [0,     np.cos(theta_4),    -np.sin(theta_4),   0],
                    [0,     np.sin(theta_4),    np.cos(theta_4),    link_32EE],
                    [0,     0,                  0,                  1]]
                   )
    return T34
def transform_ee2ground(theta_0, theta_1, theta_2, theta_3):
    Tee2ground = transform_B20(theta_0, 45)@ transform_021(theta_1, 20) @ transform_122(theta_2, 95) @ transform_223(theta_3, 105) @ transform_32EE(theta_3, 75)
    print(Tee2ground)
    return Tee2ground

transform_ee2ground(5,5,5,5)
def jacobian(theta_0, theta_1, theta_2, theta_3, L12, L23, L3ee):
    jacobian = np.array([[(L12*np.sin(theta_1) + L23*np.sin(theta_1 + theta_2) + L3ee*np.sin(theta_1 + theta_2 + theta_3))*np.cos(theta_0) ,
                          (L12*np.cos(theta_1) + L23*np.cos(theta_1 + theta_2) + L3ee*np.cos(theta_1 + theta_2 + theta_3))*np.sin(theta_0),
                          (L23*np.cos(theta_1+theta_2) + L3ee*np.cos(theta_1+theta_2+theta_3))*np.sin(theta_0) ,
                          (L3ee*np.cos(theta_1+theta_2+theta_3))*np.sin(theta_0)],
                         [(L12*np.sin(theta_1) + L23*np.sin(theta_1+theta_2) + L3ee*np.sin(theta_1+theta_2+theta_3))*np.sin(theta_0) ,
                          (L12*np.cos(theta_1) + L23*np.cos(theta_1+theta_2) + L3ee*np.cos(theta_1+theta_2+theta_3))*np.cos(theta_0) ,
                          -(L23*np.cos(theta_1+theta_2) + L3ee*np.cos(theta_1+theta_2+theta_3))*np.cos(theta_0) ,
                          -(L3ee*np.cos(theta_1+theta_2+theta_3))*np.cos(theta_0)],
                         [0 ,
                          -L12*np.sin(theta_1) - L23*np.sin(theta_1+theta_2) - L3ee*np.sin(theta_1+theta_2+theta_3),
                          -L23*np.sin(theta_1+theta_2) - L3ee*np.sin(theta_1+theta_2+theta_3) ,
                          - L3ee*np.sin(theta_1+theta_2+theta_3)]])
    return jacobian

def sm_jacobian(theta_0, theta_1, theta_2, theta_3, L12, L23, L3ee):
    jacobian = sm.Matrix([[(L12*sm.sin(theta_1) + L23*sm.sin(theta_1 + theta_2) + L3ee*sm.sin(theta_1 + theta_2 + theta_3))*sm.cos(theta_0) ,
                          (L12*sm.cos(theta_1) + L23*sm.cos(theta_1 + theta_2) + L3ee*sm.cos(theta_1 + theta_2 + theta_3))*sm.sin(theta_0),
                          (L23*sm.cos(theta_1+theta_2) + L3ee*sm.cos(theta_1+theta_2+theta_3))*sm.sin(theta_0) ,
                          (L3ee*sm.cos(theta_1+theta_2+theta_3))*sm.sin(theta_0)],
                         [(L12*sm.sin(theta_1) + L23*sm.sin(theta_1+theta_2) + L3ee*sm.sin(theta_1+theta_2+theta_3))*sm.sin(theta_0) ,
                          (L12*sm.cos(theta_1) + L23*sm.cos(theta_1+theta_2) + L3ee*sm.cos(theta_1+theta_2+theta_3))*sm.cos(theta_0) ,
                          -(L23*sm.cos(theta_1+theta_2) + L3ee*sm.cos(theta_1+theta_2+theta_3))*sm.cos(theta_0) ,
                          -(L3ee*sm.cos(theta_1+theta_2+theta_3))*sm.cos(theta_0)],
                         [0 ,
                          -L12*sm.sin(theta_1) - L23*sm.sin(theta_1+theta_2) - L3ee*sm.sin(theta_1+theta_2+theta_3),
                          -L23*sm.sin(theta_1+theta_2) - L3ee*sm.sin(theta_1+theta_2+theta_3) ,
                          - L3ee*sm.sin(theta_1+theta_2+theta_3)]])
    print("Jacobian:\n", jacobian)
    jacobian_transpose = sm.Transpose(jacobian)
    print("Jacobian transpose:\n", jacobian_transpose)
    inv_jacobian = sm.Inverse(sm.Transpose(jacobian)*jacobian)*sm.Transpose(jacobian)
    return jacobian, inv_jacobian