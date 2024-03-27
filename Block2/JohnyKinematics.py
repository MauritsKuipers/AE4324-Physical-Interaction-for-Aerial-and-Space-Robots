import numpy as np
from math import cos, sin, pi, sqrt, acos, asin, atan

def rotation_around_z_matrix(theta):
    '''Rotation matrix arround the z axis'''
    return np.array([
        [cos(theta) , -sin(theta)   , 0],
        [sin(theta) , cos(theta)    , 0],
        [0          , 0             , 1]
    ])

def homogenouse_transformation_matrix_z_rotation(theta, dx, dy, dz):
    '''
    The homogenous transformation function with a rotation in the Z axis.
    - Theta (rad) is the rotation angle
    - Dx, Dy, Dz are the linear displacements in each direction
    '''
    return np.array([
        [cos(theta) , -sin(theta)   , 0, dx],
        [sin(theta) , cos(theta)    , 0, dy],
        [0          , 0             , 1, dz],
        [0          , 0             , 0, 1 ]
    ])

def homogenouse_transformation_matrix_x_rotation(theta, dx, dy, dz):
    '''
    The homogenous transformation function with a rotation in the Z axis.
    - Theta (rad) is the rotation angle
    - Dx, Dy, Dz are the linear displacements in each direction
    '''
    return np.array([
        [1 , 0             , 0         , dx],
        [0 , cos(theta)    , sin(theta), dy],
        [0 , -sin(theta)   , cos(theta), dz],
        [0 , 0             , 0         , 1 ]
    ])

class Johny:
    def __init__(self) -> None:
        
        self.joint_limits = {
            "J0": {"min": np.deg2rad(-90), "max": np.deg2rad(100)},
            "J1": {"min": np.deg2rad(-45), "max": np.deg2rad(70)},
            "J2": {"min": np.deg2rad(15), "max": np.deg2rad(150)},
            "J3": {"min": np.deg2rad(-90), "max": np.deg2rad(90)},
            "EE": {"min": 0, "max": 1}
        }

        # set angeles to home position
        self.ang0 = 0
        self.ang1 = 0
        self.ang2 = np.deg2rad(90)
        self.ang3 = 0

    def AB_to_ground_matrix(self):
        return homogenouse_transformation_matrix_x_rotation(np.deg2rad(90), 0, 0, 0)@homogenouse_transformation_matrix_z_rotation(np.deg2rad(-90), 0, 0, 0)

    def J0_to_AB_matrix(self):
        return homogenouse_transformation_matrix_x_rotation(self.ang0, 0.05, 0, 0)
    
    def J1_to_J0_matrix(self):
        return homogenouse_transformation_matrix_z_rotation(self.ang1, 0.02, 0.01, 0)
    
    def J2_to_J1_matrix(self):
        return homogenouse_transformation_matrix_z_rotation(self.ang2, 0.095, 0, 0)
    
    def J3_to_J2_matrix(self):
        return homogenouse_transformation_matrix_z_rotation(self.ang3, 0.1025, 0, 0)
    
    def EE_to_J3_matrix(self):
        return np.array([
            [1 , 0 , 0 , 0.075],
            [0 , 1 , 0 , 0    ],
            [0 , 0 , 1 , 0    ],
            [0 , 0 , 0 , 1    ]
        ])
    
    def J0_to_ground_transform(self, position_vector):
        position_vector = np.append(position_vector, [1])
        return (
                self.AB_to_ground_matrix()
                @self.J0_to_AB_matrix()
                @position_vector
                )[0:3]
    
    def J1_to_ground_transform(self, position_vector):
        position_vector = np.append(position_vector, [1])
        return (
                self.AB_to_ground_matrix()
                @self.J0_to_AB_matrix()
                @self.J1_to_J0_matrix()
                @position_vector
                )[0:3]
    
    def J2_to_ground_transform(self, position_vector):
        position_vector = np.append(position_vector, [1])
        return (
                self.AB_to_ground_matrix()
                @self.J0_to_AB_matrix()
                @self.J1_to_J0_matrix()
                @self.J2_to_J1_matrix()
                @position_vector
                )[0:3]
    
    def J3_to_ground_transform(self, position_vector):
        position_vector = np.append(position_vector, [1])
        return (
                self.AB_to_ground_matrix()
                @self.J0_to_AB_matrix()
                @self.J1_to_J0_matrix()
                @self.J2_to_J1_matrix()
                @self.J3_to_J2_matrix()
                @position_vector
                )[0:3]
    
    def EE_to_ground_transform(self, position_vector):
        position_vector = np.append(position_vector, [1])
        return (
                self.AB_to_ground_matrix()
                @self.J0_to_AB_matrix()
                @self.J1_to_J0_matrix()
                @self.J2_to_J1_matrix()
                @self.J3_to_J2_matrix()
                @self.EE_to_J3_matrix()
                @position_vector
                )[0:3]

    def get_EE_in_ground_vector(self):
        l01y = 0.0
        l01x = 0.01
        l12 = 0.095
        l23 = 0.1025
        l3ee = 0.075
        lab0 = 0.05
        t0 = self.ang0
        t1 = self.ang1
        t2 = self.ang2
        t3 = self.ang3

        pos = np.array([
            [-(l01y + l12*sin(t1) + l23*sin(t1 + t2) + l3ee*sin(t1 + t2 + t3))*cos(t0)], 
            [-(l01y + l12*sin(t1) + l23*sin(t1 + t2) + l3ee*sin(t1 + t2 + t3))*sin(t0)], 
            [l01x + l12*cos(t1) + l23*cos(t1 + t2) + l3ee*cos(t1 + t2 + t3) + lab0]
            ])
        
        return {"x": pos[0], "y": pos[1], "z": pos[2]}
    
    def get_J3_in_ground_vector(self):
        l01y = 0.0
        l01x = 0.01
        l12 = 0.095
        l23 = 0.1025
        l3ee = 0.075
        lab0 = 0.05
        t0 = self.ang0
        t1 = self.ang1
        t2 = self.ang2
        t3 = self.ang3

        pos = np.array([
            [-(l01y + l12*sin(t1) + l23*sin(t1 + t2))*cos(t0)],
            [-(l01y + l12*sin(t1) + l23*sin(t1 + t2))*sin(t0)],
            [l01x + l12*cos(t1) + l23*cos(t1 + t2) + lab0]
            ])
        
        return {"x": pos[0], "y": pos[1], "z": pos[2]}
    
    def get_J2_in_ground_vector(self):
        l01y = 0.0
        l01x = 0.01
        l12 = 0.095
        l23 = 0.1025
        l3ee = 0.075
        lab0 = 0.05
        t0 = self.ang0
        t1 = self.ang1
        t2 = self.ang2
        t3 = self.ang3

        pos = np.array([
            [-(l01y + l12*sin(t1))*cos(t0)], 
            [-(l01y + l12*sin(t1))*sin(t0)],
            [l01x + l12*cos(t1) + lab0]
            ])
        
        return {"x": pos[0], "y": pos[1], "z": pos[2]}
    
    def get_J1_in_ground_vector(self):
        l01y = 0.0
        l01x = 0.01
        l12 = 0.095
        l23 = 0.1025
        l3ee = 0.075
        lab0 = 0.05
        t0 = self.ang0
        t1 = self.ang1
        t2 = self.ang2
        t3 = self.ang3

        pos = np.array([
            [-l01y*cos(t0)], 
            [-l01y*sin(t0)],
            [l01x + lab0]
            ])
        
        return {"x": pos[0], "y": pos[1], "z": pos[2]}
    
    def get_frame_origins_in_ground_frame(self):
        """
        Returns the coordinates of all joints and the EE in the ground reference frame, 
        x is positive toward the back of the robot, y is positive towards the top
        """

        joints = dict()
        org_vec = np.array([0., 0., 0.]).transpose()


        [x, y, z] = self.J0_to_ground_transform(org_vec)
        joints["J0"] = {"x": x, "y": y, "z": z}

        [x, y, z] = self.J1_to_ground_transform(org_vec)
        joints["J1"] = {"x": x, "y": y, "z": z}

        [x, y, z] = self.J2_to_ground_transform(org_vec)
        joints["J2"] = {"x": x, "y": y, "z": z}

        [x, y, z] = self.J3_to_ground_transform(org_vec)
        joints["J3"] = {"x": x, "y": y, "z": z}

        [x, y, z] = self.EE_to_ground_transform(org_vec)
        joints["EE"] = {"x": x, "y": y, "z": z}

        return joints
    
    def sweep_over_ang0(self):
        hist = {
            "J0": {"x": [], "y": [], "z": []},
            "J1": {"x": [], "y": [], "z": []},
            "J2": {"x": [], "y": [], "z": []},
            "J3": {"x": [], "y": [], "z": []},
            "EE": {"x": [], "y": [], "z": []},
        }
        
        for ang2 in np.arange(self.joint_limits["J2"]["min"], self.joint_limits["J2"]["max"], 0.1):
            self.ang2 = ang2
            for ang1 in np.arange(self.joint_limits["J1"]["min"], self.joint_limits["J1"]["max"], 0.1):
                self.ang1 = ang1
                for ang3 in np.arange(self.joint_limits["J3"]["min"], self.joint_limits["J3"]["max"], 0.1):
                    self.ang3 = ang3
                    locations = self.get_frame_origins_in_ground_frame()
                    for key, location in locations.items():
                        hist[key]["x"].append(location["x"])
                        hist[key]["y"].append(location["y"])
                        hist[key]["z"].append(location["z"])

        return hist
    
    def sweep_over_all(self):
        hist = {
            "J0": {"x": [], "y": [], "z": []},
            "J1": {"x": [], "y": [], "z": []},
            "J2": {"x": [], "y": [], "z": []},
            "J3": {"x": [], "y": [], "z": []},
            "EE": {"x": [], "y": [], "z": []},
        }
        
        for ang0 in np.arange(self.joint_limits["J0"]["min"], self.joint_limits["J0"]["max"], 2*pi / 40):
            self.ang0 = ang0
            for ang1 in np.arange(self.joint_limits["J1"]["min"], self.joint_limits["J1"]["max"], 2*pi / 20):
                self.ang1 = ang1
                for ang2 in np.arange(self.joint_limits["J2"]["min"], self.joint_limits["J2"]["max"], 2*pi / 10):
                    self.ang2 = ang2
                    for ang3 in np.arange(self.joint_limits["J3"]["min"], self.joint_limits["J3"]["max"], 2*pi / 10):
                        self.ang3 = ang3
                        locations = self.get_frame_origins_in_ground_frame()
                        for key, location in locations.items():
                            hist[key]["x"].append(location["x"])
                            hist[key]["y"].append(location["y"])
                            hist[key]["z"].append(location["z"])

        return hist

    def EE_workspace(self):
        hist_max = {
            "J0": {"x": [], "y": [], "z": []},
            "J1": {"x": [], "y": [], "z": []},
            "J2": {"x": [], "y": [], "z": []},
            "J3": {"x": [], "y": [], "z": []},
            "EE": {"x": [], "y": [], "z": []},
        }

        hist_min = {
            "J0": {"x": [], "y": [], "z": []},
            "J1": {"x": [], "y": [], "z": []},
            "J2": {"x": [], "y": [], "z": []},
            "J3": {"x": [], "y": [], "z": []},
            "EE": {"x": [], "y": [], "z": []},
        }

        #### Getting the data for maximum ####

        for ang0 in np.arange(self.joint_limits["J0"]["min"], self.joint_limits["J0"]["max"], 2*pi / 40):
            self.ang0 = ang0
            self.ang1 = self.joint_limits["J1"]["min"]
            self.ang2 = self.joint_limits["J2"]["min"]
            self.ang3 = self.joint_limits["J3"]['min']

            for ang3 in np.arange(self.joint_limits["J3"]["min"], self.joint_limits["J3"]["max"], 2*pi / 40):
                self.ang3 = ang3
                locations = self.get_frame_origins_in_ground_frame()
                for key, location in locations.items():
                    hist_max[key]["x"].append(location["x"])
                    hist_max[key]["y"].append(location["y"])
                    hist_max[key]["z"].append(location["z"])
            # self.ang3 = self.joint_limits["J3"]['min']
            for ang1 in np.arange(self.joint_limits["J1"]["min"], self.joint_limits["J1"]["max"], 2 * pi / 40):
                self.ang1 = ang1
                locations = self.get_frame_origins_in_ground_frame()
                for key, location in locations.items():
                    hist_max[key]["x"].append(location["x"])
                    hist_max[key]["y"].append(location["y"])
                    hist_max[key]["z"].append(location["z"])
            # self.ang1 = self.joint_limits["J1"]["min"]
            for ang2 in np.arange(self.joint_limits["J2"]["min"], self.joint_limits["J2"]["max"], 2 * pi / 40):
                self.ang2 = ang2
                locations = self.get_frame_origins_in_ground_frame()
                for key, location in locations.items():
                    hist_max[key]["x"].append(location["x"])
                    hist_max[key]["y"].append(location["y"])
                    hist_max[key]["z"].append(location["z"])
            # self.ang2 = self.joint_limits["J2"]["min"]
            for ang3 in np.arange(self.joint_limits["J3"]["min"], self.joint_limits["J3"]["max"], 2 * pi / 40):
                self.ang3 = ang3
                locations = self.get_frame_origins_in_ground_frame()
                for key, location in locations.items():
                    hist_max[key]["x"].append(location["x"])
                    hist_max[key]["y"].append(location["y"])
                    hist_max[key]["z"].append(location["z"])
            # self.ang3 = self.joint_limits["J3"]["min"]

        #### Getting the data for minimum ####

        for ang0 in np.arange(self.joint_limits["J0"]["min"], self.joint_limits["J0"]["max"], 2*pi / 5):
            self.ang0 = ang0
        return hist_max, hist_min
    
    def symbolic_fk_test(self):
        l01y = 0.0
        l01x = 0.01
        l12 = 0.095
        l23 = 0.1025
        l3ee = 0.075
        lab0 = 0.05
        t0 = self.ang0
        t1 = self.ang1
        t2 = self.ang2
        t3 = self.ang3
        joints = {}

        # Transform arm base to ground
        tg_ab =  np.array([[0, -1, 0, 0], [0, 0, -1, 0], [1, 0, 0, 0], [0, 0, 0, 1]])
        # Transform joint 0 to arm base
        tab_0 =  np.array([[1, 0, 0, lab0], [0, cos(t0), -sin(t0), 0], [0, sin(t0), cos(t0), 0], [0, 0, 0, 1]])
        # Transform joint 1 to joint 0
        t0_1 =  np.array([[cos(t1), -sin(t1), 0, l01x], [sin(t1), cos(t1), 0, l01y], [0, 0, 1, 0], [0, 0, 0, 1]])
        # Transform joint 2 to joint 1
        t2_1 =  np.array([[cos(t2), -sin(t2), 0, l12], [sin(t2), cos(t2), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        # Transform joint 3 to joint 2
        t3_2 =  np.array([[cos(t3), -sin(t3), 0, l23], [sin(t3), cos(t3), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        # Transform EE to joint 3
        tee_3 =  np.array([[1, 0, 0, l3ee], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        #---
        # EE position in ground frame:
        x = (l01y + l12*sin(t1) + l23*sin(t1 + t2) + l3ee*sin(t1 + t2 + t3))*cos(t0)
        y = (l01y + l12*sin(t1) + l23*sin(t1 + t2) + l3ee*sin(t1 + t2 + t3))*sin(t0)
        z = l01x + l12*cos(t1) + l23*cos(t1 + t2) + l3ee*cos(t1 + t2 + t3) + lab0
        joints["EE"] = {"x": x, "y": y, "z": z}
        #---
        # J3 position in ground frame:
        x = (l01y + l12*sin(t1) + l23*sin(t1 + t2))*cos(t0)
        y = (l01y + l12*sin(t1) + l23*sin(t1 + t2))*sin(t0)
        z = l01x + l12*cos(t1) + l23*cos(t1 + t2) + lab0
        joints["J3"] = {"x": x, "y": y, "z": z}
        #---
        # J2 position in ground frame:
        x = (l01y + l12*sin(t1))*cos(t0)
        y = (l01y + l12*sin(t1))*sin(t0)
        z = l01x + l12*cos(t1) + lab0
        joints["J2"] = {"x": x, "y": y, "z": z}
        #---
        # J1 position in ground frame:
        x = l01y*cos(t0)
        y = l01y*sin(t0)
        z = l01x + lab0
        joints["J1"] = {"x": x, "y": y, "z": z}
        #---
        # J0 position in ground frame:
        x = 0
        y = 0
        z = lab0
        joints["J0"] = {"x": x, "y": y, "z": z}
        #---

        return joints
    
    def manual_ik(self, xt, yt, zt, phi):
        '''
        Set the joint angles such that the EE reaches the points specified
        '''
        self.ang0 = atan(yt / xt)
        x_proj = sqrt(xt**2 + yt**2)
        y_proj = zt
        l01y = 0.0
        l01x = 0.01
        l12 = 0.095
        l23 = 0.1025
        l3ee = 0.075
        lab0 = 0.05

        print("---\nIK ee pos:\n", x_proj, y_proj)
        # Calcuate prerequisites
        x_3 = x_proj + cos(phi) * l3ee
        y_3 = y_proj + sin(phi) * l3ee
        print("---\nIK J3 pos:\n", x_3, y_3)
        l3ee_ik = sqrt((x_3 - x_proj)**2 + (y_3 - y_proj)**2)
        print("---\nIK l3ee error pos:\n",l3ee_ik - l3ee)
        x_1 = l01y
        y_1 = lab0 + l01x
        print("---\nIK J1 pos:\n", x_1, y_1)
        l13 = sqrt((x_3 - x_1)**2 + (y_3 - y_1)**2)
        print("---\nIK l13 len:\n", l13)

        # Calculate angles
        self.ang1 = pi \
                    -acos((l12**2 + l13**2 - l23**2) / (2 * l12 * l13)) \
                    -atan((x_3 - x_1) / (y_3 - y_1))
        self.ang2 = pi - acos((l12**2 + l23**2 - l13**2) / (2 * l12 * l23))
        self.ang3 = pi / 2 + phi \
                    - asin((x_3 - x_1) / l13) \
                    - acos((l23**2 + l13**2 - l12**2) / (2 * l23 * l13))
        
    def get_angles(self) -> list:
        return [self.ang0, self.ang1, self.ang2, self.ang3, self.get_phi()]
    
    def get_phi(self) -> float:
        locations = self.symbolic_fk_test()
        j3 = locations["J3"]
        ee = locations["EE"]
        return -atan((j3['z'] - ee['z']) / 
                    (sqrt(j3['x']**2 + j3['y']**2) \
                     - sqrt(ee['x']**2 + ee['y']**2)))