import numpy as np
from math import cos, sin, pi

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
        
        for ang0 in np.arange(self.joint_limits["J0"]["min"], self.joint_limits["J0"]["max"], 2*pi / 10):
            self.ang0 = ang0
            for ang1 in np.arange(self.joint_limits["J1"]["min"], self.joint_limits["J1"]["max"], 2*pi / 10):
                self.ang1 = ang1
                for ang2 in np.arange(self.joint_limits["J2"]["min"], self.joint_limits["J2"]["max"], 2*pi / 20):
                    self.ang0 = ang2
                    for ang3 in np.arange(self.joint_limits["J3"]["min"], self.joint_limits["J3"]["max"], 2*pi / 100):
                        self.ang3 = ang3
                        locations = self.get_frame_origins_in_ground_frame()
                        for key, location in locations.items():
                            hist[key]["x"].append(location["x"])
                            hist[key]["y"].append(location["y"])
                            hist[key]["z"].append(location["z"])

        return hist
        
