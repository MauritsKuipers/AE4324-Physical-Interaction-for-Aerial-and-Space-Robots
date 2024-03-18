import numpy as np
from Transformation import transform_ee2ground

def inverse_jacobian(theta_0: float, theta_1: float, theta_2: float, theta_3: float):
    L12 = 95  # [mm]
    L23 = 105  # [mm]
    L3ee = 75  # [mm]

    jacobian = np.matrix([[(L12*np.sin(theta_1) + L23*np.sin(theta_1 + theta_2) + L3ee*np.sin(theta_1 + theta_2 + theta_3))*np.cos(theta_0) ,
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
    print("Jacobian:\n", jacobian)
    print("Jacobian Mult Det:\n", np.linalg.det(np.transpose(jacobian)*jacobian))
    inv_jacobian = np.linalg.inv(np.transpose(jacobian)@jacobian)@np.transpose(jacobian)
    print("Inv transpose:\n", inv_jacobian)
    return inv_jacobian

def constant_speed_trajectory(velocity: np.ndarray, initial_state: dict):
    dt = 0.01
    track = {
        "time": [0],
        "theta_0": [initial_state["theta_0"]],
        "theta_1": [initial_state["theta_1"]],
        "theta_2": [initial_state["theta_2"]],
        "theta_3": [initial_state["theta_3"]]
    }

    for t in np.arange(dt, 3, dt):
        theta_0 = track["theta_0"][-1]
        theta_1 = track["theta_1"][-1]
        theta_2 = track["theta_2"][-1]
        theta_3 = track["theta_3"][-1]
        print("theta_1:\n", track["theta_1"])
        joint_velocities = inverse_jacobian(theta_0, theta_1, theta_2, theta_3) * velocity
        print("Joint velocities:\n", joint_velocities)
        joint_velocities.T
        track["time"].append(t)
        track["theta_0"].append(theta_0 + float(joint_velocities[0,0]) * dt)
        track["theta_1"].append(theta_1 + float(joint_velocities[1,0]) * dt)
        track["theta_2"].append(theta_2 + float(joint_velocities[2,0]) * dt)
        track["theta_3"].append(theta_3 + float(joint_velocities[3,0]) * dt)

        

    return track


if __name__ == "__main__":
    initial_state =     {
        "theta_0": np.deg2rad(10),
        "theta_1": np.deg2rad(15),
        "theta_2": np.deg2rad(135),
        "theta_3": np.deg2rad(-48)
    }

    velocity = np.array([[0], [0], [10]])

    print(constant_speed_trajectory(velocity, initial_state))