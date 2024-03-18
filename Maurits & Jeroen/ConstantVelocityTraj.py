import numpy as np


def inverse_jacobian(theta_0: float, theta_1: float, theta_2: float, theta_3: float):
    inverse_jac = np.zeros(3, 4)
    return inverse_jac

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
        joint_velocities = inverse_jacobian(theta_0, theta_1, theta_2, theta_3) * velocity

        track["time"].append(t)
        track["theta_0"].append(theta_0 + joint_velocities[0] * dt)
        track["theta_1"].append(theta_1 + joint_velocities[1] * dt)
        track["theta_2"].append(theta_2 + joint_velocities[2] * dt)
        track["theta_3"].append(theta_3 + joint_velocities[3] * dt)

    return track