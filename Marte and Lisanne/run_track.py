import rclpy
import numpy as np
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class ExampleTraj(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        self._HOME = [np.deg2rad(90), np.deg2rad(130),
                     np.deg2rad(150), np.deg2rad(60)]
        self._beginning = self.get_clock().now()
        self._publisher = self.create_publisher(JointTrajectory, 'joint_cmds', 10)
        timer_period = 0.06  # seconds
        self._timer = self.create_timer(timer_period, self.timer_callback)
        self.track = {'time': [2], 'theta_0': [0.7853981633974483], 'theta_1': [0.15083233291732956], 'theta_2': [2.40719150909267], 'theta_3': [-1.379926596913827]}
        self.offset_angles = [
            np.deg2rad(90),
            np.deg2rad(90),
            np.deg2rad(10),
            np.deg2rad(90),
            0.
        ]

    def timer_callback(self):
        now = self.get_clock().now()
        msg = JointTrajectory()
        msg.header.stamp = now.to_msg()

        dt = (now - self._beginning).nanoseconds * (1e-9)
        
        point = JointTrajectoryPoint()
        
        user_positions = self.get_current_target_position(dt, self.track)
        print(list(np.add(user_positions + [0.], self.offset_angles)))
        point.positions = list(np.add(user_positions + [0.], self.offset_angles))

        msg.points = [point]

        self._publisher.publish(msg)

    @staticmethod
    def get_current_target_position(time: float, trajectory: dict) -> list:
        '''
        Get the joint angles at a time t in seconds (float) from a specified track.
        The track must consist of a dictionary containing lists of floats with keys
        time, theta_0, theta_1, theta_2 and theta_3.
        '''
        if time < 0:
            raise ValueError("Time code input to get_current_target_position is negative")
        if time > max(trajectory['time']):
            print(
                Warning("Time code input to get_current_target_position" \
                        "is bigger than max track time, taking last value")
            )
            positions = [
                trajectory['theta_0'][-1],
                trajectory['theta_1'][-1],
                trajectory['theta_2'][-1],
                trajectory['theta_3'][-1],
            ]
        else:
            positions = [
                np.interp(time, trajectory['time'], trajectory['theta_0']),
                np.interp(time, trajectory['time'], trajectory['theta_1']),
                np.interp(time, trajectory['time'], trajectory['theta_2']),
                np.interp(time, trajectory['time'], trajectory['theta_3'])
            ]
        return positions
        

def main(args=None):
    rclpy.init(args=args)

    example_traj = ExampleTraj()

    rclpy.spin(example_traj)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    example_traj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
