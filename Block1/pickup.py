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
        self.z = 0

    def timer_callback(self):
        now = self.get_clock().now()
        msg = JointTrajectory()
        msg.header.stamp = now.to_msg()

        dt = (now - self._beginning).nanoseconds * (1e-9)
        
        point = JointTrajectoryPoint()
        cycler = np.sin(dt*2);
        t_target = 3.0
        t_hold = 4.0
        t_drop = 5.0
        
        if dt <= t_target and self.z == 0:
	        point.positions = [self._HOME[0] + np.deg2rad(0) * dt/t_target,
		self._HOME[1] - np.deg2rad(70) * dt/t_target,
		self._HOME[2] - np.deg2rad(60) * dt/t_target,
		self._HOME[3] - np.deg2rad(30) * dt/t_target,
		0.0
		]
        elif dt <= t_hold and self.z == 0:
	        point.positions = [self._HOME[0] + np.deg2rad(0),
		self._HOME[1] - np.deg2rad(70),
		self._HOME[2] - np.deg2rad(60),
		self._HOME[3] - np.deg2rad(30),
		1.0
		]
        elif dt <= t_drop and self.z == 0:
	        point.positions = [self._HOME[0] + np.deg2rad(0),
		self._HOME[1] - np.deg2rad(70),
		self._HOME[2] - np.deg2rad(60) - np.deg2rad(45) * (dt - t_hold)/(t_drop - t_hold),
		self._HOME[3] - np.deg2rad(30) + np.deg2rad(45) * (dt - t_hold)/(t_drop - t_hold),
		1.0
		]
        elif self.z == 0:
	        point.positions = [self._HOME[0] + np.deg2rad(0),
		self._HOME[1] - np.deg2rad(70),
		self._HOME[2] - np.deg2rad(60) - np.deg2rad(45),
		self._HOME[3] - np.deg2rad(30) + np.deg2rad(45),
		0.0
		]
	        self._beginning = self.get_clock().now()
	        self.z = 1
	        print(self.z)
        elif dt <= t_target and self.z == 1:
	        point.positions = [self._HOME[0] + np.deg2rad(30) * dt/t_target,
		self._HOME[1] - np.deg2rad(70) * dt/t_target,
		self._HOME[2] - np.deg2rad(60) * dt/t_target,
		self._HOME[3] - np.deg2rad(30) * dt/t_target,
		1.0
		]
        elif dt <= t_hold and self.z == 1:
	        point.positions = [self._HOME[0] + np.deg2rad(30),
		self._HOME[1] - np.deg2rad(70),
		self._HOME[2] - np.deg2rad(60),
		self._HOME[3] - np.deg2rad(30),
		0.0
		]
        elif dt <= t_drop and self.z == 1:
	        point.positions = [self._HOME[0] + np.deg2rad(30),
		self._HOME[1] - np.deg2rad(70),
		self._HOME[2] - np.deg2rad(60) - np.deg2rad(45) * (dt - t_hold)/(t_drop - t_hold),
		self._HOME[3] - np.deg2rad(30) + np.deg2rad(45) * (dt - t_hold)/(t_drop - t_hold),
		0.0
		]
        elif self.z == 1:
	        point.positions = [self._HOME[0] + np.deg2rad(30),
		self._HOME[1] - np.deg2rad(70),
		self._HOME[2] - np.deg2rad(60) - np.deg2rad(45),
		self._HOME[3] - np.deg2rad(30) + np.deg2rad(45),
		1.0
		]
	        self._beginning = self.get_clock().now()
	        self.z = 0
        
        
        #if cycler > 0:
        #	point.positions = [self._HOME[0],
        #                   self._HOME[1]- np.deg2rad(20),
        #                   self._HOME[2],
        #                   self._HOME[3],
        #                   0.5
        #                   ]
        #else:
        #	point.positions = [
        #		   self._HOME[0],
        #                   self._HOME[1] - np.deg2rad(20),
        #                   self._HOME[2] - np.deg2rad(20),
        #                   self._HOME[3] + np.deg2rad(20),
        #                   0.5
        #                   ]
        msg.points = [point]

        self._publisher.publish(msg)
        

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
