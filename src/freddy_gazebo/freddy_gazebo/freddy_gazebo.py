import rclpy
from rclpy.node import Node 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Header, Float64MultiArray
import numpy as np


keyBindings={}
speedBindings={}
componentBindings={}


class FreddyGazeboPublisher(Node):
    def __init__(self):
        super().__init__('FreddyGazeboPublisher')

        self.arm_left_pose_publisher = self.create_publisher(
            JointTrajectory, 
            '/arm_left_joint_trajectory_controller/joint_trajectory', 
            10,
            )
        self.arm_right_pose_publisher = self.create_publisher(
            JointTrajectory, 
            '/arm_right_joint_trajectory_controller/joint_trajectory', 
            10,
            )
        self.base_publisher = self.create_publisher(
            Float64MultiArray, 
            '/base_velocity_controller/commands', 
            10,
            )
        
        self.publishers = [
            self.arm_left_pose_publisher,
            self.arm_right_pose_publisher,
            self.base_publisher,
        ]

        self.timer_period = 1.0
        self.timer = self.create_timer(self.timer_period, self.publish_commands)

        self.commands = {
            "arm_left": np.zeros(7),
            "arm_left": np.zeros(8),
            "base": np.zeros(8),
        }


    def publish_commands(self, msgs: list):
        for publisher, msg in zip(self.publishers, msgs):
            publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    freddy_gazebo_publisher = FreddyGazeboPublisher()
    rclpy.spin(freddy_gazebo_publisher)
    freddy_gazebo_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

