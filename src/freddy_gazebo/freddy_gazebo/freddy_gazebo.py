import threading

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import Float64MultiArray, Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

keyBindings={}
speedBindings={}
componentBindings={}


class FreddyGazeboPublisher(Node):
    def __init__(self, ) -> None:
        super().__init__('FreddyGazeboPublisher')

        self.arm_left_pose_publisher: Publisher = self.create_publisher(
            JointTrajectory, 
            '/arm_left_joint_trajectory_controller/joint_trajectory', 
            10,
            )
        self.arm_right_pose_publisher: Publisher = self.create_publisher(
            JointTrajectory, 
            '/arm_right_joint_trajectory_controller/joint_trajectory', 
            10,
            )
        self.base_publisher: Publisher = self.create_publisher(
            Float64MultiArray, 
            '/base_velocity_controller/commands', 
            10,
            )
        
        self.arm_trajectory_duration = 2

        self.robot_publishers: dict[str, Publisher] = {
            "arm_left": self.arm_left_pose_publisher,
            "arm_right": self.arm_right_pose_publisher,
            "base": self.base_publisher,
        }

        self.commands: dict[str, np.ndarray] = {
            "arm_left": np.zeros(7),
            "arm_right": np.zeros(7),
            "base": np.zeros(8),
        }

        self.msgs: dict = {
            "arm_left": JointTrajectory(),
            "arm_right": JointTrajectory(),
            "base": Float64MultiArray(),
        }


    def update_commands(self, component_name: str, increment: np.ndarray, ) -> None:
        # Since commands are of variable length, only take the required number of elements
        self.commands[component_name] += increment[:len(self.commands[component_name])]

        self.update_msgs()

        return None


    def update_msgs(self, ) -> None:
        for component_name in self.commands:
            if "arm" in component_name:
                msg = JointTrajectory()

                trajectory_point = JointTrajectoryPoint()
                trajectory_point.positions = self.commands[component_name].tolist()
                trajectory_point._time_from_start = Duration(sec=self.arm_trajectory_duration)

                msg.points.append(trajectory_point)
                self.msgs[component_name] = msg

            elif component_name == "base":
                msg = Float64MultiArray()
                msg.data = self.commands[component_name].tolist()

                self.msgs[component_name] = msg

        return None


    def publish_commands(self, ) -> None:
        for component_name in self.robot_publishers:
            self.robot_publishers[component_name].publish(self.msgs[component_name])

        return None



def main(args=None):
    rclpy.init(args=args)
    freddy_gazebo_publisher = FreddyGazeboPublisher()
    
    spinner = threading.Thread(target=rclpy.spin, args=(freddy_gazebo_publisher,))
    spinner.start()

    try:
        while True:
            # Get pressed key

            # Update commands

            # Publish messages
            freddy_gazebo_publisher.publish_commands()
    
    except Exception as e:
        print(e)

    finally:
        rclpy.shutdown()
        spinner.join()



if __name__ == '__main__':
    main()

