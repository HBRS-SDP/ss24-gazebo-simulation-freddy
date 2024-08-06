import threading

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import Float64MultiArray, Header
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Header, Float64MultiArray
import numpy as np
import termios
import tty
import sys


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




class KeyboardPress():

    def __init__(self):

        self.keyBindings={
            'w': (1, 0, 0, 0, 0, 0, 0, 0),
            's': (-1, 0, 0, 0, 0, 0, 0, 0),
            'e': (0, 1, 0, 0, 0, 0, 0, 0),
            'd': (0, -1, 0, 0, 0, 0, 0, 0),
            'r': (0, 0, 1, 0, 0, 0, 0, 0),
            'f': (0, 0, -1, 0, 0, 0, 0, 0),
            't': (0, 0, 0, 1, 0, 0, 0, 0),
            'g': (0, 0, 0, -1, 0, 0, 0, 0),
            'y': (0, 0, 0, 0, 1, 0, 0, 0),
            'h': (0, 0, 0, 0, -1, 0, 0, 0),
            'u': (0, 0, 0, 0, 0, 1, 0, 0),
            'j': (0, 0, 0, 0, 0, -1, 0, 0),
            'i': (0, 0, 0, 0, 0, 0, 1, 0),
            'k': (0, 0, 0, 0, 0, 0, -1, 0),
            'o': (0, 0, 0, 0, 0, 0, 0, 1),
            'l': (0, 0, 0, 0, 0, 0, 0, -1),
        }
        self.speedBindings={
                '+': (1.03, 1.1),
                '-': (.6, .9)
        }
        self.componentBindings={
            'q':(1,0,0), # Left arm
            'a': (0,1,0), # Right arm
            'z': (0,0,1) # Base 
        }
        self.speed = 0.01



    def getKey(settings):
    
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    
    def modifyPosition(self,commands,component):

        key = self.getKey(termios.tcgetattr(sys.stdin))
        if key in  self.keyBindings.keys():
            moveBy = self.keyBindings[key]
        elif key in self.speedBindings.keys():
            self.speed = self.speed * self.speedBindings[key][0 if component[2] else 1]

        commands = commands + (moveBy *self.speed)

        return commands
            


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

