import rclpy
from rclpy.node import Node 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Header, Float64MultiArray
import numpy as np
import termios
import tty
import sys


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
    rclpy.spin(freddy_gazebo_publisher)
    freddy_gazebo_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

