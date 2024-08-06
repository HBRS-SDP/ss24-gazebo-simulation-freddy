import rclpy
from rclpy.node import Node 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Header  

keyBindings={}
speedBindings={}
componentBindings={}

class FreddyGazeboPublisher(Node):

    def __init__(self):
        pass

    def armLeftPosPublisher(self):
        pass
    def armRightPosPublisher(self):
        pass
    def baseVelocityPublisher(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    freddy_gazebo_publisher = FreddyGazeboPublisher()
    rclpy.spin(freddy_gazebo_publisher)
    freddy_gazebo_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

