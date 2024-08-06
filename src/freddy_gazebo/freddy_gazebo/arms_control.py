import rclpy
from rclpy.node import Node 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from std_msgs.msg import Header  
 

arm_left_joints = [ "kinova_left_joint_1",
      "kinova_left_joint_2",
       "kinova_left_joint_3",
      "kinova_left_joint_4",
      "kinova_left_joint_5",
      "kinova_left_joint_6",
      "kinova_left_joint_7"]

arm_right_joints = [ "kinova_right_joint_1",
      "kinova_right_joint_2",
       "kinova_right_joint_3",
      "kinova_right_joint_4",
      "kinova_right_joint_5",
      "kinova_right_joint_6",
      "kinova_right_joint_7"]

class ArmControllerPublisher(Node):
    """This class executes a sample trajectory for a robotic arm
     
    """     
    def __init__(self):
        super().__init__('arms_control')    
  
        self.arm_left_pose_publisher = self.create_publisher(JointTrajectory, '/arm_left_joint_trajectory_controller/joint_trajectory', 10)
        self.arm_right_pose_publisher = self.create_publisher(JointTrajectory, '/arm_right_joint_trajectory_controller/joint_trajectory', 10)
 
        self.timer_period = 3.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
 
        self.frame_id = "base_link"
        
        # Have not thought about this
        # Desired time from the trajectory start to arrive at the trajectory point.
        # Needs to be less than or equal to the self.timer_period above to allow
        # the robotic arm to smoothly transition between points.
        self.duration_sec = 2
        self.duration_nanosec = 0.5 * 1e9
 
        self.arm_positions = []
        self.arm_positions.append([-0.70, -2.05, 2.14, -2.44, -1.57, 0.00, 0.00]) # Left initial pos
        self.arm_positions.append([0.70, -2.05, 0.90, 2.44, 1.57, 0.00, 0.00]) # Right intial pos
 
        self.index = 0
 
    def timer_callback(self):
        """Set the goal pose for the robotic arm.
     
        """
        # Create new JointTrajectory messages
        msg_left_arm = JointTrajectory()
        msg_left_arm.header = Header()  
        msg_left_arm.header.frame_id = self.frame_id  
        msg_left_arm.joint_names = arm_left_joints

        msg_right_arm = JointTrajectory()
        msg_right_arm.header = Header()  
        msg_right_arm.header.frame_id = self.frame_id  
        msg_right_arm.joint_names = arm_right_joints
    
        point_arm_left = JointTrajectoryPoint()
        point_arm_right = JointTrajectoryPoint()
        point_arm_left.positions = self.arm_positions[0] if (self.index % 2 == 0)  else [0.3* i for i in range(1,8)]
        point_arm_right.positions = self.arm_positions[1] if (self.index % 2 == 1)  else [0.3* i for i in range(1,8)]

        point_arm_left.time_from_start = Duration(sec=int(self.duration_sec), nanosec=int(self.duration_nanosec))  # Time to next position
        point_arm_right.time_from_start = Duration(sec=int(self.duration_sec), nanosec=int(self.duration_nanosec))  # Time to next position

        msg_left_arm.points.append(point_arm_left)
        msg_right_arm.points.append(point_arm_right)
        self.arm_left_pose_publisher.publish(msg_left_arm)
        self.arm_right_pose_publisher.publish(msg_right_arm)
 
        
        # Reset the index
        if self.index == 4:
            self.index = 0
        else:
            self.index = self.index + 1
     
def main(args=None):
   
    # Initialize the rclpy library
    rclpy.init(args=args)
   
    # Create the node
    arms_controller = ArmControllerPublisher()
   
    rclpy.spin(arms_controller)
     
    # Destroy the node
    arms_controller.destroy_node()
   
    # Shutdown the ROS client library for Python
    rclpy.shutdown()
   
if __name__ == '__main__':
    main()
