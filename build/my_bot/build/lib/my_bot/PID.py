
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import time

#declaring tunning variables

k_p = 1.0
k_i = 0.1
k_d = 0.1

#subscriber to joint position
def create_dummy_joint_trajectory(self):
        trajectory_msg = JointTrajectory()
        trajectory_msg.header = Header()
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg.joint_names = ["joint1", "joint2", "joint3", "joint4", "joint5"]

        point = JointTrajectoryPoint()
        point.positions = [0.1, 0.2, 0.3, 0.4, 0.5]
        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0]
        point.time_from_start = rclpy.duration.Duration(seconds=1).to_msg()

        trajectory_msg.points.append(point)

        return trajectory_msg

desired_position = 10.0

#subscriber to position state interface

current_position = 13.0
previous_error = 0
current_error = desired_position - current_position


class MyNode(Node):

    def __init__(self):
        super().__init__('my_node')

        # Create subscribers
        self.joint_trajectory_subscriber = self.create_subscription(
            Float64MultiArray,
            '/arm_group_controller/joint_Trajectory',
            self.joint_trajectory_callback,
            10
        )

        self.position_feedback_subscriber = self.create_subscription(
            Float64MultiArray,
            '/position_feedback',
            self.position_feedback_callback,
            10
        )

        # Create publisher
        self.effort_publisher = self.create_publisher(
            Float64MultiArray,
            '/effort',
            10
        )

    def joint_trajectory_callback(self, msg):
        global desired_position
        desired_position = create_dummy_joint_trajectory()

    def position_feedback_callback(self, msg):
         
        global current_position
        current_position = create_dummy_joint_trajectory()


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    while(True):

        #P function
        p_factor = k_p * (node.joint_trajectory_callback() - node.position_feedback_callback())
        #I function
        i_factor = k_i * (current_error + previous_error)

        previous_error = current_error
        #D function
        d_factor = k_d * (current_error/time_variable)
        

        #publish this value to the respective joint
        effort = p_factor + i_factor + d_factor
        node.effort_publisher(effort)

        time.sleep(0.1)

    rclpy.shutdown()

if __name__ == '__main__':
    main()