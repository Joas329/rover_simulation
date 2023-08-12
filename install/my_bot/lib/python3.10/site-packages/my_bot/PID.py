
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import math
import moveit_msgs.msg
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory
from std_msgs.msg import Header
import time

flag = True

class MyNode(Node):

    def __init__(self):
        super().__init__('my_node')
        self.timer = self.create_timer(0.1,self.timercallback) #TODO YOU CAN CHANGE THE TIMER TIME HERE
        self.time_variable = 0.1
        self.stack = []
        self.current_error = []
        self.currentPosition = []
        self.effortsum = []
        self.previous_error = []
        self.namestack = []
        self.effort = []
        self.p_factor = []
        self.i_factor = []
        self.d_factor = []

        self.k_p = 100.0
        self.k_i = 0
        self.k_d = 0

        # Create subscribers
        self.joint_trajectory_subscriber = self.create_subscription(
            DisplayTrajectory,
            '/display_planned_path',
            self.joint_trajectory_callback,
            10
        )

        self.position_feedback_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.position_feedback_callback,
            10
        )

        # Create publisher
        self.effort_publisher = self.create_publisher(
            Float64MultiArray,
            '/arm_group_controller/commands',
            10
        )
    def joint_trajectory_callback(self, msg):
        self.stack = msg.trajectory

    def position_feedback_callback(self, msg):
            self.currentPosition = msg.position
            # print(self.currentPosition)
            
    

    def timercallback(self):
        effor = self.pidCalc([0.0, -86.0,0.0 ,0.0 ,0.0])
        self.effort_publisher.publish(effor)

        # if not len(self.stack) == 0:
        #     for desiredPoses in self.stack.joint_trajectory.points:
        #         efforts = self.pidCalc(desiredPoses.positions)
        #         print(efforts)
        #         self.effort_publisher.publish(efforts)


    def pidCalc(self, jointDesiredPositions):
        newmsg = Float64MultiArray()
        joint = 0
        for desiredPose in jointDesiredPositions:
        
            print("in pidcalc")
            self.current_error[joint] = desiredPose - self.currentPosition[joint]
            # self.current_error = self.stack[0].positions[0] - self.realpos
            self.p_factor[joint] = self.k_p * (self.current_error[joint])
            #I function
            self.i_factor[joint] = self.k_i * (self.effortsum[joint] + self.previous_error[joint])
            self.effortsum[joint] += self.current_error[joint]
            #D function
            self.d_factor[joint] = self.k_d * (self.current_error[joint] - self.previous_error[joint])
            #publish this value to the respective joint
            self.effort[joint] = self.p_factor[joint] + self.i_factor[joint] + self.d_factor[joint]
            newmsg.data.append(self.effort[joint])
            joint += 1
        return newmsg

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.timer.cancel()
    rclpy.shutdown()

if __name__ == '__main__':
    main()