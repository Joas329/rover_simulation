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

#This controller takes the trajectory given by MoveIt inverse Kinemamtics
#It also takes the position from the joint states and uses both data to
#Output velocity that will later be taken by the second effort PID.

class VelPID(Node):

    def __init__(self):
        super().__init__("VelPID")

        #*****Declare Node Variables Here******* 

        self.i = 0
        joints_number = 6 #Change this based on the number of joints that we have
        self.stack  = [None] 

        self.timer = self.create_timer(0.02,self.timercallback) #TODO YOU CAN CHANGE THE TIMER TIME HERE
        self.timer2 = self.create_timer(0.1,self.timercallback2) #TODO YOU CAN CHANGE THE TIMER TIME HERE
        
        self.ordered_current_pose , self.current_pose_error, self.currentPosition, self.velocity_sum, self.desired_joint_positions  = [0.0] * joints_number
        self.p_pose_factor, self.i_pose_factor, self.d_pose_factor, self.previous_pose_error, self.velocity  = [0.0] * joints_number

        self.joint_order = [5,4,2,1,0,3]
        self.k_pose_p =[1.0,12.0,6.0,5.0,7.0,1.0]
        self.k_pose_i = [0.0,0.1,0.05,0.0,0.2,0.0]
        self.k_pose_d =[0.0,1.0,0.0,0.0,0.0,0.0]


        #********Subscribers********

        self.joint_trajectory_subscriber = self.create_subscription(
            DisplayTrajectory,
            'display_planned_path',
            self.joint_trajectory_callback,
            10
        )

        self.joint_state_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.position_feedback_callback,
            10
        )

        #***************Tunning Subscribers******************

        self.pid_p_new_values = self.create_subscription(
            Float64MultiArray,
            'effortPID/pvel',
            self.P_values_callback,
            10
        )    

        self.pid_i_new_values = self.create_subscription(
            Float64MultiArray,
            'effortPID/ivel',
            self.I_values_callback,
            10
        )   

        self.pid_d_new_values = self.create_subscription(
            Float64MultiArray,
            'effortPID/dvel',
            self.D_values_callback,
            10
        )   

        #***********Publishers***************
        self.velocity_publisher = self.create_publisher(
            Float64MultiArray,
            'arm_group_controller/velocity',
            10
        )
        
        #********** Tunning Publishers ***********
        self.debug_desired_pose = self.create_publisher(
            Float64MultiArray,
            'veloctiyPID/desired_pose',
            10
        )
        self.debug_current_pose = self.create_publisher(
            Float64MultiArray,
            'velocityPID/current_pose',
            10
        )


    

    #**********Subcriber Callbacks***********

    def joint_trajectory_callback(self, msg):
        self.stack = msg.trajectory
        self.i = 0

    def position_feedback_callback(self, msg):
            self.currentPosition = msg.position        
            for i in range(0,5):
                self.ordered_current_pose[i] = self.currentPosition[self.joint_order[i]]

    #************Tunning Subcriber Callbacks*************
    def P_values_callback(self, msg):
        self.k_vel_p = msg.data

    def I_values_callback(self, msg):
        self.k_vel_i = msg.data

    def D_values_callback(self, msg):
        self.k_vel_d = msg.data

    #*******************Timed Callbacks********************

    def timercallback(self):
        #This performs pid

        if  not self.stack[0] == None:
            
            velocities = self.pidCalc(self.desired_joint_positions)
            print(self.desired_joint_positions)

            #Publish Tunning Values

            self.velocity_publisher.publish(velocities)
            
            desired = Float64MultiArray()
            desired.data = self.desired_joint_positions
            self.pid_debug_desired_pose.publish(desired)

            current = Float64MultiArray()
            current.data = self.ordered_current_pose
            self.pid_debug_current_pose.publish(current)

    def timercallback2(self):
        #This indexes through array
        if  not self.stack[0] == None:
            if  len(self.stack[0].joint_trajectory.points) -1 > self.i:
                trajectories = self.stack[0]
                holder = trajectories.joint_trajectory
                holder2 = holder.points

                self.desired_joint_positions = holder2[self.i].positions


    #******************PID Calculation**********************

    def pidVelocityCalc(self, jointDesiredPositions):
        newmsg = Float64MultiArray()
        joint = 0

        for desiredPose in jointDesiredPositions:
            self.current_pose_error[joint] = desiredPose - self.ordered_current_pose[joint]
         
            self.p_pose_factor[joint] = self.k_pose_p[joint] * (self.current_pose_error[joint])
            #I function
            self.i_pose_factor[joint] = self.k_pose_i[joint] * (self.velocity_sum[joint] + self.current_pose_error[joint])
            self.velocity_sum[joint] += self.current_pose_error[joint]
            #D function
            self.d_pose_factor[joint] = self.k_pose_d [joint]* (self.current_pose_error[joint] - self.previous_pose_error[joint])
            self.previous_pose_error[joint] = self.current_pose_error[joint]
            #publish this value to the respective joint
            self.velocity[joint] = self.p_pose_factor[joint] + self.i_pose_factor[joint] + self.d_pose_factor[joint]
            newmsg.data.append(self.velocity[joint])
            joint+= 1
        return self.pidEffortCalc(newmsg)


def main(args=None):
    rclpy.init(args=args)
    node = VelPID()
    rclpy.spin(node)
    node.timer.cancel()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
