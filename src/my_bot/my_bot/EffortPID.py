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

#This controller takes the output from the velocity PID controller and uses it to proportionally 
#It also takes the position of the joint_state topic and it differentiates its value
#based on the time difference from the previous state meassuremente and the ew state meassurement

class EffortPID(Node):

    def __init__(self):
        super().__init__("EffortPID")

        #*****Declare Node Variables Here******* 

        joints_number = 6  #Change this based on the number of joints that we have

        self.stack  = [None]

        self.readingTime = 0.0
        self.ordered_current_pose, self.curr_vel, self.prev_curr_vel, self.jointDesiredVelocity = [0.0] * joints_number
        self.current_vel_error, self.previous_vel_error, self.p_vel_factor, self.i_vel_factor , self.d_vel_factor, self.effort_sum, self.effort= [0.0] * joints_number
    
        self.joint_order = [5,4,2,1,0,3]

        self.k_vel_p =[1.0,12.0,6.0,5.0,7.0,1.0]
        self.k_vel_i = [0.0,0.1,0.05,0.0,0.2,0.0]
        self.k_vel_d =[0.0,1.0,0.0,0.0,0.0,0.0]


        #**************Subscribers*************

        self.joint_velocity_subscriber =self.create_subscription(
            Float64MultiArray,
            'arm_group_controller/velocity',
            self.velocity_callback,
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
            'effortPID/peff',
            self.P_values_callback,
            10
        )    

        self.pid_i_new_values = self.create_subscription(
            Float64MultiArray,
            'effortPID/ieff',
            self.I_values_callback,
            10
        )   

        self.pid_d_new_values = self.create_subscription(
            Float64MultiArray,
            'effortPID/deff',
            self.D_values_callback,
            10
        )     

        #***********Publishers***************
        self.effort_publisher = self.create_publisher(
            Float64MultiArray,
            'arm_group_controller/effort',
            10
        )

        #********** Tunning Publishers ***********
        self.debug_desired_velocity = self.create_publisher(
            Float64MultiArray,
            'effortPID/desired_velocity',
            10
        )
        self.debug_current_velocity = self.create_publisher(
            Float64MultiArray,
            'effortPID/current_velocity',
            10
        )
    #**********Subcriber Callbacks***********

    def velocity_callback(self, msg):
        self.stack = msg.trajectory
        self.jointDesiredVelocity = self.stack
        self.i = 0

    def position_feedback(self, msg):
         self.currentPosition = msg.position
         self.readingTime = time.time()
         for i in range(0,5):
            self.ordered_current_pose[i] = self.currentPosition[self.joint_order[i]]

         self.poseToVel(self.ordered_current_pose)

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
                
            velocities = self.pidEffortCalc(self.stack)

            self.prev_curr_vel = self.curr_vel

            self.effort_publisher.publish(velocities)


    #******************PID Calculation**********************

    def pidEffortCalc(self, jointDesiredVelocity):
        newmsg= Float64MultiArray()

        joint = 0

        for joint in range(0,5):
            self.current_vel_error[joint] = jointDesiredVelocity[joint] - self.curr_vel[joint]

            self.p_vel_factor[joint] = self.k_vel_p[joint] * (self.current_vel_error[joint])

            self.i_vel_factor[joint] = self.k_vel_i[joint] *  (self.effort_sum[joint] + self.current_vel_error[joint])
            self.effort_sum[joint] += self.current_vel_error[joint]

            self.d_vel_factor[joint] = self.k_vel_p[joint] * (self.current_vel_error[joint] - self.previous_vel_error[joint])
            self.previous_vel_error[joint] = self.current_vel_error[joint]

            self.effort[joint] = self.p_vel_factor[joint] + self.i_vel_factor[joint] + self. d_vel_factor[joint]
            newmsg.data.append(self.effort[joint])

            joint += 1

            return newmsg


    def poseToVel(self,pose):
        t = time.time() - self.readingTime

        for i in range(0,5):
            self.curr_vel[i] = (self.curr_vel[i] - self.prev_curr_vel[i]) / t

def main(args=None):
    rclpy.init(args=args)
    node = EffortPID()
    rclpy.spin(node)
    node.timer.cancel()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
