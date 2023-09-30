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
        self.stack  = [None] 
        self.trajectoryFlag = False
        self.readingTime = 0.0
        self.flag = False
        self.velocityPIDTimer = self.create_timer(0.02,self.velocityPID) #TODO YOU CAN CHANGE THE TIMER TIME HERE
        self.trajectoryIndexingTImer = self.create_timer(0.1,self.trajectoryIndexing) #TODO YOU CAN CHANGE THE TIMER TIME HERE
        self.clipub = self.create_timer(3,self.clipublisher) 

        # self.effortTimer = self.create_timer(0.02,self.effortCallback) 

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
            'velocityPID/pvel',
            self.P_values_callback,
            10
        )    

        self.pid_i_new_values = self.create_subscription(
            Float64MultiArray,
            'velocityPID/ivel',
            self.I_values_callback,
            10
        )   

        self.pid_d_new_values = self.create_subscription(
            Float64MultiArray,
            'velocityPID/dvel',
            self.D_values_callback,
            10
        )   

        self.effort_p_new_values = self.create_subscription(
            Float64MultiArray,
            'effortPID/peff',
            self.P_effort_values_callback,
            10
        )    

        self.effort_i_new_values = self.create_subscription(
            Float64MultiArray,
            'effortPID/ieff',
            self.I_effort_values_callback,
            10
        )   

        self.effort_d_new_values = self.create_subscription(
            Float64MultiArray,
            'effortPID/deff',
            self.D_effort_values_callback,
            10
        )  

        #***********Publishers***************
        self.effort_publisher = self.create_publisher(
            Float64MultiArray,
            'arm_group_controller/commands',
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

    def joint_trajectory_callback(self, msg):
        self.stack = msg.trajectory
        self.trajectoryFlag = False
        self.i = 0

    def position_feedback_callback(self, msg):
            
            if self.flag == False:                  # Runs once at the beginning of the code as the joint states are published immidiatly.
                self.joints_number = len(msg.name)
                self.joint_names = msg.name
                self.joint_order = [0] * self.joints_number
                
                # Velocity                
                self.ordered_current_pose = self.current_pose_error = self.currentPosition = self.velocity_sum = self.desired_joint_positions  = [0.0] * self.joints_number
                self.p_pose_factor= self.i_pose_factor = self.d_pose_factor = self.previous_pose_error = self.velocity  = [0.0] * self.joints_number
                self.k_pose_p =[0.1] * self.joints_number
                self.k_pose_i = [0.1] * self.joints_number
                self.k_pose_d =[0.1] * self.joints_number

                # Effort
                self.ordered_current_pose = self.curr_vel = self.prev_curr_vel = self.jointDesiredVelocity = [0.0] * len(msg.name)
                self.current_vel_error = self.previous_vel_error = self.p_vel_factor = self.i_vel_factor = self.d_vel_factor = self.effort_sum = self.effort= [0.0] * len(msg.name)
                self.k_vel_p =[0.1] * self.joints_number
                self.k_vel_i = [0.1] * self.joints_number
                self.k_vel_d =[0.1] * self.joints_number

                self.flag = True
                
                self.dynamicJointCheck(msg.name)
            
            self.readingTime = time.time()
            self.poseToVel()
            self.currentPosition = msg.position        
            for i in range(0,self.joints_number -1):
                self.ordered_current_pose[i] = self.currentPosition[self.joint_order[i]]

    #************Tunning Subcriber Callbacks*************
    # Velocity

    def P_values_callback(self, msg):
        print("p updated")
        self.k_pose_p = (msg.data).tolist()

    def I_values_callback(self, msg):
        print("i updated")
        self.k_pose_i = (msg.data).tolist()

    def D_values_callback(self, msg):
        print("d updated")
        self.k_pose_d = (msg.data).tolist()

    # Effort
    def P_effort_values_callback(self, msg):
        self.k_vel_p = (msg.data).tolist()

    def I_effort_values_callback(self, msg):
        self.k_vel_i = (msg.data).tolist()

    def D_effort_values_callback(self, msg):
        self.k_vel_d = (msg.data).tolist()

    #*******************Timed Callbacks********************

    def velocityPID(self):
        #This performs pid

        if  not self.stack[0] == None:
            
            velocities = self.pidVelocityCalc(self.desired_joint_positions)
            #print(velocities)
            self.effort_publisher.publish(velocities)

            #Publish Tunning Values

            # self.velocity_publisher.publish(velocities)
            
            desired = Float64MultiArray()
            desired.data = self.desired_joint_positions
            self.debug_desired_pose.publish(desired)

            current = Float64MultiArray()
            current.data = self.ordered_current_pose
            self.debug_current_pose.publish(current)

    def trajectoryIndexing(self):
        #This indexes through array
        if  not self.stack[0] == None:
            if self.trajectoryFlag == False:
                print("Received Trajectory")
                self.trajectoryFlag = True
            if  len(self.stack[0].joint_trajectory.points) -1 > self.i:
                trajectories = self.stack[0]
                holder = trajectories.joint_trajectory
                holder2 = holder.points

                self.desired_joint_positions = holder2[self.i].positions
                self.i +=1

    # def effortCallback(self):
    #     #This performs pid

    #     if  not self.stack[0] == None:
                
    #         velocities = self.pidEffortCalc(self.stack)

    #         self.prev_curr_vel = self.curr_vel

    #         self.effort_publisher.publish(velocities)

    def clipublisher(self):
        if self.flag == True:
            print("number of detected joints: " + str(self.joints_number))
            print("detected joints: " + str(self.joint_names)) #need to cahnge this to names instead of values
            print("***********Velocity**********")
            print("P values: " + str(self.k_pose_p))
            print("I values: " + str(self.k_pose_i))
            print("D values: " + str(self.k_pose_d))
            print("")
            print("***********Effort**********")
            print("P values: " + str(self.k_vel_p))
            print("I values: " + str(self.k_vel_i))
            print("D values: " + str(self.k_vel_d))



    #******************PID Calculation**********************

    def pidVelocityCalc(self, jointDesiredPositions):
        velocityCommand = Float64MultiArray()
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
            velocityCommand.data.append(self.velocity[joint])
            joint+= 1
        print("P factor "+str(self.p_pose_factor))
        print("I factor " +str(self.i_pose_factor))
        print("D factor " +str(self.d_pose_factor))
        return velocityCommand
        #return self.pidEffortCalc(velocityCommand.data)



    def pidEffortCalc(self, jointDesiredVelocity):
        effortCommand= Float64MultiArray()

        joint = 0

        for desiredVelocity in jointDesiredVelocity:
            self.current_vel_error[joint] = desiredVelocity - self.curr_vel[joint]

            self.p_vel_factor[joint] = self.k_vel_p[joint] * (self.current_vel_error[joint])

            self.i_vel_factor[joint] = self.k_vel_i[joint] *  (self.effort_sum[joint] + self.current_vel_error[joint])
            self.effort_sum[joint] += self.current_vel_error[joint]

            self.d_vel_factor[joint] = self.k_vel_d[joint] * (self.current_vel_error[joint] - self.previous_vel_error[joint])
            self.previous_vel_error[joint] = self.current_vel_error[joint]

            self.effort[joint] = self.p_vel_factor[joint] + self.i_vel_factor[joint] + self. d_vel_factor[joint]
            effortCommand.data.append(self.effort[joint])

            joint += 1
            
        return effortCommand

    #*************Velocity derivative***************
    def poseToVel(self):
        t = time.time() - self.readingTime

        for i in range(0,5):
            self.curr_vel[i] = (self.curr_vel[i] - self.prev_curr_vel[i]) / t


    #************Dynamically get order joints array**************
    def dynamicJointCheck(self, jointStates):
        counter = 0
        for joint in jointStates:
            self.joint_order[int(joint[len(joint)-1])-1] = counter
            counter += 1

def main(args=None):
    print("Initialiazing PID")
    rclpy.init(args=args)
    node = VelPID()
    rclpy.spin(node)
    node.timer.cancel()
    rclpy.shutdown()

    print()

if __name__ == '__main__':
    main()
