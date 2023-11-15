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

        #********Timers********

        self.velocityPIDTimer = self.create_timer(0.02,self.velocityPID) #TODO YOU CAN CHANGE THE TIMER TIME HERE
        self.trajectoryIndexingTImer = self.create_timer(0.1,self.trajectoryIndexing) #TODO YOU CAN CHANGE THE TIMER TIME HERE
        self.clipub = self.create_timer(3,self.clipublisher) 

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
                self.ordered_current_pose = self.currentPosition = self.desired_joint_positions  = [0.0] * self.joints_number
                self.command = self.velocity  = [0.0] * self.joints_number

                # Effort
                self.ordered_current_pose = self.curr_vel = self.prev_curr_vel = self.jointDesiredVelocity = [0.0] * len(msg.name)
                self.effort= [0.0] * len(msg.name)


                #Parameter Definition
                self.declare_parameter("k_pose_p", [0.0] * self.joints_number)
                self.declare_parameter("k_pose_i", [0.0] * self.joints_number)
                self.declare_parameter("k_pose_d", [0.0] * self.joints_number)
                self.declare_parameter("k_vel_p", [0.0] * self.joints_number)
                self.declare_parameter("k_vel_i", [0.0] * self.joints_number)
                self.declare_parameter("k_vel_d", [0.0] * self.joints_number)

                self.flag = True
                
                self.dynamicJointCheck(msg.name)
            
            self.readingTime = time.time()
            self.poseToVel()
            self.currentPosition = msg.position        
            for i in range(0,self.joints_number -1):
                self.ordered_current_pose[i] = self.currentPosition[self.joint_order[i]]

    #*******************Timed Callbacks********************

    def velocityPID(self):
        #This performs pid

        if  not self.stack[0] == None:
            
            velocities = self.pidVelocityCalc(self.desired_joint_positions,self.joints_number)
            #print(velocities)
            self.effort_publisher.publish(velocities)
            self.command = velocities

            #Publish Tunning Values

            # self.velocity_publisher.publish(velocities)
            
            desired = Float64MultiArray()
            desired.data = self.desired_joint_positions
            self.debug_desired_pose.publish(desired)

            current = Float64MultiArray()
            current.data = self.ordered_current_pose
            self.debug_current_pose.publish(current)

    def trajectoryIndexing(self):
        #This indexes through trajectory array
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

    def effortCallback(self):
        #This performs pid

        if  not self.stack[0] == None:
                
            velocities = self.pidEffortCalc(self.stack)

            self.prev_curr_vel = self.curr_vel

            self.effort_publisher.publish(velocities)

    def clipublisher(self):
        
        if self.flag == True:

            k_pose_p = self.get_parameter('k_pose_p').get_parameter_value().double_array_value
            k_pose_i = self.get_parameter('k_pose_i').get_parameter_value().double_array_value
            k_pose_d = self.get_parameter('k_pose_d').get_parameter_value().double_array_value
            k_vel_p = self.get_parameter('k_vel_p').get_parameter_value().double_array_value
            k_vel_i = self.get_parameter('k_vel_i').get_parameter_value().double_array_value
            k_vel_d = self.get_parameter('k_vel_d').get_parameter_value().double_array_value


            print("number of detected joints: " + str(self.joints_number))
            print("detected joints: " + str(self.joint_names)) #need to cahnge this to names instead of values
            print("***********Velocity**********")
            print("P values: " + str(k_pose_p.tolist()))
            print("I values: " + str(k_pose_i.tolist()))
            print("D values: " + str(k_pose_d.tolist()))
            print("")
            print("***********Effort**********")
            print("P values: " + str(k_vel_p.tolist()))
            print("I values: " + str(k_vel_i.tolist()))
            print("D values: " + str(k_vel_d.tolist()))
            print("***********Commands**********")
            print("commands" + str((self.command)))
            print("**********Current Position*************")
            print(self.currentPosition)
            print("**********Desired Position*************")
            print(self.desired_joint_positions)
            print("\n \n")


    #******************Velocity PID Calculation**********************

    def pidVelocityCalc(self, jointDesiredPositions, joints_number):
        velocityCommand = Float64MultiArray()

        k_pose_p = self.get_parameter('k_pose_p').get_parameter_value().double_array_value
        k_pose_i = self.get_parameter('k_pose_i').get_parameter_value().double_array_value
        k_pose_d = self.get_parameter('k_pose_d').get_parameter_value().double_array_value

        joint = 0

        current_pose_error = [0.1] * joints_number
        velocity_sum = [0.1] * joints_number
        previous_pose_error = [0.1] * joints_number
        p_pose_factor = [0.1] * joints_number
        i_pose_factor = [0.1] * joints_number
        d_pose_factor = [0.1] * joints_number

        for desiredPose in jointDesiredPositions:
            current_pose_error[joint] = desiredPose - self.ordered_current_pose[joint]
            #P function
            p_pose_factor[joint] = k_pose_p[joint] * (current_pose_error[joint])
            #I function
            i_pose_factor[joint] = k_pose_i[joint] * ((velocity_sum[joint] + current_pose_error[joint]))
            velocity_sum[joint] += current_pose_error[joint]
            #D function
            d_pose_factor[joint] = k_pose_d[joint] * (current_pose_error[joint] - previous_pose_error[joint])
            previous_pose_error[joint] = current_pose_error[joint]
            #publish this value to the respective joint
            self.velocity[joint] = p_pose_factor[joint] + i_pose_factor[joint] + d_pose_factor[joint]
            velocityCommand.data.append(self.velocity[joint])
            joint+= 1
        return velocityCommand
        #return self.pidEffortCalc(velocityCommand.data)

    #******************Effort PID Calculation**********************

    def pidEffortCalc(self, jointDesiredVelocity, joints_number):
        effortCommand= Float64MultiArray()

        k_vel_p = self.get_parameter('k_vel_p').get_parameter_value().double_array_value
        k_vel_i = self.get_parameter('k_vel_i').get_parameter_value().double_array_value
        k_vel_d = self.get_parameter('k_vel_d').get_parameter_value().double_array_value

        joint = 0

        current_vel_error = [0.1] * joints_number
        effort_sum = [0.1] * joints_number
        previous_vel_error = [0.1] * joints_number
        p_vel_factor = [0.1] * joints_number
        i_vel_factor = [0.1] * joints_number
        d_vel_factor = [0.1] * joints_number

        for desiredVelocity in jointDesiredVelocity:
            current_vel_error[joint] = desiredVelocity - self.curr_vel[joint]

            p_vel_factor[joint] = k_vel_p[joint] * (current_vel_error[joint])

            i_vel_factor[joint] = k_vel_i[joint] *  (effort_sum[joint] + current_vel_error[joint])
            effort_sum[joint] += current_vel_error[joint]

            d_vel_factor[joint] = k_vel_d[joint] * (current_vel_error[joint] - previous_vel_error[joint])
            previous_vel_error[joint] = current_vel_error[joint]

            self.effort[joint] = p_vel_factor[joint] + i_vel_factor[joint] + d_vel_factor[joint]
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