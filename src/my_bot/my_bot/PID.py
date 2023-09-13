
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
flag = False

class MyNode(Node):

    def __init__(self):
        super().__init__('pid_node')
        self.timer = self.create_timer(0.02,self.timercallback) #TODO YOU CAN CHANGE THE TIMER TIME HERE
        self.timer2 = self.create_timer(0.1,self.timercallback2) #TODO YOU CAN CHANGE THE TIMER TIME HERE
        self.time_variable = 0.1
        self.stack  = [None] 
        #DO NOT FORGET: number of 0s must match number of joints 
        self.current_error = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.currentPosition = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.effortsum =[0.0,0.0,0.0,0.0,0.0,0.0]
        self.desired_joint_positions = [0.0,0.0,0.0,0.0,0.0,0.0]

        self.previous_error = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.namestack = []
        self.effort = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.p_factor = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.i_factor = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.d_factor = [0.0,0.0,0.0,0.0,0.0,0.0]

        self.i = 0
        self.k_p =[5.0,15.0,6.0,5.0,7.0,1.0]
        self.k_i = [0.0,0.1,0.05,0.0,0.2,0.0]
        self.k_d =[0.0,1.0,0.0,0.0,0.0,0.0]


        #order of the joints that come from the joint_states topic
        #Make sure to check the order by echoing the topic and checkng the positions of the joints, to ensure a good PID control
        #CLARIFICATION: substract -1 to the order of the joint to index the array correctly
        # self.joint_order = [0,2,3,4,1,5]
        self.joint_order = [5,4,2,1,0,3]
        self.ordered_current_pose = [0.0 , 0.0, 0.0, 0.0, 0.0, 0.0]

        # Create subscribers
        self.joint_trajectory_subscriber = self.create_subscription(
            DisplayTrajectory,
            'display_planned_path',
            self.joint_trajectory_callback,
            10
        )

        self.position_feedback_subscriber = self.create_subscription(
            JointState,
            'joint_states',
            self.position_feedback_callback,
            10
        )

        # Create publisher
        self.effort_publisher = self.create_publisher(
            Float64MultiArray,
            'arm_group_controller/commands',
            10
        )
        self.pid_debug_effort = self.create_publisher(
            Float64MultiArray,
            'pid/effort',
            10
        )
        self.pid_debug_desired_pose = self.create_publisher(
            Float64MultiArray,
            'pid/desired_pose',
            10
        )
        self.pid_debug_current_pose = self.create_publisher(
            Float64MultiArray,
            'pid/current_pose',
            10
        )

        # self.pid_values = self.create_subscription(
        #     Float64MultiArray,
        #     'pid/values',
        #     self.pid_values_callback,
        #     10
        # )

        #GUI
        # self.gui_feedback_subscriber = self.create_subscription(
        #     Float64MultiArray,
        #     'gui_commands',
        #     self.gui_feedback_callback,
        #     10
        # )


    def joint_trajectory_callback(self, msg):
        self.stack = msg.trajectory
        self.i = 0

    # def pid_values_callback(self, msg):
    #     for p in range(0,5):
    #         self.p_factor[p] = msg[p] 
    #     for i in range(6, 11):
    #         self.i_factor[i] = msg[i]
    #     for d in range(12, 17):
    #         self.d_factor[d] = msg[d]
    #     print("p_factor")
    #     print(self.p_factor)
    #     print("i_factor")
    #     print(self.i_factor)
    #     print("d_factor")
    #     print(self.d_factor)
      

    # def gui_feedback_callback(self, msg):
    #     self.desired_joint_positions = msg.trajectory
    #     self.i=0

    def position_feedback_callback(self, msg):
            self.currentPosition = msg.position        
            for i in range(0,5):
                self.ordered_current_pose[i] = self.currentPosition[self.joint_order[i]]
            
    def timercallback2(self):
        #This indexes through array
        if  not self.stack[0] == None:
            if  len(self.stack[0].joint_trajectory.points) -1 > self.i:
                trajectories = self.stack[0]
                holder = trajectories.joint_trajectory
                holder2 = holder.points

                self.desired_joint_positions = holder2[self.i].positions
                self.i +=1

    def timercallback(self):
        #This performs pid

        if  not self.stack[0] == None:
            # if len(self.stack[0].joint_trajectory.points) > self.i:
                
            efforts = self.pidCalc(self.desired_joint_positions)
            # print(efforts)
            print(self.desired_joint_positions)

            self.effort_publisher.publish(efforts)

            desired = Float64MultiArray()
            desired.data = self.desired_joint_positions
            self.pid_debug_desired_pose.publish(desired)

            current = Float64MultiArray()
            current.data = self.ordered_current_pose
            self.pid_debug_current_pose.publish(current)

        #Code here is 
        # if not self.desired_joint_positions ==  [0.0,0.0,0.0,0.0,0.0,0.0]:
        #     efforts = self.pidCalc(self.desired_joint_positions)
        #     print(efforts)
        #     self.effort_publisher.publish(efforts)



          


    def pidCalc(self, jointDesiredPositions):
        newmsg = Float64MultiArray()
        # print(jointDesiredPositions)
        joint = 0
        for desiredPose in jointDesiredPositions:
            self.current_error[joint] = desiredPose - self.ordered_current_pose[joint]
         
            self.p_factor[joint] = self.k_p[joint] * (self.current_error[joint])
            #I function
            self.i_factor[joint] = self.k_i[joint] * (self.effortsum[joint] + self.current_error[joint])
            self.effortsum[joint] += self.current_error[joint]
            #D function
            self.d_factor[joint] = self.k_d [joint]* (self.current_error[joint] - self.previous_error[joint])
            self.previous_error[joint] = self.current_error[joint]
            #publish this value to the respective joint
            self.effort[joint] = self.p_factor[joint] + self.i_factor[joint] + self.d_factor[joint]
            newmsg.data.append(self.effort[joint])
            joint+= 1
        return newmsg
    

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.timer.cancel()
    rclpy.shutdown()

if __name__ == '__main__':
    main()