#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import NavSatFix, Imu
from custom_msg.msg import FilteredData
from std_msgs.msg import Float64MultiArray
from custom_msg.msg import NextPoint
import re
import math




buffer = 5  # means a 10 degree window (in the range 0-180)
tolerance = 2  # the minimum acceptable distance to the final point acceptable
rv = 0
lv = 0

#defining global variables for CSV file reading
dx_initial = 0 
dy_initial = 0
alpha = 0

class AutonomousMovementController(Node):

    def __init__(self):
        super().__init__('amc')

        self.initialx 
        self.initialy
        self.finalx
        self.finaly
        self.efforts= [5.0,5.0,5.0,5.0,5.0,5.0] 


        self.amber = ""

        self.imu_data = self.create_subscription(
            Imu,
            'imu',
            self.imu_callback,
            10
        )

        self.gps_data = self.create_subscription(
            NavSatFix,
            'gps',
            self.gps_callback,
            10
        )

        self.amber_light = self.create_subscription(
            String,
            'autonomy/amber_light',
            self.amber_light_callback,
            10
        )

        self.amber_effort_publisher = self.create_publisher(
            Float64MultiArray,
            '/driver_controller/commands',
            10
        )
        self.forward_effort_publisher = self.create_publisher(
            Float64MultiArray,
            '/forward_effort',
            10
        )

    def gps_callback(self, msg):
        
        self.finalx = "0"
        self.finaly = "0"

    def imu_callback(self, msg):
        self.initialx = "0"
        self.initialy = "0"

    def amber_light_callback(self, msg):
        self.amber = msg

    def AMC(self, dx_final, dy_final):
        status = False
        while(True):

            iota, direction = self.iotaCalc(dx_initial, dy_initial, dx_final, dy_final, alpha)
            distance = np.sqrt((dx_final - dx_initial)**2 +
                        (dy_final - dy_initial)**2)

            if(iota > buffer):
                self.changeYaw(direction)
                status = False
            else:
                status = self.drive(distance)

            if(status):
                break


    def iotaCalc(self, curr_x, curr_y, dx_final, dy_final, alpha):

        theta
        iota
        direction # if 1 --> turn right || 0 --> turn left
        fy = dy_final - curr_y
        fx = dx_final - curr_x
        if (fx > 0 and fy > 0): #if in cuadrant 1 calculation
            theta = 270 + abs(np.arctan2(fy, fx)) * (180 / np.pi)
            
        elif (fx < 0 and fy > 0): # if in cuadrant 2 calculation
            theta = abs(np.arctan2(-fx, fy)) * (180 / np.pi)
        elif(fx < 0 and fy < 0): # if in cuadrant 3 calculation
            theta = 90 + abs(np.arctan2(-fy, -fx)) * (180 / np.pi)
        elif(fx > 0 and fy < 0): # if in cuadrant 4 calculation
            theta = 180 + abs(np.arctan2(fx, -fy)) * (180 / np.pi)
        else:
            theta = 0
        
        iota = theta - alpha
        if(iota > 180):
            iota = 360 - iota
            print("\n Turn right by: ") + print(iota) 
            direction = 1

        elif(theta < alpha):
            iota = alpha -theta
            direction = 1
            print("\n Turn right by: ") +print(alpha - theta )
        else:
            direction = 0
            print("\n Turn left by: " )+print(iota) 

        return iota, direction
    

    def changeYaw(self, direction): 

        if (direction == 1):
            # turn rotate right
            self.efforts= [5.0,5.0,5.0,-5.0,-5.0,-5.0]
            self.forward_effort_publisher.publish(self.efforts)

        else:
            # turn rotate left
            self.efforts= [-5.0,-5.0,-5.0,5.0,5.0,5.0]
            self.forward_effort_publisher.publish(self.efforts)


    def drive(self,distance):  # TO DO make the drive function better

        if (distance > tolerance):
            self.efforts= [-5.0,-5.0,-5.0,5.0,5.0,5.0]
            self.forward_effort_publisher.publish(self.efforts)
        else:
            return True
        return False
    
    def GCStoM(self, lat1, long1, lat2, long2):
        r = 6365.766  # Radius of Earth in kilometers
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(long2 - long1)
        alpha = math.atan2(dlon, dlat)
        alpha = math.degrees(alpha) #turns it into degrees
        if alpha < 0:
            alpha += 360
        if alpha > 180:
            alpha = 360 - alpha 
        a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2     
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        d = r * c * 1000  # Convert to meters
        return d, alpha


    def amberLight(self):
       
        while self.amber == "Insufficient signal":
            self.efforts= [-5.0,-5.0,-5.0,5.0,5.0,5.0]
            self.amber_effort_publisher.publish(self.efforts)

        match = re.search(r'x: (\d+\.\d+), intensity: (\d+\.\d+)', self.amber)
        x = float(match.group(1))
        y = float(match.group(2))

        if x < 0.43:

            self.efforts= [5.0,5.0,5.0,-5.0,-5.0,-5.0]
            self.amber_effort_publisher.publish(self.efforts)

        elif x > 0.53:

            self.efforts= [-5.0,-5.0,-5.0,5.0,5.0,5.0]
            self.amber_effort_publisher.publish(self.efforts)
        elif x > 0.44 and x < 0.52:

            self.efforts= [5.0,5.0,5.0,5.0,5.0,5.0]
            self.amber_effort_publisher.publish(self.efforts)
        



       



def main(args=None):
    rclpy.init(args=args)
    node = AutonomousMovementController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


                
