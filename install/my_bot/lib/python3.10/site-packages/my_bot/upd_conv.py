import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointTrajectory_Formatter_ManuelWay(Node):


    def init(self):
        super().init('JointTrajectoryFormatter')
        self.publisher = self.create_publisher(Float64MultiArray, '/planned_path', 10)
        self.subscription = self.create_subscription(String, '/arm-control-values', self.listener_callback, 10)

    def listener_callback(self, msg):
        floatarr = Float64MultiArray
        k = msg.data.split(',')
        for fl in k:
            floatarr.data.append(float(fl))

        self.publisher.publish(floatarr)

def main(args=None):
    rclpy.init(args=args)

    jointTrajectory_formatter = JointTrajectory_Formatter_ManuelWay()

    rclpy.spin(jointTrajectory_formatter)
    jointTrajectory_formatter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()