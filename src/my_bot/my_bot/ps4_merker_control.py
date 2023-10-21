import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from pyPS4Controller.controller import Controller
import time
global speed
speed = 10

def map_to_torque(current_num):
    global speed
    current_max = 32767

    scaling_factor = float(speed) / float(current_max)

    desired_number = float(current_num) * scaling_factor

    return desired_number

# ros2 topic pub /drive_controller/commands std_msgs/msg/Float64MultiArray "data: [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]"



def disconnect():
    print("Controller Disconnected")
    global ros2_node
    node.get_logger().info("Controller Disconnected")
    node.update_left_data([0.0, 0.0, 0.0])
    node.update_right_data([0.0, 0.0, 0.0])


class MyROS2Node(Node):
    def __init__(self):
        super().__init__('ps4_control')
        print("ROS2 Node Initialized")
        self.publisher = self.create_publisher(
            Float64MultiArray, '/drive_controller/commands', 10)
        self.right_data = [0.0, 0.0, 0.0]
        self.left_data = [0.0, 0.0, 0.0]
        self.controller = MyController(
        ros2_node=self, interface="/dev/input/js0", connecting_using_ds4drv=False, event_format="3Bh2b")
        self.controller.listen(on_disconnect=disconnect)
        safety_thread = threading.Thread(target=self.safety)
        safety_thread.start()
    
    def safety(self):
        time = time.time()
        while True:
            if self.controller.has_events():
                last_received = time.time()
                self.controller.clear_events()
            if last_received >= 0.5:
                self.update_left_data([0.0, 0.0, 0.0])
                self.update_right_data([0.0, 0.0, 0.0])
                
    def publish_data(self):
        print("Publishing Data")
        msg = Float64MultiArray()
        msg.data = self.right_data + self.left_data
        self.publisher.publish(msg)
        self.get_logger().info(f"Published data: {msg.data}")
        print(f"Published data: {msg.data}")

    def update_left_data(self, data):
        self.left_data = data
        self.publish_data()
        self.get_logger().info(f"Updated left data: {data}")

    def update_right_data(self, data):
        self.right_data = data
        self.publish_data()
        self.get_logger().info(f"Updated right data: {data}")


class MyController(Controller):
    def __init__(self, ros2_node, **kwargs):
        super().__init__(**kwargs)
        self.ros2_node = ros2_node
        
    def on_L3_up(self, value):
        torque = float(map_to_torque(value))
        self.ros2_node.update_left_data([-torque, -torque, -torque])

    def on_L3_down(self, value):
        torque = float(map_to_torque(value))
        self.ros2_node.update_left_data([-torque, -torque, -torque])

    def on_L3_y_at_rest(self):
        self.ros2_node.update_left_data([0.0, 0.0, 0.0])

    def on_R3_up(self, value):
        torque = float(map_to_torque(value))
        self.ros2_node.get_logger().info(f"R3 UP: {torque}")
        self.ros2_node.update_right_data([torque, torque, torque])

    def on_R3_down(self, value):
        torque = float(map_to_torque(value))
        self.ros2_node.get_logger().info(f"R3 Down: {torque}")
        self.ros2_node.update_right_data([torque, torque, torque])

    def on_R3_y_at_rest(self):
        self.ros2_node.update_right_data([0.0, 0.0, 0.0])

    # click triangle and x and circle to change speed
    def on_triangle_press(self):
        # changle global speed variable
        global speed
        speed = 40
        print(f" Current Speed: {speed}")
        self.ros2_node.get_logger().info(f"Current Speed: {speed}")

        
    def on_x_press(self):
        global speed
        speed = 10
        print(f" Current Speed: {speed}")
        self.ros2_node.get_logger().info(f"Current Speed: {speed}")

    def on_circle_press(self):
        global speed
        speed = 20
        print(f" Current Speed: {speed}")
        self.ros2_node.get_logger().info(f"Current Speed: {speed}")

    # when pressing the square button, the robot will stop
    def on_square_press(self):
        self.ros2_node.update_left_data([0.0, 0.0, 0.0])
        self.ros2_node.update_right_data([0.0, 0.0, 0.0])


def main(args=None):
    rclpy.init(args=args)
    global node
    node = MyROS2Node()
    
    rclpy.spin(node)
    rclpy.shutdown()

    disconnect()


if __name__ == '__main__':
    main()
