import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Int32 

class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')
        self.declare_parameter('port', '/dev/ttyACM0')  # Default Arduino port
        self.declare_parameter('baudrate', 9600)

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value

        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
            self.get_logger().info(f"Connected to Arduino on {self.port}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            exit(1)

        self.subscription = self.create_subscription(
            Int32,  # Assuming you use std_msgs/Int32 for the angle
            'servo_angle',
            self.servo_callback,
            10
        )

    def servo_callback(self, msg):
        angle = msg.data
        us = self.angle2us(angle)
        if 0 <= angle <= 180:  # Ensure the angle is valid
            self.serial_conn.write(f"{angle}\n".encode())
            self.get_logger().info(f"Sent angle: {angle}")
        else:
            self.get_logger().warn(f"Invalid angle: {angle}")
            
    def angle2us(self, angle):
        # Assuming a typical servo operates between 1000us and 2000us
        min_pulse = 900
        max_pulse = 2200
        min_angle = 30
        max_angle = 170

        if angle < min_angle:
            return min_pulse
        elif angle > max_angle:
            return max_pulse
        else:
            return int((angle - min_angle) * (max_pulse - min_pulse) / (max_angle- min_angle) + min_pulse)
        
        
        

def main(args=None):
    rclpy.init(args=args)
    node = ServoControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()