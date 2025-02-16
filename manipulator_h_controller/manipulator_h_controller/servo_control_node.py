import rclpy
from rclpy.node import Node
import serial
from std_msgs.msg import Int32 

'''
Technical Details servo Savox SC1257TG


Operating Temperature: 14° - 122°F (-10° - 50°C)
Running Current (no load): 100mA @4.8V, 120mA @6.0V
Stall Current (at locked): 4000mA @4.8V, 5000mA @6.0V
Idle Current: 5mA @4.8V, 5mA @6.0V
Wire Length: 250 +/- 5mm
Output Gear Spline: 25 Tooth
Bearings: 2BB
Operating Travel: 100° (1000 → 2000 µ second)
Neutral Position: 1500 µ second
Pulse Width Range: 800 → 2200 µ second
Maximum Travel: Appx 130° (900 → 2100 µ second)
Refresh Rate: 333 Hz
Motor Type: Coreless
'''

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
        self.shutdown_pulse = 3000
        
        
        if 0 <= angle <= 180:  # Ensure the angle is valid
            self.serial_conn.write(f"{us}\n".encode())
            self.get_logger().info(f"Sent angle: {angle}, sent us: {us}")
        elif angle == 3000:
            self.get_logger().info(f"Shutting down servo/. Sent us: {self.shutdown_pulse}")
            self.serial_conn.write(f"{self.shutdown_pulse}\n".encode())
        
        else:            
            self.get_logger().warn(f"Invalid angle: {angle}")
            
    def angle2us(self, angle):
        # Assuming a typical servo operates between 1000us and 2000us
        min_pulse = 1000
        max_pulse = 2000
        min_angle = 0
        max_angle = 100
        
       

        if angle < min_angle:
            return min_pulse
        elif angle > max_angle:
            return max_pulse
        else:
            return int((angle) * (max_pulse - min_pulse) / (max_angle- min_angle) + min_pulse)
        
        

def main(args=None):
    rclpy.init(args=args)
    node = ServoControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()