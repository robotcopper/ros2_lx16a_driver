import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from .lx16a_driver import LX16ADriver

class LX16ANode(Node):
    def __init__(self):
        super().__init__('lx16a_node')
        self.driver = LX16ADriver('/dev/ttyUSB0')  # Adapter au port série correct
        self.subscription = self.create_subscription(
            Float64,
            'servo_angle',
            self.set_servo_angle,
            10
        )
        self.get_logger().info('LX16A Node started')

    def set_servo_angle(self, msg):
        angle = msg.data
        self.get_logger().info(f'Setting servo to {angle} degrees')
        self.driver.move(1, angle)

    def destroy_node(self):
        self.driver.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LX16ANode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
