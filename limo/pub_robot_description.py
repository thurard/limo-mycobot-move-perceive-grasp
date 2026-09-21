#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import rclpy.qos

class RobotDescriptionPublisher(Node):
    def __init__(self):
        super().__init__('robot_description_publisher')
        qos = rclpy.qos.QoSProfile(
            depth=1,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE
        )
        self.pub = self.create_publisher(String, '/robot_description', qos)
        with open('/home/agilex/limo_propre.urdf', 'r') as f:
            urdf = f.read()
        msg = String()
        msg.data = urdf
        self.pub.publish(msg)
        self.get_logger().info('robot_description publié !')

def main():
    rclpy.init()
    node = RobotDescriptionPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
