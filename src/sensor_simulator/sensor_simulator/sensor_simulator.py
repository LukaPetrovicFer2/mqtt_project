import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, UInt8
import random
import time

class SensorSimulator(Node):
    def __init__(self):
        super().__init__('sensor_simulator')

        # Float64 topics
        self.pub_depth = self.create_publisher(Float64, 'depth', 10)
        self.pub_depth_variance = self.create_publisher(Float64, 'depth_variance', 10)
        self.pub_altitude = self.create_publisher(Float64, 'altitude', 10)
        self.pub_altitude_covariance = self.create_publisher(Float64, 'altitude_covariance', 10)

        # Float64MultiArray
        self.pub_velocity_covariance = self.create_publisher(Float64MultiArray, 'velocity_covariance', 10)

        # UInt8 topic
        self.pub_velocity_reference = self.create_publisher(UInt8, 'velocity_reference', 10)

        # Timer to publish at 2 Hz
        self.timer = self.create_timer(0.5, self.publish_random_values)

    def publish_random_values(self):
        self.pub_depth.publish(Float64(data=random.uniform(1.0, 10.0)))
        self.pub_depth_variance.publish(Float64(data=random.uniform(0.0, 0.5)))
        self.pub_altitude.publish(Float64(data=random.uniform(0.0, 100.0)))
        self.pub_altitude_covariance.publish(Float64(data=random.uniform(0.0, 1.0)))

        msg = Float64MultiArray()
        msg.data = [random.uniform(0.0, 1.0) for _ in range(9)]
        self.pub_velocity_covariance.publish(msg)

        self.pub_velocity_reference.publish(UInt8(data=random.randint(0, 255)))

        self.get_logger().info("Published simulated sensor values.")

def main(args=None):
    rclpy.init(args=args)
    node = SensorSimulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

