#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

class AlgorithmNode(Node):
    def __init__(self):
        super().__init__('motion_algorithm')

        # Declare parameters with default values
        self.declare_parameter('forward_speed', 0.11)
        self.declare_parameter('threshold', 1.0)

        # Initialize from parameter values
        self.forward_speed = self.get_parameter('forward_speed').get_parameter_value().double_value
        self.threshold     = self.get_parameter('threshold').get_parameter_value().double_value

        # Add parameter change callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # LiDAR subscriber
        self.lidar_subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        # Velocity command publisher
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)

        # Steering servo angle publisher
        self.servo_publisher = self.create_publisher(Int16, '/servo1', 10)

        # Constants for servo positions
        self.STRAIGHT = 90
        self.LEFT     = 0
        self.RIGHT    = 180

    # Dynamic change of variable values on node runtime
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'forward_speed' and param.type_ == Parameter.Type.DOUBLE:
                self.forward_speed = param.value
                self.get_logger().info(f"Updated forward_speed to {self.forward_speed}")
            elif param.name == 'threshold' and param.type_ == Parameter.Type.DOUBLE:
                self.threshold = param.value
                self.get_logger().info(f"Updated threshold to {self.threshold}")
        result = SetParametersResult()
        result.successful = True
        return result

    def lidar_callback(self, msg: LaserScan):
        if not msg.ranges:
            self.get_logger().warn("No LiDAR data received!")
            return

        n     = len(msg.ranges)
        mid   = n // 2
        left  = min(n - 1, mid + n // 4)
        right = max(0, mid - n // 4)

        c_dist = msg.ranges[mid]
        l_dist = msg.ranges[left]
        r_dist = msg.ranges[right]

        twist     = Twist()
        servo_msg = Int16(data=self.STRAIGHT)

        # Obstacle check: anything closer than threshold triggers avoidance
        if c_dist < self.threshold:
            if l_dist >= self.threshold and r_dist >= self.threshold:
                servo_msg.data = self.RIGHT
                self.get_logger().info(" -> Obstacle ahead; BOTH sides clear — choosing RIGHT")
            elif l_dist < self.threshold <= r_dist:
        # left is blocked, right is clear → steer RIGHT
                servo_msg.data = self.RIGHT
                self.get_logger().info(" -> Steering RIGHT")
            elif r_dist < self.threshold <= l_dist:
        # right is blocked, left is clear → steer LEFT
                servo_msg.data = self.LEFT
                self.get_logger().info(" -> Steering LEFT")
            else:
        # both sides blocked: pick the side with the larger gap
                if l_dist > r_dist:
                    servo_msg.data = self.LEFT
                    self.get_logger().info(" -> BOTH sides blocked — choosing LEFT (larger gap)")
                else:
                    servo_msg.data = self.RIGHT
                    self.get_logger().info(" -> BOTH sides blocked — choosing RIGHT (larger gap)")

    # Always full forward when avoiding
                    twist.linear.x = self.forward_speed

        else:
    # No obstacle: full speed straight
            twist.linear.x = self.forward_speed
            servo_msg.data = self.STRAIGHT
            self.get_logger().info("Moving forward")

        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)
        self.servo_publisher.publish(servo_msg)


def main(args=None):
    rclpy.init(args=args)
    node = AlgorithmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

