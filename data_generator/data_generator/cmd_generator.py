import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from prius_msgs.msg import Control
from sensor_msgs.msg import JointState
import sys
from enum import Enum
import random


class CommandMode(Enum):
    ACCELERATE = 1
    DECELERATE = 2


class CommandGenerator(Node):
    def __init__(self, node_name, name_space) -> None:
        super().__init__(node_name, namespace=name_space)

        # Declare parameters with default values
        self.declare_parameter('wheel_radius', 0.31265)  # meters
        self.declare_parameter('driving_wheels', [
            "front_left_wheel_joint",
            "front_right_wheel_joint",
            "rear_left_wheel_joint",
            "rear_right_wheel_joint"
        ])
        self.declare_parameter('throttle_max', 1.0)
        self.declare_parameter('brake_max', 1.0)
        self.declare_parameter('steer_max', 1.0)  # Full range for steering
        self.declare_parameter('update_rate', 0.02)  # seconds
        self.declare_parameter('accelerate_duration', 10.0)  # 10 seconds
        self.declare_parameter('decelerate_duration', 5.0)  # 5 seconds

        # Retrieve parameter values
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.driving_wheels = self.get_parameter('driving_wheels').get_parameter_value().string_array_value
        self.throttle_max = self.get_parameter('throttle_max').get_parameter_value().double_value
        self.brake_max = self.get_parameter('brake_max').get_parameter_value().double_value
        self.steer_max = self.get_parameter('steer_max').get_parameter_value().double_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.accelerate_duration = self.get_parameter('accelerate_duration').get_parameter_value().double_value
        self.decelerate_duration = self.get_parameter('decelerate_duration').get_parameter_value().double_value

        # Define QoS Profile
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=50
        )

        # Publisher for Control commands
        self.pub = self.create_publisher(Control, "control", qos_profile)

        # Initialize Control message
        self.command = Control()
        self.command.throttle = 0.0
        self.command.brake = 0.0
        self.command.steer = 0.0
        self.command.shift_gears = Control.NO_COMMAND

        # State Management
        self.current_mode = CommandMode.DECELERATE
        self.mode_timer = self.create_timer(self.accelerate_duration, self.switch_mode)
        self.command_timer = self.create_timer(self.update_rate, self.update_command)

        self.get_logger().info("CommandGenerator node initialized.")

    def switch_mode(self):
        """
        Switch between ACCELERATE and DECELERATE modes based on duration.
        """
        if self.current_mode == CommandMode.ACCELERATE:
            self.current_mode = CommandMode.DECELERATE
            self.mode_timer.cancel()
            self.mode_timer = self.create_timer(self.decelerate_duration, self.switch_mode)
            self.get_logger().info("Switched to DECELERATE mode.")
        else:
            self.current_mode = CommandMode.ACCELERATE
            self.mode_timer.cancel()
            self.mode_timer = self.create_timer(self.accelerate_duration, self.switch_mode)
            self.get_logger().info("Switched to ACCELERATE mode.")

    def smooth_change(self, target, current, step):
        """
        Smoothly change from current to target value with given step size.
        """
        if current < target:
            return min(current + step, target)
        elif current > target:
            return max(current - step, target)
        return current

    def update_command(self):
        """
        Update and publish Control commands based on the current mode and speed.
        """
        if self.current_mode == CommandMode.ACCELERATE:
            self.accelerate()
        elif self.current_mode == CommandMode.DECELERATE:
            self.decelerate()

        # Enforce safety limits
        self.command.throttle = max(0.0, min(self.command.throttle, self.throttle_max))
        self.command.brake = max(0.0, min(self.command.brake, self.brake_max))
        self.command.steer = max(-self.steer_max, min(self.command.steer, self.steer_max))

        # Publish the command
        self.pub.publish(self.command)

    def accelerate(self):
        """
        Accelerate with random throttle and steering.
        """
        target_throttle = random.uniform(0.3, self.throttle_max)  # Random throttle
        target_steer = random.uniform(-1.0, 1.0)  # Random steering within a reasonable range

        self.command.throttle = self.smooth_change(target_throttle, self.command.throttle, 0.02)
        self.command.steer = self.smooth_change(target_steer, self.command.steer, 0.05)
        self.command.brake = self.smooth_change(0.0, self.command.brake, 0.02)

    def decelerate(self):
        """
        Decelerate by reducing throttle and steering.
        """
        target_throttle = 0.0
        target_steer = random.uniform(-1.0, 1.0)
        target_brake = random.uniform(0.0, 1.0)

        self.command.throttle = self.smooth_change(target_throttle, self.command.throttle, 0.02)
        self.command.steer = self.smooth_change(target_steer, self.command.steer, 0.05)
        self.command.brake = self.smooth_change(target_brake, self.command.brake, 0.02)

def main(args=None):
    rclpy.init(args=args)
    node = CommandGenerator(node_name="cmd_generator", name_space="prius")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)
