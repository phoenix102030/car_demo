import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from prius_msgs.msg import Control
from sensor_msgs.msg import JointState
import sys
from enum import Enum
import random  # Import the random module

class CommandMode(Enum):
    ACCELERATE = 1
    DECELERATE = 2
    HOLD_BRAKE = 3

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
        self.declare_parameter('steer_max', 1.0)  # As per URDF's max_steer
        self.declare_parameter('update_rate', 0.1)  # seconds
        self.declare_parameter('state_timer_duration', 10.0)  # seconds
        self.declare_parameter('speed_threshold', 0.1)  # m/s

        # Retrieve parameter values
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.driving_wheels = self.get_parameter('driving_wheels').get_parameter_value().string_array_value
        self.throttle_max = self.get_parameter('throttle_max').get_parameter_value().double_value
        self.brake_max = self.get_parameter('brake_max').get_parameter_value().double_value
        self.steer_max = self.get_parameter('steer_max').get_parameter_value().double_value
        self.update_rate = self.get_parameter('update_rate').get_parameter_value().double_value
        self.state_timer_duration = self.get_parameter('state_timer_duration').get_parameter_value().double_value
        self.speed_threshold = self.get_parameter('speed_threshold').get_parameter_value().double_value

        # Define QoS Profile with reliable reliability policy
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10  # Adjust as needed
        )

        # Publisher for Control commands
        self.pub = self.create_publisher(Control, "control", qos_profile)

        # Subscriber for JointState to get current wheel velocities
        self.sub = self.create_subscription(
            JointState,
            "/prius/joint_states",
            self.joint_state_callback,
            qos_profile
        )

        # Initialize Control message
        self.command = Control()
        self.command.throttle = 0.0
        self.command.brake = 0.0
        self.command.steer = 0.0
        self.command.shift_gears = Control.NO_COMMAND

        # State Management
        self.current_mode = CommandMode.ACCELERATE
        self.state_timer = self.create_timer(self.state_timer_duration, self.switch_mode)

        # Timer for updating commands
        self.command_timer = self.create_timer(self.update_rate, self.update_command)

        # Speed Tracking
        self.current_speed = 0.0  # m/s

        # Timer for holding brake
        self.hold_brake_timer = None  # Initialized when needed

        self.get_logger().info("CommandGenerator node has been initialized with the following parameters:")
        self.get_logger().info(f"Wheel Radius: {self.wheel_radius} m")
        self.get_logger().info(f"Driving Wheels: {self.driving_wheels}")
        self.get_logger().info(f"Throttle Max: {self.throttle_max}")
        self.get_logger().info(f"Brake Max: {self.brake_max}")
        self.get_logger().info(f"Steer Max: {self.steer_max}")
        self.get_logger().info(f"Update Rate: {self.update_rate} s")
        self.get_logger().info(f"State Timer Duration: {self.state_timer_duration} s")
        self.get_logger().info(f"Speed Threshold: {self.speed_threshold} m/s")

    def joint_state_callback(self, msg: JointState):
        """
        Callback to calculate speed from wheel velocities.
        """
        # Initialize variables
        total_angular_velocity = 0.0
        count = 0

        # Iterate through driving wheels and accumulate their angular velocities
        for wheel in self.driving_wheels:
            if wheel in msg.name:
                index = msg.name.index(wheel)
                angular_velocity = msg.velocity[index]  # rad/s
                total_angular_velocity += abs(angular_velocity)  # Use absolute value to get speed magnitude
                count += 1
            else:
                self.get_logger().warn(f"Joint '{wheel}' not found in JointState message.")

        if count > 0:
            average_angular_velocity = total_angular_velocity / count
            # Convert angular velocity to linear speed
            self.current_speed = average_angular_velocity * self.wheel_radius  # m/s
            self.get_logger().debug(f"Calculated Speed: {self.current_speed:.2f} m/s from {count} wheels")
        else:
            self.get_logger().warn("No driving wheel joints found in JointState message.")

    def switch_mode(self):
        """
        Switch between ACCELERATE and DECELERATE modes every state_timer_duration seconds.
        """
        if self.current_mode == CommandMode.ACCELERATE:
            self.current_mode = CommandMode.DECELERATE
            self.get_logger().info("Switched to DECELERATE mode.")
        elif self.current_mode == CommandMode.DECELERATE:
            # Check if the car has stopped
            if self.current_speed <= self.speed_threshold:
                # Transition to HOLD_BRAKE mode
                self.current_mode = CommandMode.HOLD_BRAKE
                self.get_logger().info("Speed is below threshold. Switching to HOLD_BRAKE mode.")
                self.activate_hold_brake()
            else:
                self.current_mode = CommandMode.ACCELERATE
                self.get_logger().info("Switched to ACCELERATE mode.")

    def activate_hold_brake(self):
        """
        Activate HOLD_BRAKE mode to apply brake for a short duration.
        """
        # Set mode to HOLD_BRAKE
        self.current_mode = CommandMode.HOLD_BRAKE

        # Create a one-shot timer to hold brake for 0.2 seconds
        self.hold_brake_timer = self.create_timer(0.2, self.finish_hold_brake)
        self.get_logger().info("Holding brake for 0.2 seconds.")

    def finish_hold_brake(self):
        """
        Finish HOLD_BRAKE mode and switch to ACCELERATE.
        """
        # Cancel the hold_brake_timer
        if self.hold_brake_timer is not None:
            self.hold_brake_timer.cancel()
            self.hold_brake_timer = None

        # Switch back to ACCELERATE mode
        self.current_mode = CommandMode.ACCELERATE
        self.get_logger().info("Exiting HOLD_BRAKE mode. Switching to ACCELERATE mode.")

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
        elif self.current_mode == CommandMode.HOLD_BRAKE:
            self.hold_brake()
        else:
            self.get_logger().warn(f"Unknown Command Mode: {self.current_mode}")

        # Enforce safety limits
        self.command.throttle = max(0.0, min(self.command.throttle, self.throttle_max))
        self.command.brake = max(0.0, min(self.command.brake, self.brake_max))
        self.command.steer = max(-1.0, min(self.command.steer, 1.0))  # Adjusted to full range -1 to +1

        # Publish the command
        self.pub.publish(self.command)
        self.get_logger().debug(f"Published Control Command: {self.command}")

    def accelerate(self):
        """
        Handle acceleration by setting a random throttle and random steering.
        """
        # Generate random throttle and steering values
        random_throttle = self.get_random_throttle()
        random_steer = self.get_random_steer()

        # Smoothly transition throttle and steer towards random targets
        self.command.throttle = self.smooth_change(random_throttle, self.command.throttle, 0.02)  # Smaller step for throttle
        self.command.steer = self.smooth_change(random_steer, self.command.steer, 0.1)  # Smaller step for steering
        self.command.brake = self.smooth_change(0.0, self.command.brake, 0.02)  # Smoothly set brake to 0 during acceleration

        self.get_logger().debug(f"Accelerating: Throttle set to {self.command.throttle:.3f}, Steer set to {self.command.steer:.3f}")

    def decelerate(self):
        """
        Handle deceleration by reducing throttle smoothly and setting brake to 0.
        """
        # Generate random throttle and steering values
        random_brake = self.get_random_brake()  
        random_steer = self.get_random_steer()

        # Smoothly transition throttle and steer towards random targets
        self.command.throttle = self.smooth_change(0.0, self.command.throttle, 0.02)  # Smaller step for throttle
        self.command.steer = self.smooth_change(0.0, self.command.steer, 0.1)  # Smaller step for steering
        self.command.brake = self.smooth_change(random_brake, self.command.brake, 0.02)  # Ensure brake remains 0 during deceleration

        self.get_logger().debug(f"Decelerating: Throttle set to {self.command.throttle:.3f}, Steer set to {self.command.steer:.3f}, Brake set to {self.command.brake:.3f}")

    def hold_brake(self):
        """
        Maintain brake application during HOLD_BRAKE mode with random steering.
        """
        # Generate random steering value
        random_steer = self.get_random_steer()

        # Smoothly transition brake and steer towards targets
        self.command.throttle = self.smooth_change(0.0, self.command.throttle, 0.02)  # Ensure throttle is 0
        self.command.brake = self.smooth_change(self.brake_max * 0.7, self.command.brake, 0.02)  # Apply strong brake
        self.command.steer = self.smooth_change(random_steer, self.command.steer, 0.1)  # Smaller step for steering

        self.get_logger().debug(f"Holding Brake: Brake set to {self.command.brake:.3f}, Steer set to {self.command.steer:.3f}")

    def get_random_throttle(self):
        """Generate a random throttle value between 0.0 and throttle_max."""
        return random.uniform(0.0, self.throttle_max)

    def get_random_brake(self):
        """Generate a random brake value between 0.0 and brake_max."""
        return random.uniform(0.0, self.brake_max)

    def get_random_steer(self):
        """Generate a random steering value between -1.0 and 1.0."""
        return random.uniform(-1.0, 1.0)  # Adjusted to full steering range

    def destroy_node_callback(self):
        """
        Ensure that the vehicle receives neutral commands upon node shutdown.
        """
        self.command.throttle = 0.0
        self.command.brake = 0.0
        self.command.steer = 0.0
        self.command.shift_gears = Control.NO_COMMAND
        self.pub.publish(self.command)
        self.get_logger().info("Published neutral Control Command upon shutdown.")

    def destroy_node(self):
        """
        Override destroy_node to include shutdown behavior.
        """
        self.destroy_node_callback()
        super().destroy_node()

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
