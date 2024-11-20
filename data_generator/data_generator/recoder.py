import rclpy
from rclpy.node import Node
from prius_msgs.msg import Control
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import csv

class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder')

        # Declare file name for CSV logging
        self.declare_parameter('output_file', 'training_data.csv')
        self.output_file = self.get_parameter('output_file').value

        # Open CSV file for writing
        self.csv_file = open(self.output_file, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['timestamp', 'throttle', 'brake', 'steer', 
                                   'joint_states', 'position_x', 'position_y', 'orientation_z', 
                                   'linear_velocity_x', 'angular_velocity_z'])

        # Initialize data variables
        self.control_data = None
        self.joint_state_data = None
        self.odometry_data = None

        # Subscribers
        self.create_subscription(Control, '/prius/control', self.control_callback, 10)
        self.create_subscription(JointState, '/prius/joint_states', self.joint_state_callback, 10)
        self.create_subscription(Odometry, '/prius/ground_truth', self.odometry_callback, 10)

        # Timer to periodically log data
        self.create_timer(0.1, self.log_data)  # Log every 0.1 seconds

        self.get_logger().info(f"Data Recorder initialized, logging to {self.output_file}")

    def control_callback(self, msg: Control):
        """Callback to store control data."""
        self.control_data = msg

    def joint_state_callback(self, msg: JointState):
        """Callback to store joint state data."""
        self.joint_state_data = msg

    def odometry_callback(self, msg: Odometry):
        """Callback to store odometry data."""
        self.odometry_data = msg

    def log_data(self):
        """Log data to CSV if all data is available."""
        if self.control_data and self.joint_state_data and self.odometry_data:
            timestamp = self.get_clock().now().to_msg()
            joint_states = ','.join([f"{v:.2f}" for v in self.joint_state_data.velocity])

            # Extract odometry data
            position_x = self.odometry_data.pose.pose.position.x
            position_y = self.odometry_data.pose.pose.position.y
            orientation_z = self.odometry_data.pose.pose.orientation.z
            linear_velocity_x = self.odometry_data.twist.twist.linear.x
            angular_velocity_z = self.odometry_data.twist.twist.angular.z

            # Write to CSV
            self.csv_writer.writerow([
                f"{timestamp.sec}.{timestamp.nanosec}",
                self.control_data.throttle,
                self.control_data.brake,
                self.control_data.steer,
                joint_states,
                position_x,
                position_y,
                orientation_z,
                linear_velocity_x,
                angular_velocity_z
            ])
            self.get_logger().info("Logged data row")

    def destroy_node(self):
        """Override destroy_node to close the CSV file."""
        self.csv_file.close()
        self.get_logger().info(f"Closed file {self.output_file}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('KeyboardInterrupt, shutting down node.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
