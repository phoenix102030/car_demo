import rclpy
from rclpy.node import Node
from prius_msgs.msg import Control
from nav_msgs.msg import Odometry
import csv
from threading import Lock


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
                                   'position_x', 'position_y', 'orientation_z',
                                   'linear_velocity_x', 'angular_velocity_z'])

        # Buffer to store recent Control message
        self.control_buffer = None
        self.buffer_lock = Lock()

        # Variables for logging at 0.1-second intervals
        self.last_log_time = None
        self.log_interval = 0.1  # 0.1 seconds

        # Subscriber for Odometry
        self.create_subscription(Odometry, '/prius/ground_truth', self.odometry_callback, 10)

        # Subscriber for Control messages
        self.create_subscription(Control, '/prius/control', self.control_callback, 10)

        self.get_logger().info(f"Data Recorder initialized, logging to {self.output_file}")

    def odometry_callback(self, msg: Odometry):
        """Callback to handle Odometry messages."""
        with self.buffer_lock:
            # Extract timestamp from Odometry message
            current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            # Check if enough time has passed since the last log
            if self.last_log_time is not None and current_time - self.last_log_time < self.log_interval - 0.003:
                return

            # Extract data from Odometry message
            position_x = msg.pose.pose.position.x
            position_y = msg.pose.pose.position.y
            orientation_z = msg.pose.pose.orientation.z
            linear_velocity_x = msg.twist.twist.linear.x
            angular_velocity_z = msg.twist.twist.angular.z

            # Use the latest Control message data
            throttle, brake, steer = 0.0, 0.0, 0.0
            if self.control_buffer:
                throttle = self.control_buffer.throttle
                brake = self.control_buffer.brake
                steer = self.control_buffer.steer

            # Format the timestamp
            formatted_timestamp = f"{msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}"

            # Write to CSV
            self.csv_writer.writerow([
                formatted_timestamp,
                throttle,
                brake,
                steer,
                position_x,
                position_y,
                orientation_z,
                linear_velocity_x,
                angular_velocity_z
            ])

            # Flush the CSV buffer to ensure data is written immediately
            self.csv_file.flush()

            self.get_logger().info(f"Logged data at {formatted_timestamp}")

            # Update the last log time
            self.last_log_time = current_time

    def control_callback(self, msg: Control):
        """Callback to store the latest control data."""
        with self.buffer_lock:
            self.control_buffer = msg
            self.get_logger().debug('Control data received.')

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
