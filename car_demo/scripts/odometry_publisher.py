#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from gazebo_msgs.srv import GetModelState

class OdometryPublisher(Node):
    def __init__(self):
        super().__init__('odom_pub')

        # Parameters
        self.declare_parameter('robot_name', 'prius')
        self.declare_parameter('odom_topic', '/my_odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('rate', 10.0)

        self.robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.rate_value = self.get_parameter('rate').get_parameter_value().double_value

        # Create a publisher for Odometry
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        # Create a client for the GetModelState service
        self.client = self.create_client(GetModelState, '/gazebo/get_model_state')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /gazebo/get_model_state not available, waiting...')

        # Create a timer to call the service periodically
        self.timer = self.create_timer(1.0 / self.rate_value, self.timer_callback)

    def timer_callback(self):
        # Create a request
        request = GetModelState.Request()
        request.model_name = self.robot_name
        request.relative_entity_name = 'world'

        # Call the service asynchronously
        future = self.client.call_async(request)
        future.add_done_callback(self.service_callback)

    def service_callback(self, future):
        try:
            response = future.result()
            if response.success:
                # Create Odometry message
                odom_msg = Odometry()
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = self.odom_frame

                # Set pose
                odom_msg.pose.pose = response.pose

                # Set twist
                odom_msg.twist.twist = response.twist

                # Publish the odometry message
                self.odom_pub.publish(odom_msg)
            else:
                self.get_logger().warn('Failed to get model state: {}'.format(response.status_message))
        except Exception as e:
            self.get_logger().error('Service call failed: {}'.format(e))

def main(args=None):
    rclpy.init(args=args)
    node = OdometryPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
