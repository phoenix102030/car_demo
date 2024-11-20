from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare output file parameter for the data recorder
    declare_output_file_arg = DeclareLaunchArgument(
        'output_file',
        default_value='training_data.csv',
        description='Path to the output CSV file for the data recorder'
    )

    # Node to generate commands
    cmd_generator_node = Node(
        package='data_generator',  # Replace with your package name
        executable='generator',  # Replace with your cmd generator executable
        name='cmd_generator',
        namespace='prius',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # Node to record training data
    data_recorder_node = Node(
        package='data_generator',  # Replace with your package name
        executable='recoder',  # Replace with your recorder executable
        name='data_recorder',
        namespace='prius',
        parameters=[
            {'use_sim_time': True},
            {'output_file': 'my_training_data.csv'}
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_output_file_arg,
        cmd_generator_node,
        data_recorder_node
    ])
