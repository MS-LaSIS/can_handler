from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

DEBUG = os.getenv('DEBUG', default='False').lower() == 'true' or os.getenv('DEBUG', default='False').lower() == '1'

def generate_launch_description():
    # Declare launch arguments
    can_interface_arg = DeclareLaunchArgument(
        'can_interface',
        default_value='can0',
        description='CAN interface to use (e.g., can0, vcan0)'
    )
    
    motor_current_topic_arg = DeclareLaunchArgument(
        'motor_current_topic',
        default_value='/motor_current',
        description='Topic name for motor current messages'
    )
    
    vehicle_speed_topic_arg = DeclareLaunchArgument(
        'vehicle_speed_topic',
        default_value='/vehicle_speed',
        description='Topic name for vehicle speed messages'
    )
    log_level = 'debug' if DEBUG else 'info'
    # Create the can_handler_node
    can_handler_node = Node(
        package='can_handler',
        executable='can_handler_node.py',
        name='can_handler_node',
        output='screen',
        parameters=[{
            'can_interface': LaunchConfiguration('can_interface'),
            'motor_current_topic': LaunchConfiguration('motor_current_topic'),
            'vehicle_speed_topic': LaunchConfiguration('vehicle_speed_topic'),
        }],
        arguments=['--ros-args', '--log-level', log_level],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        can_interface_arg,
        motor_current_topic_arg,
        vehicle_speed_topic_arg,
        can_handler_node,
    ])