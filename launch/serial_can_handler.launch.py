from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

DEBUG = os.getenv('DEBUG', default='False').lower() == 'true' or os.getenv('DEBUG', default='False').lower() == '1'

def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for Arduino (e.g., /dev/ttyUSB0, /dev/ttyACM0)'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Serial baud rate'
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
    
    # Create the serial_can_handler_node
    serial_can_handler_node = Node(
        package='can_handler',
        executable='serial_can_handler_node.py',
        name='serial_can_handler_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'motor_current_topic': LaunchConfiguration('motor_current_topic'),
            'vehicle_speed_topic': LaunchConfiguration('vehicle_speed_topic'),
        }],
        arguments=['--ros-args', '--log-level', log_level],
        emulate_tty=True,
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        motor_current_topic_arg,
        vehicle_speed_topic_arg,
        serial_can_handler_node,
    ])
