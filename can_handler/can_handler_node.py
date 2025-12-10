#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from can_msgs.msg import Frame
import can
import struct


class CANHandlerNode(Node):
    def __init__(self):
        super().__init__('can_handler_node')
        
        # Declare parameters
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('motor_current_topic', '/motor_current')
        self.declare_parameter('vehicle_speed_topic', '/vehicle_speed')
        self.declare_parameter('to_can_topic', '/to_can')
        
        # Get parameters
        can_interface = self.get_parameter('can_interface').value
        motor_current_topic = self.get_parameter('motor_current_topic').value
        vehicle_speed_topic = self.get_parameter('vehicle_speed_topic').value
        to_can_topic = self.get_parameter('to_can_topic').value
        
        # Create publishers with topics under node namespace
        self.motor_current_pub = self.create_publisher(
            Float32, 
            f'{self.get_name()}{motor_current_topic}',
            10
        )
        self.vehicle_speed_pub = self.create_publisher(
            Float32, 
            f'{self.get_name()}{vehicle_speed_topic}',
            10
        )
        
        # Create subscriber for outgoing CAN messages
        self.to_can_sub = self.create_subscription(
            Frame,
            to_can_topic,
            self.to_can_callback,
            10
        )
        
        # Initialize CAN bus
        try:
            self.bus = can.interface.Bus(
                channel=can_interface,
                bustype='socketcan',
                bitrate=250000  # Adjust if needed
            )
            self.get_logger().info(f'CAN bus initialized on {can_interface}')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize CAN bus: {e}')
            raise
        
        # Create a timer to read CAN messages
        self.timer = self.create_timer(0.001, self.read_can_messages)  # 1ms polling
        
        self.get_logger().info('CAN Handler Node has been started.')
        self.get_logger().info(f'Listening for:')
        self.get_logger().info(f'  - Motor Current (ID 0x26E)')
        self.get_logger().info(f'  - Vehicle Speed (ID 0x18FEF100)')
        self.get_logger().info(f'Subscribing to: {to_can_topic} for outgoing CAN messages')

    def read_can_messages(self):
        """Read available CAN messages and process them"""
        try:
            # Non-blocking read with timeout
            msg = self.bus.recv(timeout=0.0)
            
            if msg is not None:
                self.process_can_message(msg)
                
        except Exception as e:
            self.get_logger().error(f'Error reading CAN message: {e}')

    def process_can_message(self, msg):
        """Process incoming CAN messages based on their ID"""
        
        # Motor Current - ID 0x26E
        if msg.arbitration_id == 0x26E:
            self.process_motor_current(msg)
        
        # Vehicle Speed - ID 0x18FEF100
        elif msg.arbitration_id == 0x18FEF100:
            self.process_vehicle_speed(msg)

    def process_motor_current(self, msg):
        """
        Extract motor current from ID 0x26E. First 2 bytes contain the value in two's complement        
        """
        if len(msg.data) < 2:
            self.get_logger().warn('Motor current message too short')
            return
        
        # Extract first 2 bytes (16-bit value) as signed integer
        signed_value = struct.unpack('<h', msg.data[0:2])[0]  # Little endian signed short
        
        # Publish the current value
        current_msg = Float32()
        # current_msg.header.stamp = self.can_to_ros_time(msg.timestamp)
        # current_msg.header.frame_id = 'base_link'
        current_msg.data = float(signed_value)
        
        self.motor_current_pub.publish(current_msg)
        
        self.get_logger().debug(f'Motor Current: {signed_value}')

    def process_vehicle_speed(self, msg):
        """
        Extract vehicle speed from ID 0x18FEF100
        Bytes 2 and 3 contain speed as: byte[3] + byte[2]/256.0
        Units: km/h
        """
        if len(msg.data) < 4:
            self.get_logger().warn('Vehicle speed message too short')
            return
        
        # Extract bytes 2 and 3
        fractional_part = msg.data[1]  # Byte 2: fractional part
        integer_part = msg.data[2]     # Byte 3: integer part
        
        # Combine: integer + fractional/256
        speed = float(integer_part) + (float(fractional_part) / 256.0)
        # Combine as uint16
       
        # Publish the speed value
        speed_msg = Float32()
        # speed_msg.header.stamp = self.can_to_ros_time(msg.timestamp)
        # speed_msg.header.frame_id = 'base_link'
        speed_msg.data = speed
        
        self.vehicle_speed_pub.publish(speed_msg)
        
        self.get_logger().debug(f'Vehicle Speed: {speed:.2f} km/h')

    def to_can_callback(self, msg: Frame):
        """
        Send CAN message received from ROS2 topic.
        Uses the standard can_msgs/Frame message format.
        """
        try:
            # Convert can_msgs/Frame to python-can Message
            can_msg = can.Message(
                arbitration_id=msg.id,
                data=bytes(msg.data[:msg.dlc]),
                is_extended_id=msg.is_extended,
                is_remote_frame=msg.is_rtr,
                is_error_frame=msg.is_error
            )
            
            # Send the message
            self.bus.send(can_msg)
            
            self.get_logger().debug(
                f'Sent CAN message: ID=0x{msg.id:X}, DLC={msg.dlc}, '
                f'Data={msg.data[:msg.dlc]}'
            )
            
        except can.CanError as e:
            self.get_logger().error(f'Failed to send CAN message: {e}')
        except Exception as e:
            self.get_logger().error(f'Error in to_can_callback: {e}')

    def destroy_node(self):
        """Clean up CAN bus on shutdown"""
        if hasattr(self, 'bus'):
            self.bus.shutdown()
        super().destroy_node()

    def can_to_ros_time(self, can_timestamp):
        """Convert CAN timestamp to ROS2 time"""
        sec = int(can_timestamp)
        nsec = int((can_timestamp - sec) * 1e9)
        return rclpy.time.Time(seconds=sec, nanoseconds=nsec).to_msg()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CANHandlerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
