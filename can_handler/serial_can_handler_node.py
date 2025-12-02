#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import struct


class SerialCANHandlerNode(Node):
    def __init__(self):
        super().__init__('serial_can_handler_node')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 921600)
        self.declare_parameter('motor_current_topic', '/motor_current')
        self.declare_parameter('vehicle_speed_topic', '/vehicle_speed')
        
        # Get parameters
        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        motor_current_topic = self.get_parameter('motor_current_topic').value
        vehicle_speed_topic = self.get_parameter('vehicle_speed_topic').value
        
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
        
        # Initialize Serial connection
        try:
            self.serial_conn = serial.Serial(
                port=serial_port,
                baudrate=baud_rate,
                timeout=1.0
            )
            self.get_logger().info(f'Serial port opened: {serial_port} at {baud_rate} baud')
            
            # Wait for Arduino to be ready
            while self.serial_conn.in_waiting == 0:
                pass
            
            # Read the "CAN_READY" message
            ready_msg = self.serial_conn.readline().decode('utf-8').strip()
            self.get_logger().info(f'Arduino says: {ready_msg}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise
        
        # Create a timer to read serial messages
        self.timer = self.create_timer(0.001, self.read_serial_messages)  # 1ms polling
        
        self.get_logger().info('Serial CAN Handler Node has been started.')
        self.get_logger().info(f'Listening for:')
        self.get_logger().info(f'  - Motor Current (ID 0x26E)')
        self.get_logger().info(f'  - Vehicle Speed (ID 0x18FEF100)')

    def read_serial_messages(self):
        """Read available serial messages and process them"""
        try:
            # Check if data is available
            if self.serial_conn.in_waiting > 0:
                line = self.serial_conn.readline().decode('utf-8').strip()
                
                if line:
                    self.process_serial_line(line)
                    
        except Exception as e:
            self.get_logger().error(f'Error reading serial message: {e}')

    def process_serial_line(self, line):
        """
        Process incoming serial line in CSV format: ID,DLC,DATA0,DATA1,DATA2,...
        Example: 26E,2,1A,2B
        """
        try:
            parts = line.split(',')
            
            if len(parts) < 2:
                return
            
            # Parse CAN ID (hex) and DLC (decimal)
            can_id = int(parts[0], 16)
            dlc = int(parts[1])
            
            # Parse data bytes (hex)
            data = []
            for i in range(2, min(2 + dlc, len(parts))):
                data.append(int(parts[i], 16))
            
            # Process based on CAN ID
            if can_id == 0x26E:
                self.process_motor_current(data)
            elif can_id == 0x18FEF100:
                self.process_vehicle_speed(data)
                
        except ValueError as e:
            self.get_logger().debug(f'Failed to parse line: {line} - {e}')

    def process_motor_current(self, data):
        """
        Extract motor current from ID 0x26E. First 2 bytes contain the value in two's complement        
        """
        if len(data) < 2:
            self.get_logger().warn('Motor current message too short')
            return
        
        # Combine two bytes into 16-bit signed integer (little endian)
        signed_value = struct.unpack('<h', bytes(data[0:2]))[0]
        
        # Publish the current value
        current_msg = Float32()
        current_msg.data = float(signed_value)
        
        self.motor_current_pub.publish(current_msg)
        
        self.get_logger().debug(f'Motor Current: {signed_value}')

    def process_vehicle_speed(self, data):
        """
        Extract vehicle speed from ID 0x18FEF100
        Bytes 2 and 3 contain speed as: byte[3] + byte[2]/256.0
        Units: km/h
        """
        if len(data) < 4:
            self.get_logger().warn('Vehicle speed message too short')
            return
        
        # Extract bytes 2 and 3 (index 1 and 2 in the data array)
        fractional_part = data[1]  # Byte 2: fractional part
        integer_part = data[2]     # Byte 3: integer part
        
        # Combine: integer + fractional/256
        speed = float(integer_part) + (float(fractional_part) / 256.0)
        
        # Publish the speed value
        speed_msg = Float32()
        speed_msg.data = speed
        
        self.vehicle_speed_pub.publish(speed_msg)
        
        self.get_logger().debug(f'Vehicle Speed: {speed:.2f} km/h')

    def destroy_node(self):
        """Clean up serial connection on shutdown"""
        if hasattr(self, 'serial_conn') and self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info('Serial port closed')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SerialCANHandlerNode()
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
