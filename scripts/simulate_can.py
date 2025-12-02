#!/usr/bin/env python3
"""
CAN bus simulator for testing the can_handler_node.
Simulates motor current (ID 0x26E) and vehicle speed (ID 0x18FEF100) messages.
Can read from candump log files or generate simulated data.
"""

import can
import time
import struct
import random
import argparse
import re


def parse_candump_line(line):
    """
    Parse a candump log line.
    Format: (timestamp) interface can_id#data
    Example: (1760709380.398842) can0 726#00
    """
    # Match the candump format: (timestamp) interface can_id#data
    match = re.match(r'\(([0-9.]+)\)\s+(\w+)\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)', line)
    
    if not match:
        return None
    
    timestamp_str, interface, can_id_str, data_str = match.groups()
    
    timestamp = float(timestamp_str)
    can_id = int(can_id_str, 16)
    
    # Convert hex string to bytes
    if data_str:
        data = bytes.fromhex(data_str)
    else:
        data = b''
    
    return {
        'timestamp': timestamp,
        'interface': interface,
        'arbitration_id': can_id,
        'data': data
    }


def simulate_motor_current(bus, current_value):
    """
    Simulate motor current message (ID 0x26E)
    First 2 bytes: signed 16-bit integer (two's complement)
    """
    # Pack as signed 16-bit little-endian
    data = struct.pack('<h', current_value)
    # Pad with zeros to make 8 bytes (standard CAN frame)
    data = data + b'\x00' * 6
    
    msg = can.Message(
        arbitration_id=0x26E,
        data=data,
        is_extended_id=False
    )
    
    try:
        bus.send(msg)
        print(f"Sent Motor Current: {current_value} A (0x{current_value:04X})")
        return True
    except can.CanError as e:
        print(f"Failed to send motor current: {e}")
        return False


def simulate_vehicle_speed(bus, speed_kmh):
    """
    Simulate vehicle speed message (ID 0x18FEF100)
    Bytes 2-3: speed as integer_part (byte 3) + fractional_part (byte 2) / 256.0
    Units: km/h
    """
    # Split into integer and fractional parts
    integer_part = int(speed_kmh)
    fractional_part = int((speed_kmh - integer_part) * 256.0)
    
    # Create 8-byte data array
    data = bytearray(8)
    data[0] = 0x00  # Byte 0: unused
    data[1] = 0x00  # Byte 1: unused
    data[2] = fractional_part  # Byte 2: fractional part
    data[3] = integer_part      # Byte 3: integer part
    # Bytes 4-7: unused
    
    msg = can.Message(
        arbitration_id=0x18FEF100,
        data=bytes(data),
        is_extended_id=True  # Extended ID (29-bit)
    )
    
    try:
        bus.send(msg)
        print(f"Sent Vehicle Speed: {speed_kmh:.2f} km/h (int={integer_part}, frac={fractional_part})")
        return True
    except can.CanError as e:
        print(f"Failed to send vehicle speed: {e}")
        return False


def replay_candump_log(bus, log_file, rate_multiplier=1.0, loop=False):
    """
    Replay a candump log file to the CAN bus.
    
    Args:
        bus: CAN bus interface
        log_file: Path to candump log file
        rate_multiplier: Speed multiplier (1.0 = real-time, 2.0 = 2x speed, etc.)
        loop: Whether to loop the log file continuously
    """
    print(f"Reading candump log from: {log_file}")
    print(f"Rate multiplier: {rate_multiplier}x")
    
    while True:
        try:
            with open(log_file, 'r') as f:
                messages = []
                
                # Parse all messages
                for line in f:
                    line = line.strip()
                    if not line or line.startswith('#'):
                        continue
                    
                    parsed = parse_candump_line(line)
                    if parsed:
                        messages.append(parsed)
                
                if not messages:
                    print("No valid CAN messages found in log file")
                    return
                
                print(f"Loaded {len(messages)} CAN messages")
                
                # Get the first timestamp as reference
                start_timestamp = messages[0]['timestamp']
                start_time = time.time()
                
                # Replay messages
                for i, msg_data in enumerate(messages):
                    # Calculate when this message should be sent
                    relative_time = msg_data['timestamp'] - start_timestamp
                    target_time = start_time + (relative_time / rate_multiplier)
                    
                    # Wait until it's time to send this message
                    current_time = time.time()
                    sleep_time = target_time - current_time
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                    
                    # Create and send CAN message
                    msg = can.Message(
                        arbitration_id=msg_data['arbitration_id'],
                        data=msg_data['data'],
                        is_extended_id=(msg_data['arbitration_id'] > 0x7FF),
                        timestamp=time.time()
                    )
                    
                    try:
                        bus.send(msg)
                        
                        # Print progress every 100 messages
                        if (i + 1) % 100 == 0:
                            print(f"Replayed {i + 1}/{len(messages)} messages...")
                    except can.CanError as e:
                        print(f"Failed to send message: {e}")
                
                print(f"\nFinished replaying {len(messages)} messages")
                
                if not loop:
                    break
                else:
                    print("Looping log file...\n")
                    
        except FileNotFoundError:
            print(f"Error: Log file not found: {log_file}")
            return
        except KeyboardInterrupt:
            print("\nReplay stopped by user")
            break
        except Exception as e:
            print(f"Error reading log file: {e}")
            return


def main():
    parser = argparse.ArgumentParser(description='Simulate CAN messages for testing')
    parser.add_argument('--interface', default='vcan0', help='CAN interface (default: vcan0)')
    parser.add_argument('--rate', type=float, default=10.0, help='Message rate in Hz for simulation mode (default: 10)')
    parser.add_argument('--mode', choices=['static', 'random', 'sweep', 'log'], default='random',
                       help='Simulation mode: static, random, sweep, or log (replay from file)')
    parser.add_argument('--current', type=int, default=50, help='Static current value in A (default: 50)')
    parser.add_argument('--speed', type=float, default=30.0, help='Static speed value in km/h (default: 30.0)')
    parser.add_argument('--log-file', type=str, help='Path to candump log file (required for log mode)')
    parser.add_argument('--rate-multiplier', type=float, default=1.0, 
                       help='Replay speed multiplier for log mode (default: 1.0 = real-time)')
    parser.add_argument('--loop', action='store_true', help='Loop the log file continuously')
    
    args = parser.parse_args()
    
    # Validate log mode arguments
    if args.mode == 'log' and not args.log_file:
        parser.error("--log-file is required when using --mode log")
    
    print(f"Starting CAN simulator on {args.interface}")
    
    try:
        # Initialize CAN bus
        bus = can.interface.Bus(channel=args.interface, bustype='socketcan')
        print(f"CAN bus initialized on {args.interface}")
    except Exception as e:
        print(f"Failed to initialize CAN bus: {e}")
        print("\nMake sure vcan0 is set up:")
        print("  sudo modprobe vcan")
        print("  sudo ip link add dev vcan0 type vcan")
        print("  sudo ip link set up vcan0")
        return 1
    
    try:
        if args.mode == 'log':
            # Replay from log file
            replay_candump_log(bus, args.log_file, args.rate_multiplier, args.loop)
        else:
            # Generate simulated data
            print(f"Mode: {args.mode}, Rate: {args.rate} Hz")
            
            period = 1.0 / args.rate
            current = args.current
            speed = args.speed
            sweep_direction = 1
            
            print("\nSimulating CAN messages... Press Ctrl+C to stop\n")
            
            while True:
                if args.mode == 'static':
                    # Send static values
                    simulate_motor_current(bus, current)
                    simulate_vehicle_speed(bus, speed)
                    
                elif args.mode == 'random':
                    # Send random values
                    current = random.randint(-100, 300)  # -100 to 300 A
                    speed = random.uniform(0, 120)  # 0 to 120 km/h
                    simulate_motor_current(bus, current)
                    simulate_vehicle_speed(bus, speed)
                    
                elif args.mode == 'sweep':
                    # Sweep through values
                    simulate_motor_current(bus, current)
                    simulate_vehicle_speed(bus, speed)
                    
                    # Update for next iteration
                    current += sweep_direction * 10
                    speed += sweep_direction * 2.5
                    
                    # Reverse direction at limits
                    if current >= 300 or current <= -100:
                        sweep_direction *= -1
                    if speed >= 120 or speed <= 0:
                        sweep_direction *= -1
                
                print()  # Empty line for readability
                time.sleep(period)
            
    except KeyboardInterrupt:
        print("\n\nSimulation stopped by user")
    except Exception as e:
        print(f"\nError during simulation: {e}")
    finally:
        bus.shutdown()
        print("CAN bus closed")
    
    return 0


if __name__ == '__main__':
    exit(main())