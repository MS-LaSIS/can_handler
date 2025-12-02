# Serial CAN Handler for Arduino

This package provides a ROS2 node that reads CAN bus data from an Arduino via serial and publishes it as ROS2 topics, mirroring the functionality of the `can_handler_node`.

## Arduino Setup

### Hardware
- Arduino with MCP2515 CAN controller
- MCP2515 connected via SPI (CS on pin 10)
- CAN bus connection

### Software
1. Upload the `can_read.ino` sketch to your Arduino
2. The sketch will:
   - Initialize the MCP2515 at 1000 kbps
   - Read CAN messages continuously
   - Send them over serial in CSV format: `ID,DLC,DATA0,DATA1,...`

### Serial Output Format
```
CAN_READY
26E,2,1A,2B
18FEF100,8,0,64,32,0,0,0,0,0
```

## ROS2 Setup

### Build the package
```bash
cd ~/ros2_ws
colcon build --packages-select can_handler
source install/setup.bash
```

### Install dependencies
```bash
pip3 install pyserial
```

## Usage

### Run with default parameters
```bash
ros2 launch can_handler serial_can_handler.launch.py
```

### Run with custom serial port
```bash
ros2 launch can_handler serial_can_handler.launch.py serial_port:=/dev/ttyACM0
```

### Run with all parameters
```bash
ros2 launch can_handler serial_can_handler.launch.py \
  serial_port:=/dev/ttyUSB0 \
  baud_rate:=115200 \
  motor_current_topic:=/motor_current \
  vehicle_speed_topic:=/vehicle_speed
```

### Run node directly
```bash
ros2 run can_handler serial_can_handler_node.py
```

## Published Topics

The node publishes to the following namespaced topics:

- `/serial_can_handler_node/motor_current` (std_msgs/Float32) - Motor current from CAN ID 0x26E
- `/serial_can_handler_node/vehicle_speed` (std_msgs/Float32) - Vehicle speed from CAN ID 0x18FEF100

## Parameters

- `serial_port` (string, default: `/dev/ttyUSB0`) - Serial port for Arduino
- `baud_rate` (int, default: `115200`) - Serial communication baud rate
- `motor_current_topic` (string, default: `/motor_current`) - Topic name for motor current
- `vehicle_speed_topic` (string, default: `/vehicle_speed`) - Topic name for vehicle speed

## Monitoring

### View all topics
```bash
ros2 topic list
```

### Echo motor current
```bash
ros2 topic echo /serial_can_handler_node/motor_current
```

### Echo vehicle speed
```bash
ros2 topic echo /serial_can_handler_node/vehicle_speed
```

## Troubleshooting

### Serial port permission denied
```bash
sudo usermod -a -G dialout $USER
# Log out and log back in for changes to take effect
```

Or run with sudo (not recommended):
```bash
sudo -E ros2 launch can_handler serial_can_handler.launch.py
```

### Arduino not detected
Check available serial ports:
```bash
ls /dev/tty*
# Look for /dev/ttyUSB0 or /dev/ttyACM0
```

### Debug mode
Enable debug logging:
```bash
DEBUG=true ros2 launch can_handler serial_can_handler.launch.py
```

## Differences from can_handler_node

| Feature | can_handler_node | serial_can_handler_node |
|---------|------------------|-------------------------|
| CAN Interface | SocketCAN (can0) | Serial (Arduino) |
| Dependencies | python-can | pyserial |
| Bitrate Config | 250 kbps | Configured in Arduino (1000 kbps) |
| Hardware | CAN interface | Arduino + MCP2515 |

Both nodes publish identical topics with the same message formats.
