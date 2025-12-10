# CAN Handler

A ROS2 package for reading and writing CAN bus messages. This node acts as a bridge between the CAN bus and ROS2 topics.

## Features

- **Read CAN messages**: Receives CAN frames and publishes parsed data to ROS2 topics
  - Motor Current (CAN ID `0x26E`) → `std_msgs/Float32`
  - Vehicle Speed (CAN ID `0x18FEF100`) → `std_msgs/Float32`
- **Write CAN messages**: Subscribes to a ROS2 topic and sends CAN frames
  - Uses standard `can_msgs/Frame` message format

## Dependencies

```bash
# ROS2 packages
sudo apt install ros-${ROS_DISTRO}-can-msgs

# Python packages
pip install canopen
```

## Installation

```bash
cd ~/ros2_ws/src
git clone <repository_url> can_handler
cd ~/ros2_ws
colcon build --packages-select can_handler
source install/setup.bash
```

## Usage

### Running with Physical CAN Interface

```bash
# Make sure CAN interface is up
sudo ip link set can0 up type can bitrate 250000

# Launch the node
ros2 launch can_handler can_handler.launch.py can_interface:=can0
```

### Running with Virtual CAN (for testing)

#### 1. Set up Virtual CAN Interface

```bash
# Load the vcan kernel module
sudo modprobe vcan

# Create virtual CAN interface
sudo ip link add dev vcan0 type vcan

# Bring the interface up
sudo ip link set up vcan0

# Verify it's running
ip link show vcan0
```

#### 2. Launch the CAN Handler Node

```bash
ros2 launch can_handler can_handler_vinterface.launch.py
```

#### 3. Simulate CAN Messages (in another terminal)

The package includes a simulation script to generate test CAN messages:

```bash
# Random mode (default) - generates random current and speed values
ros2 run can_handler simulate_can.py --interface vcan0 --mode random

# Static mode - sends fixed values
ros2 run can_handler simulate_can.py --interface vcan0 --mode static --current 100 --speed 50.5

# Sweep mode - gradually changes values
ros2 run can_handler simulate_can.py --interface vcan0 --mode sweep --rate 5

# Replay from candump log file
ros2 run can_handler simulate_can.py --interface vcan0 --mode log --log-file /path/to/candump.log

# Replay at 2x speed with looping
ros2 run can_handler simulate_can.py --mode log --log-file candump.log --rate-multiplier 2.0 --loop
```

### Verify Topics

```bash
# List available topics
ros2 topic list

# Echo motor current
ros2 topic echo /can_handler_node/motor_current

# Echo vehicle speed
ros2 topic echo /can_handler_node/vehicle_speed
```

### Sending CAN Messages

To send CAN messages from ROS2 to the CAN bus, publish to the `/to_can` topic using the `can_msgs/Frame` message:

```bash
# Send a CAN frame with ID 0x123 and 8 bytes of data
ros2 topic pub /to_can can_msgs/msg/Frame "{id: 0x123, dlc: 8, data: [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]}"

# Send an extended frame (29-bit ID)
ros2 topic pub /to_can can_msgs/msg/Frame "{id: 0x18FF0001, is_extended: true, dlc: 4, data: [0xAA, 0xBB, 0xCC, 0xDD, 0, 0, 0, 0]}"
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `can_interface` | `can0` | CAN interface name (e.g., `can0`, `vcan0`) |
| `motor_current_topic` | `/motor_current` | Topic suffix for motor current messages |
| `vehicle_speed_topic` | `/vehicle_speed` | Topic suffix for vehicle speed messages |
| `to_can_topic` | `/to_can` | Topic for outgoing CAN messages |

## Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `<node_name>/motor_current` | `std_msgs/Float32` | Motor current in Amps |
| `<node_name>/vehicle_speed` | `std_msgs/Float32` | Vehicle speed in km/h |

## Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/to_can` | `can_msgs/Frame` | CAN frames to send to the bus |

## CAN Message Format

### Motor Current (ID: 0x26E)
- **Bytes 0-1**: Signed 16-bit integer (little-endian, two's complement)
- **Unit**: Amperes

### Vehicle Speed (ID: 0x18FEF100)
- **Byte 2**: Fractional part (value / 256.0)
- **Byte 3**: Integer part
- **Unit**: km/h
- **Formula**: `speed = byte[3] + byte[2] / 256.0`

## Simulation Script Options

```
usage: simulate_can.py [-h] [--interface INTERFACE] [--rate RATE]
                       [--mode {static,random,sweep,log}] [--current CURRENT]
                       [--speed SPEED] [--log-file LOG_FILE]
                       [--rate-multiplier RATE_MULTIPLIER] [--loop]

Options:
  --interface      CAN interface (default: vcan0)
  --rate           Message rate in Hz for simulation (default: 10)
  --mode           Simulation mode: static, random, sweep, or log
  --current        Static current value in A (default: 50)
  --speed          Static speed value in km/h (default: 30.0)
  --log-file       Path to candump log file (required for log mode)
  --rate-multiplier Replay speed multiplier (default: 1.0 = real-time)
  --loop           Loop the log file continuously
```

## Troubleshooting

### Virtual CAN not working
```bash
# Check if vcan module is loaded
lsmod | grep vcan

# If not loaded
sudo modprobe vcan
```

### Permission denied on CAN interface
```bash
# Add user to can group (may require logout/login)
sudo usermod -aG can $USER
```

### Monitor CAN traffic
```bash
# Install can-utils
sudo apt install can-utils

# Dump all CAN messages
candump vcan0

# Send a test message
cansend vcan0 123#DEADBEEF
```

## License

Apache-2.0