# ROS 2

* DDS-bases communication (Data Distribution Service)
* Better real-time support.
* Multiplatform (Linux, Windows, mascOS, RTOS).
* Integrated security.


## Architecture

```
Sistema ROS 2 = Nodos + Topics + Services + Actions + Parámetros
                ↓
           Middleware (DDS)
                ↓
           Sistema Operativo
```

# 2. Installation and configuration recommended ROS 2 Distributions

```bash
# Ubuntu 22.04 (recommended for production)
sudo apt update
sudo apt install ros-humbre-desktop-full

# For minimal development
sudo apt install ros-humbre-ros-base

# Set up environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## Check install

```bash
ros2 --version

# List installed pacjages
ros2 pkg list

# Run basic demo
ros2 run demo_nodes_cpp talker
# In other terminal
ros2 run demo_nodes_cpp listener
```

# 3. Workspaces and packages

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Create new package (Python)
ros2 pkg create --build-type ament_python my_fist_pkg
ros2 pkg create --build-type ament_cmake my_firts_cpp_pkg

# Python package struct:

my_fist_pkg/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/my_first_pkg
├── test/
├── my_first_pkg/
│   └── __init__.py
└── launch/

```


# Nodes 

A node is computing unit, a program that performs a specific work.

# Topics - Asynchronous Communication

A topic is a unidirectional channel of communication for data stream.

# Services - Synchronous Communication

Services provide request-response operations.

# Actions - Large operations

Actions are for long time tasks, with feedback and cancellation.

# 4. Command Line Tools

```bash
# List all running nodes
ros2 node list

# Get details for a node
ros2 node info /my_first_node

# List all topics
ros2 topic list

# See messages in a topic
ros2 topic echo /chatter

# Publish manually
ros2 topic pub /chatter std_msgs/msg/String "data: 'Hello from CLI'"

# See topic information
ros2 topic info /chatter --verbose

# List services
ros2 service list

# Call a service
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5, b: 3}"

```

Running commands

```bash


# Running a node
ros2 run my_first_pkg my_first_node

# Running with parameters
ros2 run my_first_pkg my_first_node --ros-args -r __node:=custom_name

# Run with launch file
ros2 launch my_package my_launch_file.launch.py

# See logs on real time
ros2 topic hz /chatter # Frequncy
ros2 topic bw /chatter # Band width
ros2 topic delay /chatter # Latency
```

# 5. Build System (colcon)

## Bulding packages

```bash

cd ~/ros2_ws
# Installing dependencies
rosdep install -i --from-path src --rosdistro humbre -y

# Compile packages
colcon build

# Compile specific package
colcon build --packages-select my_first_pkg

# Compile with debug symbols
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug

# WS Source
source install/setup.bash

```

## Colcon utils

```bash
# List workspace packages
colcon list

# Package test
colcon test

# Clean build
colcon build --cmake-clean-first

# Parallel compile
colcon build --parallel-workers 4
```
# 6. Custom messages creation

## Message struct

```bash
# msg/MyCustomMessage.msg
string first_name
string last_name
uint8 age
float32 height
float32[] sensor_readings
```

## Definition in package.xml

```xml
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

## Python code usage

```python
from my_package.msg import MyCustomMessage

msg = MyCustomMessage()
msg.first_name = "John"
msg.last_name = "Doe"
msg.age = 30
msg.height = 1.75
msg.sensor_readings = [1.0, 2.0, 3.0]
```

# 7. Launch files

## Basic launch file

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
     return LaunchDescription([
          Node(
               package='my_first_pkg',
               executable='my_first_node',
               name='custom_node_name',
               parameters=[
                    {'param1':'value1'},
                    {'param2':42}
               ]
          ),
          Node(
               package='demo_nodes_cpp',
               executable='talker',
               name='talker_node'
          )
     ])
```
## Launch with args

