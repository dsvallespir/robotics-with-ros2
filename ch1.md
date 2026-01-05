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

A node is computing unit, a program that performs a specific work. Nodes can communicate with other nodes within the same process, in a different process, or on a different machine. Nodes are typically the unit of computation in a ROS graph; each node shoud do one logical thing.

# Messages
Messages are a way for a ROS 2 node to send data on the network to other ROS nodes, with no response expected.
Messages are described and defined in .msg files in the msg/ directory of a ROS package. 

### Example

```bash
int32[] unbounded_integer_array
int32[5] five_integers_array
int32[<=5] up_to_five_integers_array

string string_of_unbounded_size
string<=10 up_to_ten_characters_string

string[<=5] up_to_five_unbounded_strings
string<=10[] unbounded_array_of_strings_up_to_ten_characters_each
string<=10[<=5] up_to_five_strings_up_to_ten_characters_each
```



# Topics - Asynchronous Communication

A topic should be used for continuous data streams, like sensor data, robot state, etc.

ROS 2 is a strongly-typed, anonymous publish/subscribe system.

## Publish/Subscribe

A publish/subscribe system is one in which there are producers of data (publishers) and consumers of data (subscribers). They contact each other throught a topic. There may be zero or more publishers and zero or more subscribers on any particular topic. When data is published to the topic by any of the publishers, all subscribers in the system will receive the data.

## Anonymous
When a subscriber gets a piece of data, it doesn't generally know or care which publisher originally sent it.

## Strongly-typed
The types of each field in a ROS message are typed, and that type is enforced at various levels.
The semantic of each field are well-defined. There is no automated mechanism to ensure this, but all of the core ROS types have strong semantics associated with them. For instance, the IMU message contains a 3-dimensional vector for the measured angular velocity, and each of the dimensions is specified to be in radians/second. 

# Services - Synchronous Communication

Services provide request/response communication, where the cliente (requester) is waiting for the server (responder) to make a short computation and return a result.

Services are described and defined in `.srv` file in the `srv/` directory of a ROS package. You cannot embed another service inside of a service.

### Example
Service that takes in a string and return a string:
```bash
string str
---
string str
```

```bash
# request constants
int8 FOO=1
int8 BAR=2
# request fields
int8 foobar
another_pkg/AnotherMessage msg
---
# response constants
uint32 SECRET=123456
# response fields
another_pkg/YetAnotherMessage val
CustomMessageDefinedInThisPackage value
uint32 an_integer
```

# Actions - Large operations

Actions are for long-running request/response communication, where the action client (requester) is waiting for the action server (responser) to take some action and return a result. They provide feedback and can be interrupted (cancellation).

Action definitions have the following form:
```bash
<request_type> <request_fieldname>
---
<response_type> <response_fieldname>
---
<feedback_type> <feedback_fieldname>
```
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

```python
from launch import LaunchDescription
from launch.actions import DecalreLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
     return LaunchDescription([
          DeclareLaunchArgument(
               'node_name', 
               default_value='default_node',
               description='Nombre del nodo'
          ),
          Node(
               package='my_package',
               executable='my_node',
               name=LaunchConfiguration('node_name')
          )
     ])
```

# 8. Parameters system

## Definition and usage of parameters



# 10. C++ Package Struct

```bash
# Create Workspace
mkdir -p ~/ros2_cpp_ws/src
cd ~/ros2_cpp_ws/src

# Create basic C++ package
ros2 pkg create cpp_basics \ 
     --build-type ament_cmake \
     --dependencies rclcpp std_msgs

# Estructura generada:
cpp_basics/
├── CMakeLists.txt
├── include/cpp_basics/
├── package.xml
└── src/
```

### CMakeLists.txt minimal

```bash
cmake_minimum_required(VERSION 3.8)
project(cpp_basics)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Ejecutable 1
add_executable(minimal_publisher src/minimal_publisher.cpp)
ament_target_dependencies(minimal_publisher rclcpp std_msgs)

# Instalación
install(TARGETS
  minimal_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
```

# 11. Basic examples

See ex/src/minimal_publisher.cpp
