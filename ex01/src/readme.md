
# Compile and Run

```bash
# On the workspace
cd ~/ros2_cpp_ws

# Build
colcon build --packages-select cpp_minimal

# Source
source install/setup.bash

# Run publisher
ros2 run cpp_minimal minimal_publisher

# On another terminal
ros2 run cpp_minimal minimal_subcriber

```