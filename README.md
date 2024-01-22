# ROS GUI Library (ROS 2 Branch)

This library is intended to be used with ros2 to create simple UI applications.

It uses OpenGL and SDL2 to handle drawing the window, and uses Dear ImGui to draw UI elements.

## Installation

```bash
cd ~/ros_ws/src
git clone https://github.com/hamyyy/ros_gui.git
cd ..
colcon build
```

To use this library, make sure to include it in your `CMakeLists.txt` and `Package.xml` files. Use [the provided example](./src/example.cpp) as a starting point.

## Usage

for more information on how to use ImGui, please read the ImGui [documentation](https://github.com/ocornut/imgui/tree/docking) (docking branch) or have a look at the [ImGui Demo file](./lib/imgui/imgui_demo.cpp).