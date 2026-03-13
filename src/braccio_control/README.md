# DOC
https://control.ros.org/master/doc/ros2_control_demos/example_7/doc/userdoc.html


# DOC

## ros2_control overview
**state_interfaces** and **command_interfaces** to abstract hardware interfacing. 
- The **state_interfaces** are read only data handles that generally represent sensors readings.
- The **command_interfaces** are read and write data handles that hardware commands (exclusively accessed , 1 can use it)

provides the **ControllerInterface** and **HardwareInterface**.

controllers request **state_interfaces** and **command_interfaces** required for operation through the ControllerInterface

hardware drivers offer **state_interfaces** and **command_interfaces** via the HardwareInterface

ros2_control ensure all requested interfaces are available before starting the controllers. The interface pattern allows vendors to write hardware specific drivers that are loaded at runtime.

- read call, hardware drivers that conform to HardwareInterface update their offered state_interfaces with the newest values received from the hardware
- update call, controllers calculate commands from the updated state_interfaces and writes them into its command_interfaces
- write call, the hardware drivers read values from the command_interfaces and sends them to the hardware

ros2_control_node runs 
- the main loop in a realtime thread.
- second non-realtime thread to interact with ROS publishers, subscribers, and services.

## Writing a URDF

