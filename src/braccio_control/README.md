# DOC
https://control.ros.org/master/doc/getting_started/getting_started.html

https://control.ros.org/master/doc/ros2_control_demos/example_7/doc/userdoc.html

# Architecture

![alt text](./doc/img/architecture_ros2control.png)

### ros2 control framework

![alt text](./doc/img/framework_ros2control.png)

## Control manager

connects the controllers and hardware-abstraction


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

- The <link name="world"/> and <link name="tool0"/> elements are not required. However, it is convention to set the link at the tip of the robot to tool0 and to define the robot’s base link relative to a world frame.

- The ros2_control tag specifies hardware configuration of the robot. More specifically, the available state and command interfaces. The tag has two required attributes: name and type. Additional elements, such as sensors, are also included in this tag.

- The hardware and plugin tags instruct the ros2_control framework to dynamically load a hardware driver conforming to HardwareInterface as a plugin. The plugin is specified as <{Name_Space}/{Class_Name}.

- Finally, the joint tag specifies the state and command interfaces that the loaded plugins will offer. The joint is specified with the name attribute. The command_interface and state_interface tags specify the interface type, usually position, velocity, acceleration, or effort.

test xacro:
```bash
xacro braccio_simulation/model/urdf/braccio.urdf.xacro > braccio.urdf
urdf_to_graphviz braccio.urdf braccio
```
