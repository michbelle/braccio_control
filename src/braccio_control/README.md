# DOC
https://control.ros.org/master/doc/getting_started/getting_started.html

https://control.ros.org/master/doc/ros2_control_demos/example_7/doc/userdoc.html

# Architecture

![alt text](./doc/img/architecture_ros2control.png)

### ros2 control framework

![alt text](./doc/img/framework_ros2control.png)

## Control manager

connects the controllers and hardware-abstraction.
CM manages (e.g. loads, activates, deactivates, unloads) controllers and the interfaces they require. On the other hand, it has access (via the Resource Manager) to the hardware components, i.e. their interfaces. The Controller Manager matches required and provided interfaces, granting controllers access to hardware when enabled, or reporting an error if there is an access conflict.

The execution of the control-loop is managed by the CM’s update() method. It reads data from the hardware components, updates outputs of all active controllers, and writes the result to the components

## Resource Manager
abstracts physical hardware and its drivers (called hardware components) for the ros2_control framework. The RM loads the components using the pluginlib-library, manages their lifecycle and components’ state and command interfaces. The abstraction provided by RM allows reuse of implemented hardware components

In the control loop execution, the RM’s read() and write() methods handle the communication with the hardware components.

## Controllers
are based on control theory. They compare the reference value with the measured output and, based on this error, calculate a system’s input. The controllers are objects derived from ControllerInterface (controller_interface package in ros2_control) and exported as plugins using pluginlib-library.

When the control-loop is executed, the update() method is called. This method can access the latest hardware state and enable the controller to write to the hardware command interfaces.

## User Interfaces
Users interact with the ros2_control framework using Controller Manager’s services. For a list of services and their definitions, check the srv folder in the controller_manager_msgs package.

## Hardware Components
ealize communication to physical hardware and represent its abstraction in the ros2_control framework. The components have to be exported as plugins using pluginlib-library. The Resource Manager dynamically loads those plugins and manages their lifecycle.
**System**

    Complex (multi-DOF) robotic hardware like industrial robots. The main difference between the Actuator component is the possibility to use complex transmissions like needed for humanoid robot’s hands. This component has reading and writing capabilities. It is used when there is only one logical communication channel to the hardware (e.g., KUKA-RSI).
**Sensor**

    Robotic hardware is used for sensing its environment. A sensor component is related to a joint (e.g., encoder) or a link (e.g., force-torque sensor). This component type has only reading capabilities.
**Actuator**

    Simple (1 DOF) robotic hardware like motors, valves, and similar. An actuator implementation is related to only one joint. This component type has reading and writing capabilities. Reading is not mandatory if not possible (e.g., DC motor control with Arduino board). The actuator type can also be used with a multi-DOF robot if its hardware enables modular design, e.g., CAN-communication with each motor independently.



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
