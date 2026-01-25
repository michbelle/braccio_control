# Costruzione del modello di un braccio 6DOF

## doc

https://robofoundry.medium.com/esp32-micro-ros-actually-working-over-wifi-and-udp-transport-519a8ad52f65
https://micro.ros.org/docs/tutorials/core/first_application_linux/
https://zenoh.io/blog/2021-10-04-zenoh-pico-guide/
https://docs.px4.io/main/en/middleware/zenoh
https://zenoh.io/blog/2021-11-09-ros2-zenoh-pico/
https://control.ros.org/kilted/doc/getting_started/getting_started.html

https://control.ros.org/kilted/doc/ros2_control_demos/doc/index.html
https://github.com/ros-controls/ros2_control_demos/tree/master/example_7

### hardware components

https://control.ros.org/kilted/doc/getting_started/getting_started.html#hardware-components
https://youtu.be/ZRrC6Hss01Y


## Costruzione del modello del braccio


##Creazione del file per la realizzazionde della navigazione del braccio


#arduino
rosrun servo_control serial_node.py /dev/ttyACM0

#Commander
tool per comandare in terminal il braccio

rosrun moveit_commander moveit_commander_cmdline.py


### devcontainer

start ssh agent
```bash
eval "$(ssh-agent -s)"
ssh-add ~/.ssh/id_ed25519
```

then run
```bash
DOCKER_BUILDKIT=1 docker build -t braccio_control_img . --ssh default
```

then launch the image with all build in with

share x with:
```bash
xhost local:root
```