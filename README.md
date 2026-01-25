# Costruzione del modello di un braccio 6DOF

## Costruzione del modello del braccio

prima del lancio di rviz bisogna adeguarsi agli americani :-(
    usare il comando 
    export LC_NUMERIC="en_US.UTF-8"
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