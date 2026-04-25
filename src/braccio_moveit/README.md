https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html

ros2 launch braccio_moveit demo.launch.py


per funzionare planner bisogna modificare `joint_limits.yaml` che
```bash

    has_acceleration_limits: true
    max_acceleration: 1.0
```