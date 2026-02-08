code for the pca9685
https://github.com/kimsniper/pca9685

i2c
https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/i2c.html

it can go over 2.1 -> 2.4

duticycle:
- grasp: 
    - open : 0.6ms
    - close : 1.2ms
- rot_grasp
    - plan-servo-up : 0.6ms
    - plan-servo-down : 2.1ms
- arm_3_up_down
    - plan-servo-up : 0.6ms
    - plan-servo-down : 2.2ms
- arm_2_up_down
    - plan-servo-up : 0.55ms
    - plan-servo-down : 2.0ms
- arm_1_up_down
    - plan-servo-up : 0.55ms
    - plan-servo-down : 1.4ms
- arm_rot
    - r : 0.6ms
    - l: : 2.2ms
