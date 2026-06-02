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


# with ROS
https://github.com/Pico-ROS

raspi with zenohpico
https://zenoh.io/blog/2025-01-08-introducing-raspberry-pi-pico-support-in-zenoh-pico/

zenohd -l serial//dev/ttyACM0#baudrate=112500

https://zenoh.io/blog/2022-08-12-zenoh-serial/

```

$ git clone -b api-changes https://github.com/eclipse-zenoh/zenoh
$ cd zenoh
$ cargo build --bin zenohd --example z_sub --no-default-features --features transport_tcp --features transport_serial

    Run zenohd on one terminal:

$ RUST_LOG=debug ./target/debug/zenohd --no-multicast-scouting -l "serial//dev/tty.usbserial-0001#baudrate=115200" -l “tcp/127.0.0.1:7447”
