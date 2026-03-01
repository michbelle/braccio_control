import zenoh, random, time
import ctypes
import struct

import common

class motor_position(ctypes.Structure):
    # _pack_ = 1
    _fields_ = [
        ('rot1', ctypes.c_float),
        ('arm1_up', ctypes.c_float),
        ('arm2_up', ctypes.c_float),
        ('arm3_up', ctypes.c_float),
        ('rot_grasp', ctypes.c_float),
        ('grasp', ctypes.c_float),
        
    ]

pos=motor_position()

random.seed()

def read_temp():
    pos.rot1=1.1
    pos.arm1_up=2.3
    pos.arm2_up=2.4
    pos.arm3_up=2.5
    pos.rot_grasp=2.6
    pos.grasp=2.7
    
    return pos

def main(
    conf: zenoh.Config,
    key: str,
    payload: str,
    iter: Optional[int],
    interval: int,
    add_matching_listener: bool,
):
    # initiate logging
    zenoh.init_log_from_env_or("error")

    print("Opening session...")
    with zenoh.open(conf) as session:
        key = 'braccio_control/positions'
        pub = session.declare_publisher(key)
        while True:
            
            t = read_temp()
            t=struct.pack("ffffff",
                          pos.rot1, 
                          pos.arm1_up, 
                          pos.arm2_up, 
                          pos.arm3_up,
                          pos.rot_grasp,
                          pos.grasp
                          )
            
            payload = zenoh.ZBytes(t)
            buf = payload.to_bytes()
            print(buf)
            pub.put(buf)
            time.sleep(1)


if __name__ == "__main__":
    import argparse
    import itertools

    import common

    parser = argparse.ArgumentParser(prog="z_pub", description="zenoh pub example")
    common.add_config_arguments(parser)
    parser.add_argument(
        "--key",
        "-k",
        dest="key",
        default="demo/example/zenoh-python-pub",
        type=str,
        help="The key expression to publish onto.",
    )
    parser.add_argument(
        "--payload",
        "-p",
        dest="payload",
        default="Pub from Python!",
        type=str,
        help="The payload to publish.",
    )
    parser.add_argument(
        "--iter", dest="iter", type=int, help="How many puts to perform"
    )
    parser.add_argument(
        "--interval",
        dest="interval",
        type=float,
        default=1.0,
        help="Interval between each put",
    )
    parser.add_argument(
        "--add-matching-listener",
        default=False,
        action="store_true",
        help="Add matching listener",
    )

    args = parser.parse_args()
    conf = common.get_config_from_args(args)

    main(
        conf,
        args.key,
        args.payload,
        args.iter,
        args.interval,
        args.add_matching_listener,
    )