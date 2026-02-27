import zenoh, random, time
import ctypes
import struct

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

if __name__ == "__main__":
    with zenoh.open(zenoh.Config()) as session:
        key = 'demo/example/test'
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
            # buf = f"{t}"
            # print(f"Putting Data ('{key}': '{buf}')...")
            
            # t=32.5
            # t=struct.pack("f",t)
            payload = zenoh.ZBytes(t)
            buf = payload.to_bytes()
            print(buf)
            pub.put(buf)
            time.sleep(1)