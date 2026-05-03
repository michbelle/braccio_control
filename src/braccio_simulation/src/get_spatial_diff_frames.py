import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
import math

class JointDistanceNode(Node):
    def __init__(self):
        super().__init__('joint_distance_node')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.5, self.timer_cb)  # 2 Hz

        # Frames to compare — change to your joint/link frame names
        # self.frame_a = 'arm_1_up_down_p1'
        # self.frame_b = 'tool01'
        
        # self.frame_a = 'arm_1_up_down_p1'
        # self.frame_b = 'arm_2_up_down'
        
        # self.frame_a = 'arm_2_up_down'
        # self.frame_b = 'arm_3_up_down'
        
        self.frame_a = 'arm_3_up_down'
        self.frame_b = 'tool01'

    def timer_cb(self):
        try:
            t = self.tf_buffer.lookup_transform(self.frame_a, self.frame_b, rclpy.time.Time())
            dx = t.transform.translation.x
            dy = t.transform.translation.y
            dz = t.transform.translation.z
            print(dx, dy, dz)
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)
            self.get_logger().info(f"Distance {self.frame_a} -> {self.frame_b}: {dist:.6f} m")
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = JointDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()