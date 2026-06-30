import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleControlMode
from rclpy.qos import *

class PX4Listener(Node):

    def __init__(self):
        
        super().__init__("px4_listener")

        qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            history = HistoryPolicy.KEEP_LAST,
            depth = 10
        )

        self.listener = self.create_subscription(
            VehicleControlMode,
            "/fmu/out/vehicle_control_mode",
            self.status_callback,
            qos
        )

        self.get_logger().info("listener active")

    def status_callback(self, msg):
        self.get_logger().info(f"armed? {msg.flag_armed}")
        self.get_logger().info(f"offboard? {msg.flag_control_offboard_enabled}")
        

def main(args=None):

    rclpy.init(args=args)

    node = PX4Listener()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()