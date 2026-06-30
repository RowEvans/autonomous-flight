import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class OffboardNode(Node):

    def __init__(self):
        super().__init__("offboard_node")

        self.counter = 0

        self.publisher = self.create_publisher(
            String,
            "test_topic", 
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.timer_callback
        )
    
    def timer_callback(self):
        self.counter += 1

        self.get_logger().info(f"publishing tick: {self.counter}")

        msg = String()
        msg.data = f"tick {self.counter}"

        self.publisher.publish(msg)

def main(args=None):
    
    rclpy.init(args=args)

    node = OffboardNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()