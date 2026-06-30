import rclpy
from rclpy.node import Node
from rclpy.qos import *
from px4_msgs.msg import *

class OffboardNode(Node):

    def __init__(self):

        super().__init__("offboard_node")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher= self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            qos
        )

        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            qos
        )

        self.arm_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            qos
        )

        self.arm_timer = self.create_timer(1.0, self.arm)

        self.offboard_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            qos
        )

        self.offboard_timer = self.create_timer(1.0, self.offboard)

        self.altitude_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position",
            self.altitude,
            qos
        )

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        
        msg = OffboardControlMode()

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        msg.timestamp = (
            self.get_clock()
            .now()
            .nanoseconds
            // 1000
        )

        self.publisher.publish(msg)

        setpoint = TrajectorySetpoint()

        setpoint.position = [0.0, 0.0, -10.0]

        setpoint.yaw = 0.0

        setpoint.timestamp = (
            self.get_clock()
            .now()
            .nanoseconds
            // 1000
        )

        self.setpoint_pub.publish(setpoint)

    def altitude(self, msg):
        self.get_logger().info(f"Altitude: {(msg.x * -1):.2f}")

    def arm(self):
        msg = VehicleCommand()

        msg.command = 400
        msg.param1 = 1.0
        msg.param2 = 0.0
        msg.param3 = 0.0
        msg.param4 = 0.0

        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.arm_pub.publish(msg)

    def offboard(self):
        msg = VehicleCommand()
        
        msg.command = 176
        msg.param1 = 1.0
        msg.param2 = 6.0
        msg.param3 = 0.0
        msg.param4 = 0.0

        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True

        self.offboard_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = OffboardNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
