import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleStatus, VehicleLocalPosition


class OffboardNode(Node):

    def __init__(self):

        super().__init__("offboard_node")

        # quality of service profile for publishers
        qos_pub = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT, # ensures publishers are being sent
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL, # persists samples for 'late' subscriptions
            history=QoSHistoryPolicy.KEEP_LAST, # only store up to n samples, n = depth
            depth = 10 # queue size -> only if KEEP_LAST
        )

        # subscriber qos profile
        qos_sub = QoSProfile(
        reliability = QoSReliabilityPolicy.BEST_EFFORT,
            durability = QoSDurabilityPolicy.VOLATILE, # makes no attempt to persist
            history=QoSHistoryPolicy.KEEP_LAST,
            depth = 10
        )

        # subscribes to status to get real time updates of position, etc.
        self.status_sub = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status_v1',
            self.status_callback,
            qos_sub
        )

        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            "fmu/out/vehicle_local_position_v1",
            self.position_callback,
            qos_sub
        )

        self.max_heading = 0.0
        self.min_heading = 0.0

        # creates the three publishers
        self.offboard_pub = self.create_publisher(OffboardControlMode, 'fmu/in/offboard_control_mode', qos_pub)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, 'fmu/in/trajectory_setpoint', qos_pub)
        self.command_pub = self.create_publisher(VehicleCommand, 'fmu/in/vehicle_command', qos_pub)

        timer_period = 0.1

        #main timer
        self.main_timer = self.create_timer(timer_period, self.main_callback) 

        self.dt = timer_period # delta theta
        self.declare_parameter('altitude', 50.0) # altitude of orbit (usually 50m)
        self.declare_parameter('radius', 100.0) # radius of orbiting circle
        self.declare_parameter('omega', 0.1875) # angular velocity magnitude

        #parameters to update live
        self.theta = 0.0 # angle in radians going around the orbiting circle
        self.radius = self.get_parameter('radius').value
        self.omega = self.get_parameter('omega').value
        self.altitude = self.get_parameter('altitude').value

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED

        self.offboard_count = 0

    #status callback every time a status gets published
    def status_callback(self, msg):
        print("nav status: ", msg.nav_state)
        print(" -offboard status: " , VehicleStatus.NAVIGATION_STATE_OFFBOARD)
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def position_callback(self, msg):
        print("true heading: ", msg.heading)
        print(f"x: {msg.x} | y: {msg.y} | z: {-msg.z}")
        if msg.heading < self.min_heading:
            self.min_heading = msg.heading
        elif msg.heading > self.max_heading:
            self.max_heading = msg.heading

    #main callback function
    def main_callback(self):
        if self.offboard_count == 10:
            self.offboard()
            self.arm()

        offboard_msg = OffboardControlMode()

        offboard_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_msg.position = True
        offboard_msg.velocity = False
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False

        self.offboard_pub.publish(offboard_msg)

        if self.offboard_count <= 11:
            self.offboard_count += 1

        if self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED:
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position[0] = self.radius * np.cos(self.theta)
            trajectory_msg.position[1] = self.radius * np.sin(self.theta)
            trajectory_msg.position[2] = -self.altitude
            trajectory_msg.yaw = self.theta + np.pi/2
            print('my heading: ', self.theta + np.pi/2)
            self.trajectory_pub.publish(trajectory_msg)
            self.theta = self.theta + self.omega * self.dt

    def arm(self):
        self.command_publisher(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0) # calls the command_publisher saying "arm"
        self.get_logger().info('Arming vehicle...')

    def offboard(self):
        self.command_publisher(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) # calls the command_publisher saying "enter offboard"
        self.get_logger().info('Switching to offboard')

    def command_publisher(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.param1 = param1
        msg.param2 = param2
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.command_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = OffboardNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
