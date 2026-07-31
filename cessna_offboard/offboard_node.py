import rclpy
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry


class OffboardNode(Node):

    def __init__(self):

        super().__init__("offboard_node")

        # quality of service profile for publishers
        qos_pub = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT, # ensures publishers are being sent
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL, # persists samples for 'late' subscriptions
            history=QoSHistoryPolicy.KEEP_LAST, # only store up to n samples, n = depth
            depth = 0 # queue size -> only if KEEP_LAST
        )

        # subscriber qos profile
        qos_sub = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            durability = QoSDurabilityPolicy.VOLATILE, # makes no attempt to persist
            history=QoSHistoryPolicy.KEEP_LAST,
            depth = 1
        )

        # subscribes to status to get real time updates of position, etc.
        self.status_sub = self.create_subscription(
            VehicleOdometry,
            'fmu/out/vehicle_odometry',
            self.status_callback,
            qos_sub
        )

        # creates the three publishers
        self.offboard_pub = self.create_publisher(OffboardControlMode, 'fmu/in/offboard_control_mode', qos_pub)
        self.trajectory_pub = self.create_publisher(TrajectorySetpoint, 'fmu/in/trajectory_setpoint', qos_pub)
        self.command_pub = self.create_publisher(VehicleCommand, 'fmu/in/vehicle_command', qos_pub)

        timer_period = 0.1

        #main timer
        self.main_timer = self.create_timer(timer_period, self.main_callback) 

        #gets initial status
        #self.vehicle_status = VehicleStatus()

        self.dt = timer_period 
        self.declare_parameter('radius', 10.0) # radius of orbiting circle
        self.declare_parameter('omega', 5.0) # angular velocity magnitude

        self.x = 0.0 # x position
        self.y = 0.0 # y position
        self.z_increment = 1.0
        self.current_z = 0.0 # z position
        self.new_z = 0.0
        self.target_z = 10.0 # target z position


        #parameters to update live
        self.theta = 0.0 # angle going around the orbiting cirlce
        self.radius = self.get_parameter('radius').value
        self.omega = self.get_parameter('omega').value

        self.offboard_count = 0

    #status callback every time a status gets published
    def status_callback(self, msg):
        #get new position
        self.x = msg.position[0]
        self.y = msg.position[1]
        self.current_z = msg.position[2]

        #calculate new z value
        if self.current_z >= self.target_z:
            self.new_z = self.current_z + self.z_increment
        else:
            self.new_z = self.current_z - self.z_increment

        #publish what the altitude is
        self.get_logger().info(f"altitude:{(self.current_z * -1):.2f}")


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

        if self.offboard_count == 10:
            trajectory_msg = TrajectorySetpoint()
            trajectory_msg.position[0] = self.radius * np.cos(self.theta)
            trajectory_msg.position[1] = self.radius * np.sin(self.theta)
            trajectory_msg.position[2] = -self.new_z
            self.trajectory_pub.publish(trajectory_msg)

            self.theta = self.theta + self.omega * self.dt
            self.get_logger().info('Publishing setpoints')

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
