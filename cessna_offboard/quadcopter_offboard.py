import rclpy
import numpy as np
import time
import threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from px4_msgs.msg import VehicleStatus, VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition

class OffboardNode(Node):
    def __init__(self):
        super().__init__("quadcopter_offboard")

        qos_out = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT, # ensures being sent
            durability = QoSDurabilityPolicy.TRANSIENT_LOCAL, # persists policies for 'late' subscriptions
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 10
        )

        qos_in = QoSProfile(
            reliability = QoSReliabilityPolicy.BEST_EFFORT,
            durability = QoSDurabilityPolicy.VOLATILE,
            history = QoSHistoryPolicy.KEEP_LAST,
            depth = 10
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            'fmu/out/vehicle_status_v1',
            self.status_callback,
            qos_in
        )

        self.pos_sub = self.create_subscription(
            VehicleLocalPosition,
            'fmu/out/vehicle_local_position_v1',
            self.pos_callback,
            qos_in
        )

        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX # not offboard
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED # not armed for external cmd flight

        self.cmd_pub = self.create_publisher(VehicleCommand, 'fmu/in/vehicle_command', qos_out) # command publisher
        self.pos_pub = self.create_publisher(TrajectorySetpoint, 'fmu/in/trajectory_setpoint', qos_out) # setpoint publisher
        self.ob_pub = self.create_publisher(OffboardControlMode, 'fmu/in/offboard_control_mode', qos_out) # offboard publisher

        timer_period = 0.02 # seconds
        self.timer = self.create_timer(timer_period, self.main_callback)

        self.dt = timer_period # delta theta

        self.declare_parameter('radius', 15.0) # radius of 15.0m
        self.declare_parameter('altitude', 50.0) # altitude of 50.0m
        self.declare_parameter('omega', 0.5) # angular velocity of leading tangential point

        self.declare_parameter('mode', 0)

        self.theta = 0 # angle in a circle
        self.radius = self.get_parameter('radius').value
        self.altitude = self.get_parameter('altitude').value
        self.omega = self.get_parameter('omega').value

        self.mode = self.get_parameter('mode').value
        self.modes = {0: 'PRE_FLIGHT',
                      1: 'ARMING',
                      2: 'CLIMBING',
                      3: 'LOITERING'}

        self.z = 0.0

        self.ob_count = 0


    def status_callback(self, msg):
        print('nav: ', msg.nav_state)
        print('offboard: ', msg.arming_state)
        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    def pos_callback(self, msg):
        self.z = -msg.z
        self.get_logger().info(f"altitude: {self.z}")



    def main_callback(self):

        #SEQUENCE:
        # MODES: PRE_FLIGHT, ARMING, CLIMBING, LOITERING
        # PRE_FLIGHT: Posting messages, but waiting
        # ARMING: Arms and enters offboard mode
        # CLIMBING: Sending setpoints on its way up
        # LOITERING: Back to orbiting logic
        # need to always be sending offboard_msgs
        self.get_logger().info("MODE: " + self.modes[self.mode])

        self.ob_msgs() # sending constant ob_msgs

        if self.mode == 0:
            self.ob_count += 1

        if self.ob_count == 10:
            self.ob_count = 11
            self.mode = 1

        elif self.mode == 1:
            self.arm()
            self.enter_offboard()
            if (self.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.arming_state == VehicleStatus.ARMING_STATE_ARMED):
                self.mode = 2

        elif self.mode == 2:
            self.climb()
            if (self.altitude - 3) < self.z: # 45 is less than 50 then self.mode == 3 and loiter
                self.mode = 3


        elif self.mode == 3:
            self.loiter()

    def ob_msgs(self):
        ob_msg = OffboardControlMode()

        ob_msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        ob_msg.position = True
        ob_msg.velocity = False
        ob_msg.acceleration = False
        ob_msg.attitude = False
        ob_msg.body_rate = False

        self.ob_pub.publish(ob_msg)

    def climb(self):
        pos_msg = TrajectorySetpoint()

        pos_msg.position[0] = 0.0
        pos_msg.position[1] = 0.0
        pos_msg.position[2] = -self.altitude

        self.pos_pub.publish(pos_msg)

    def loiter(self):
        pos_msg = TrajectorySetpoint()

        pos_msg.position[0] = self.radius * np.cos(self.theta)
        pos_msg.position[1] = self.radius * np.sin(self.theta)
        pos_msg.position[2] = -self.altitude
        self.pos_pub.publish(pos_msg)

        self.theta = self.theta + self.omega * self.dt

    def arm(self):
        self.cmd_publisher(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("Arming vehicle...")

    def enter_offboard(self):
        self.cmd_publisher(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info("Entering offboard...")


    def cmd_publisher(self, cmd, param1=0.0, param2=0.0):
        msg = VehicleCommand()

        msg.command = cmd
        msg.param1 = param1
        msg.param2 = param2
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OffboardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()