from px4_msgs.msg import *
import time
from ls2n_drone_bridge.common import *
import transforms3d as tf3d


def rpy_from_quaternion(q):
    return tf3d.euler.mat2euler(tf3d.quaternions.quat2mat(q), axes='sxyz')


class Px4Comm:
    def __init__(self, node):
        self.node = node
        self.target_system = 1
        # Vehicle control
        self.command_publisher = self.node.create_publisher(
            VehicleCommand,
            'VehicleCommand_PubSubTopic',
            qos_profile_px4_com
        )
        self.offboard_mode_publisher = self.node.create_publisher(
            OffboardControlMode,
            'OffboardControlMode_PubSubTopic',
            qos_profile_px4_com
        )
        self.attitude_setpoint_publisher = self.node.create_publisher(
            VehicleAttitudeSetpoint,
            'VehicleAttitudeSetpoint_PubSubTopic',
            qos_profile_px4_com
        )
        self.rates_setpoint_publisher = self.node.create_publisher(
            VehicleRatesSetpoint,
            'VehicleRatesSetpoint_PubSubTopic',
            qos_profile_px4_com
        )
        self.local_position_setpoint_publisher = self.node.create_publisher(
            VehicleLocalPositionSetpoint,
            'VehicleLocalPositionSetpoint_PubSubTopic',
            qos_profile_px4_com
        )
        # Vehicle feedback
        self.node.create_subscription(
            VehicleControlMode,
            'VehicleControlMode_PubSubTopic',
            self.control_mode_listener_callback,
            qos_profile_px4_com
        )
        self.node.create_subscription(
            VehicleStatus,
            'VehicleStatus_PubSubTopic',
            self.status_listener_callback,
            qos_profile_px4_com
        )
        self.node.create_subscription(
            VehicleOdometry,
            'VehicleOdometry_PubSubTopic',
            self.odometry_callback,
            qos_profile_px4_com
        )
        self.node.create_subscription(
            BatteryStatus,
            'BatteryStatus_PubSubTopic',
            self.battery_callback,
            qos_profile_px4_com
        )
        # MOCAP input
        self.mocap_publisher = self.node.create_publisher(
            VehicleVisualOdometry,
            'VehicleVisualOdometry_PubSubTopic',
            qos_profile_px4_com
        )
        self.confirmed_connection = False
        self.offboard = False

    # Odometry in/out
    def odometry_callback(self, msg):
        odometry = FullState()
        # Transpose frames from PX4 representation (front, right, down) to our representation (front, left, up)
        odometry.position = np.array([msg.x, -msg.y, -msg.z])
        q = [float(msg.q[0]), float(msg.q[1]), float(-msg.q[2]), float(-msg.q[3])]
        odometry.orientation = np.array(q)
        _, _, odometry.yaw = rpy_from_quaternion(q)
        odometry.velocity = np.array([msg.vx, -msg.vy, -msg.vz])
        odometry.angular_velocity = np.array([msg.rollspeed, -msg.pitchspeed, -msg.yawspeed])
        # The timestamp samples for PX4 are in usec
        odometry.sec = msg.timestamp_sample // 1000000
        odometry.nanosec = (msg.timestamp_sample % 1000000) * 1000
        # Convert monotonic time (used by PX4 sync) to ros time
        ros_time = self.node.get_clock().now().nanoseconds
        offset = (ros_time - time.monotonic_ns())
        odometry.sec += offset // 1000000000
        odometry.nanosec += offset % 1000000000
        # Run the main loop of the bridge node
        self.node.main_loop(odometry)

    def send_mocap(self, msg_in):
        msg_out = VehicleVisualOdometry()
        msg_out.timestamp_sample = round(time.monotonic_ns() / 1.0e3)
        msg_out.timestamp = round(time.monotonic_ns() / 1.0e3)
        # Transform into the drone frame
        msg_out.x = msg_in.pose.pose.position.x
        msg_out.y = -msg_in.pose.pose.position.y
        msg_out.z = -msg_in.pose.pose.position.z
        msg_out.q = [msg_in.pose.pose.orientation.w,
                     msg_in.pose.pose.orientation.x,
                     -msg_in.pose.pose.orientation.y,
                     -msg_in.pose.pose.orientation.z]
        self.mocap_publisher.publish(msg_out)

    # Commands
    def default_command(self):
        msg = VehicleCommand()
        msg.timestamp = round(time.monotonic_ns() / 1.0e3)
        msg.target_system = self.target_system
        msg.target_component = 1
        msg.source_system = 255
        msg.from_external = True
        return msg

    def set_arm(self, arm):
        msg = self.default_command()
        msg.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        msg.param1 = 1.0 if arm else 0.0
        self.command_publisher.publish(msg)

    def set_offboard(self):
        msg = self.default_command()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0
        msg.param2 = 6.0
        self.command_publisher.publish(msg)

    def set_manual(self):
        msg = self.default_command()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        msg.param1 = 1.0
        msg.param2 = 1.0
        self.command_publisher.publish(msg)

    def set_offboard_mode(self, mode):
        msg = OffboardControlMode()
        msg.timestamp = round(time.monotonic_ns() / 1.0e3)
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        if mode == ControllerType.RATES_THRUST:
            msg.body_rate = True
        if mode == ControllerType.ATTITUDE_THRUST:
            msg.attitude = True
        self.offboard_mode_publisher.publish(msg)

    def set_lockdown(self):
        msg = self.default_command()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_FLIGHTTERMINATION
        msg.param1 = 2.0
        self.command_publisher.publish(msg)

    def release_lockdown(self):
        msg = self.default_command()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_FLIGHTTERMINATION
        msg.param1 = 0.0
        self.command_publisher.publish(msg)

    # Vehicle feedback
    def control_mode_listener_callback(self, received_msg):
        self.node.update_armed(received_msg.flag_armed)
        self.offboard = received_msg.flag_control_offboard_enabled

    def status_listener_callback(self, received_msg):
        if not self.confirmed_connection:
            self.confirmed_connection = True
            self.node.get_logger().info("Connection to PX4 confirmed")

    def battery_callback(self, msg):
        self.node.update_battery_voltage(msg.voltage_filtered_v)

    # Setpoints
    def set_attitude(self, desired_state, desired_scaled_thrust):
        setpoint = VehicleAttitudeSetpoint()
        setpoint.timestamp = round(time.monotonic_ns() / 1.0e3)
        setpoint.q_d = desired_state.orientation.astype("float32")
        # Converting to PX4 frame
        setpoint.thrust_body = [0.0,
                                0.0,
                                -min(max(desired_scaled_thrust, 0.05), 1.0)]
        setpoint.q_d[2] = -setpoint.q_d[2]
        setpoint.q_d[3] = -setpoint.q_d[3]
        # This needs to be done in rtps as it was done by the mavlink receiver
        setpoint.roll_body, setpoint.pitch_body, setpoint.yaw_body = rpy_from_quaternion(setpoint.q_d)
        self.set_offboard_mode(ControllerType.ATTITUDE_THRUST)
        if self.offboard:
            self.attitude_setpoint_publisher.publish(setpoint)

    def set_rates(self, desired_state: FullState, desired_scaled_thrust: float):
        setpoint = VehicleRatesSetpoint()
        setpoint.timestamp = round(time.monotonic_ns() / 1.0e3)
        setpoint.roll = desired_state.angular_velocity[0]
        setpoint.pitch = desired_state.angular_velocity[1]
        setpoint.yaw = desired_state.angular_velocity[2]
        # Converting to PX4 frame
        setpoint.thrust_body = [0.0,
                                0.0,
                                -min(max(desired_scaled_thrust, 0.05), 1.0)]
        setpoint.pitch = -setpoint.pitch
        setpoint.yaw = -setpoint.yaw
        self.set_offboard_mode(ControllerType.RATES_THRUST)
        if self.offboard:
            self.rates_setpoint_publisher.publish(setpoint)
