import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from nav_msgs.msg import Odometry
from ls2n_drone_bridge.controllers import *
import importlib
import os


class Status:
    status = DroneStatus.IDLE
    lastAlive = None
    battery_voltage = 12.0


class DroneBridge(Node):
    def __init__(self):
        super().__init__('drone_bridge')
        # Parameters with dynamic update
        parameters = [
            ['mass', 1.0, ParameterType.PARAMETER_DOUBLE, 'Drone mass'],
            ['max_thrust', 47.3, ParameterType.PARAMETER_DOUBLE, 'Drone maximal thrust'],
            ['max_thrust_a', 7.8731, ParameterType.PARAMETER_DOUBLE, 'Ramp for drone maximal thrust'],
            ['max_thrust_b', -43.072, ParameterType.PARAMETER_DOUBLE, 'Origin for drone maximal thrust'],
            ['use_battery_for_thrust', False, ParameterType.PARAMETER_BOOL,
             'Use battery values to adapt max thrust value (true/false)'],
            ['kp_position', 2.0, ParameterType.PARAMETER_DOUBLE, 'Drone position controller P gain'],
            ['kd_position', 2.0, ParameterType.PARAMETER_DOUBLE, 'Drone position controller D gain'],
            ['ki_position', 0.05, ParameterType.PARAMETER_DOUBLE, 'Drone position controller I gain'],
            ['kp_velocity', 2.0, ParameterType.PARAMETER_DOUBLE, 'Drone velocity controller P gain'],
            ['ki_velocity', 0.05, ParameterType.PARAMETER_DOUBLE, 'Drone velocity controller I gain'],
            ['firmware', 'PX4', ParameterType.PARAMETER_STRING, 'Drone firmware'],
            ['take_off_height', 1.0, ParameterType.PARAMETER_DOUBLE, 'Height for automatic take off']
        ]
        for parameter in parameters:
            self.declare_parameter(parameter[0], parameter[1], ParameterDescriptor(type=parameter[2],
                                                                                   description=parameter[3]))
            setattr(self, parameter[0], lambda param=parameter[0]: self.get_parameter(param).value)

        self.get_logger().info("Starting drone bridge node")
        # Firmware communication
        if self.firmware() == "PX4":
            com_module = importlib.import_module("ls2n_drone_bridge.px4_com")
            com = getattr(com_module, 'Px4Comm')
            self.com = com(self)
        else:
            self.get_logger().error("Invalid firmware")
            exit(os.EX_USAGE)
        # Keep alive
        self.keep_alive_timer = self.create_timer(0.05, self.keep_alive_check)
        self.create_subscription(
            KeepAlive,
            'KeepAlive',
            self.keep_alive_callback,
            qos_profile_sensor_data
        )
        self.keep_alive_publisher = self.create_publisher(
            KeepAlive,
            'KeepAlive',
            qos_profile_sensor_data
        )
        # Services
        self.create_service(
            DroneRequest,
            'Request',
            self.handle_requests
        )
        # Bridge feedback
        self.status_publisher = self.create_publisher(
            DroneStatus,
            'Status',
            qos_profile_sensor_data
        )
        self.odom_publisher = self.create_publisher(
            Odometry,
            'EKF/odom',
            qos_profile_sensor_data
        )
        self.status_timer = self.create_timer(0.01, self.publish_status)
        # MOCAP subscription
        self.mocap_subscription = self.create_subscription(
            Odometry,
            "Mocap/odom",
            self.com.send_mocap,
            qos_profile_sensor_data
        )
        # Set points
        self.create_subscription(
            AttitudeThrustSetPoint,
            'AttitudeThrustSetPoint',
            self.attitude_thrust_set_point_callback,
            qos_profile_sensor_data
        )
        self.create_subscription(
            RatesThrustSetPoint,
            'RatesThrustSetPoint',
            self.rates_thrust_set_point_callback,
            qos_profile_sensor_data
        )
        self.create_subscription(
            JointTrajectory,
            'Trajectory',
            self.trajectory_callback,
            qos_profile_sensor_data
        )
        # Publish the attitude thrust internally computed
        self.attitude_thrust_publisher = self.create_publisher(
            AttitudeThrustSetPoint,
            'AttitudeThrustCommanded',
            qos_profile_sensor_data)
        # subscribers related to disturbances observation
        self.create_subscription(
            Vector3Stamped,
            'Observer/DisturbancesWorld',
            self.disturbances_callback,
            qos_profile_sensor_data
        )
        self.recovery_timer = self.create_timer(10.0, self.recover_from_emergency)
        self.recovery_timer.cancel()
        self.status.lastAlive = self.get_clock().now()

    odometry = FullState()
    previous_loop = 0.0
    status = Status()
    controller = None
    arming_time_init = None
    land_time_init = None
    to_time_init = None
    to_final_height = None
    to_z_init = None
    stopping_time_init = None
    stopping_thrust_init = None
    thrust_landing_counter = 0

    # Keep alive
    def keep_alive_callback(self, received_msg):
        if not received_msg.feedback:
            if received_msg.keep_alive:
                self.status.lastAlive = self.get_clock().now()
            msg = KeepAlive()
            msg.stamp = self.get_clock().now().to_msg()
            msg.keep_alive = True
            msg.origin_stamp = received_msg.stamp
            msg.feedback = True
            self.keep_alive_publisher.publish(msg)

    def keep_alive_check(self):
        if (self.get_clock().now() - self.status.lastAlive).nanoseconds > 5e8 and \
                (self.status.status != DroneStatus.IDLE):
            if not self.status.status == DroneStatus.EMERGENCY_STOP:
                self.get_logger().info("Keep alive lost, emergency shutdown")
                self.emergency_stop()
            self.recovery_timer.reset()  # We do not recover from emergency stop unless we get keep alive again

    def emergency_stop(self):
        # Emergency stop, stop sending data to the controller
        # and let the autopilot react with its own failsafe parameters
        if self.status.status != DroneStatus.EMERGENCY_STOP:
            self.get_logger().warn("Drone Emergency Stop")
            self.status.status = DroneStatus.EMERGENCY_STOP
            self.controller = None

    def recover_from_emergency(self):
        self.get_logger().info("Recovered from Emergency Stop")
        self.recovery_timer.cancel()
        self.status.status = DroneStatus.IDLE

    # Bridge feedback
    def publish_status(self):
        msg = DroneStatus()
        msg.status = self.status.status
        self.status_publisher.publish(msg)

    # Drone feedback actions
    def update_armed(self, armed):
        if self.status.status == DroneStatus.PRE_ARMED and armed:
            self.get_logger().info("Drone Armed")
            self.status.status = DroneStatus.ARMED
        if self.status.status == DroneStatus.IDLE and armed:
            self.get_logger().warning("Drone was armed while IDLE, I hope you are in manual!")
        if (self.status.status not in {DroneStatus.IDLE,
                                       DroneStatus.EMERGENCY_STOP,
                                       DroneStatus.PRE_ARMED}) and not armed:
            self.get_logger().info("Drone Disarmed")
            self.status.status = DroneStatus.IDLE
            self.controller = None
            self.com.set_manual()

    def update_battery_voltage(self, voltage):
        self.status.battery_voltage = voltage
        # update max thrust parameter as function of the battery voltage
        if self.use_battery_for_thrust():
            new_max_thrust = self.max_thrust_b() + self.max_thrust_a() * self.status.battery_voltage
            self.set_parameters([
                Parameter('max_thrust', Parameter.Type.DOUBLE, new_max_thrust)
            ])

    def update_states(self, odometry):
        self.odometry = odometry
        # Broadcast odometry in reference frame (front, left, up)
        odom = Odometry()
        odom.pose.pose.position.x = odometry.position[0]
        odom.pose.pose.position.y = odometry.position[1]
        odom.pose.pose.position.z = odometry.position[2]
        odom.pose.pose.orientation.w = odometry.orientation[0]
        odom.pose.pose.orientation.x = odometry.orientation[1]
        odom.pose.pose.orientation.y = odometry.orientation[2]
        odom.pose.pose.orientation.z = odometry.orientation[3]
        odom.twist.twist.linear.x = odometry.velocity[0]
        odom.twist.twist.linear.y = odometry.velocity[1]
        odom.twist.twist.linear.z = odometry.velocity[2]
        odom.twist.twist.angular.x = odometry.angular_velocity[0]
        odom.twist.twist.angular.y = odometry.angular_velocity[1]
        odom.twist.twist.angular.z = odometry.angular_velocity[2]
        odom.header.stamp.sec = odometry.sec
        odom.header.stamp.nanosec = odometry.nanosec
        self.odom_publisher.publish(odom)

    # Requests handling
    def handle_requests(self, request: DroneRequest.Request, response: DroneRequest.Response):
        request_str = DroneRequestString[request.request]
        # Conditions to accept the request
        conditions = [None] * DroneRequest.Request.LAST
        conditions[DroneRequest.Request.SPIN_MOTORS] = {DroneStatus.IDLE}
        conditions[DroneRequest.Request.TAKE_OFF] = {DroneStatus.ARMED}
        conditions[DroneRequest.Request.LAND] = {DroneStatus.FLYING, DroneStatus.TAKE_OFF}
        conditions[DroneRequest.Request.POSITION_CONTROL] = {DroneStatus.FLYING}
        conditions[DroneRequest.Request.VELOCITY_CONTROL] = {DroneStatus.FLYING}
        conditions[DroneRequest.Request.ACCELERATION_CONTROL] = {DroneStatus.FLYING}
        # Attitude control, rate thrust control can skip the take_off phase
        conditions[DroneRequest.Request.ATTITUDE_THRUST_CONTROL] = {DroneStatus.ARMED, DroneStatus.FLYING}
        conditions[DroneRequest.Request.RATES_THRUST_CONTROL] = {DroneStatus.ARMED, DroneStatus.FLYING}
        # Actions to take
        actions = [None] * DroneRequest.Request.LAST
        actions[DroneRequest.Request.SPIN_MOTORS] = self.switch_arm_and_spin
        actions[DroneRequest.Request.TAKE_OFF] = self.switch_take_off
        actions[DroneRequest.Request.LAND] = self.switch_land
        actions[DroneRequest.Request.POSITION_CONTROL] = self.switch_position_control
        actions[DroneRequest.Request.VELOCITY_CONTROL] = self.switch_velocity_control
        actions[DroneRequest.Request.ACCELERATION_CONTROL] = self.switch_acceleration_control
        actions[DroneRequest.Request.ATTITUDE_THRUST_CONTROL] = self.switch_attitude_thrust_control
        actions[DroneRequest.Request.RATES_THRUST_CONTROL] = self.switch_rates_thrust_control

        self.get_logger().info(request_str + " request received")
        if request.request < DroneRequest.Request.LAST:
            if self.status.status in conditions[request.request]:
                response = actions[request.request]()
            else:
                response.success = False
                response.message = request_str + " request refused, drone status is not " + \
                                   ' or '.join(
                                       [DroneStatusString[condition] for condition in conditions[request.request]]) + \
                                   '. Current status ' + DroneStatusString[self.status.status] + '.'

        else:
            response.success = False
            response.message = "Invalid request"
        self.get_logger().info(response.message)
        return response

    # Switch function must be immediate to not stuck the system
    # Long term actions must be handled by functions called by the main loop
    def switch_arm_and_spin(self) -> DroneRequest.Response:
        response = DroneRequest.Response()
        self.controller = Controller(self)
        self.controller.desired_scaled_thrust = 0.03
        self.controller.desired_state = self.odometry
        self.status.status = DroneStatus.PRE_ARMED
        self.arming_time_init = self.get_clock().now().nanoseconds
        response.success = True
        response.message = "Starting arming procedure"
        return response

    def arm_and_spin(self):
        if (self.get_clock().now().nanoseconds - self.arming_time_init) > 1e9:
            if self.status.status != DroneStatus.ARMED:
                self.get_logger().info("Arming procedure failed")
            if self.status.status not in {DroneStatus.IDLE, DroneStatus.EMERGENCY_STOP}:
                self.status.status = DroneStatus.IDLE
            self.controller = None
            return
        elif not self.com.offboard:
            self.com.set_offboard()
        elif self.status.status != DroneStatus.ARMED:
            self.com.set_arm(True)

    def switch_take_off(self) -> DroneRequest.Response:
        response = DroneRequest.Response()
        self.status.status = DroneStatus.TAKE_OFF
        self.to_time_init = self.get_clock().now().nanoseconds
        self.controller = PositionController(self)
        self.controller.desired_state = self.odometry
        self.to_final_height = self.take_off_height()
        if self.to_final_height > 2.0:
            self.to_final_height = 2.0
            self.get_logger().warn("Take off height limited to 2.0 m")
        self.to_z_init = self.odometry.position[2]
        response.message = "Taking off"
        response.success = True
        return response

    def take_off(self):
        duration = 3.0
        curr_time = (self.get_clock().now().nanoseconds - self.to_time_init) / 1.0e9
        if curr_time < duration and self.status.status == DroneStatus.TAKE_OFF:
            self.controller.desired_state.position[2] = \
                (self.to_final_height - self.to_z_init) * curr_time / duration + self.to_z_init
        else:
            self.status.status = DroneStatus.FLYING
            self.get_logger().info("Take off completed. Drone hovering in position control.")

    def switch_position_control(self) -> DroneRequest.Response:
        self.controller = PositionController(self)
        self.controller.desired_state = self.odometry
        response = DroneRequest.Response()
        response.success = True
        response.message = "Position control"
        return response

    def switch_velocity_control(self) -> DroneRequest.Response:
        self.controller = VelocityController(self)
        self.controller.desired_state = self.odometry
        self.controller.desired_state.velocity = np.array([0.0, 0.0, 0.0])
        self.controller.desired_state.acceleration = np.array([0.0, 0.0, 0.0])
        response = DroneRequest.Response()
        response.success = True
        response.message = "Velocity control"
        return response

    def switch_acceleration_control(self) -> DroneRequest.Response:
        self.controller = AccelerationController(self)
        self.controller.desired_state = self.odometry
        self.controller.desired_state.acceleration = np.array([0.0, 0.0, 0.0])
        response = DroneRequest.Response()
        response.success = True
        response.message = "Acceleration control"
        return response

    def switch_attitude_thrust_control(self) -> DroneRequest.Response:
        self.controller = AttitudeThrustController(self)
        response = DroneRequest.Response()
        self.status.status = DroneStatus.FLYING
        response.success = True
        response.message = "Attitude thrust control"
        return response

    def switch_rates_thrust_control(self) -> DroneRequest.Response:
        self.controller = RatesThrustController(self)
        response = DroneRequest.Response()
        self.status.status = DroneStatus.FLYING
        response.success = True
        response.message = "Rates thrust control"
        return response

    def switch_land(self) -> DroneRequest.Response:
        response = DroneRequest.Response()
        self.status.status = DroneStatus.LANDING
        self.land_time_init = self.get_clock().now().nanoseconds
        self.controller = VelocityController(self)
        self.controller.desired_state = self.odometry
        self.controller.desired_state.velocity[0] = 0.0
        self.controller.desired_state.velocity[1] = 0.0
        self.controller.desired_state.velocity[2] = -0.5
        self.controller.desired_state.acceleration = np.array([0.0, 0.0, 0.0])
        self.thrust_landing_counter = 0
        if self.odometry.position[2] > 2.5:
            response.success = False
            response.message = "Landing refused, drone height over 2.5 m"
            return response
        response.success = True
        response.message = "Landing"
        return response

    def land(self):
        curr_time = (self.get_clock().now().nanoseconds - self.land_time_init) / 1.0e9
        self.controller.desired_state.velocity[0] = 0.0
        self.controller.desired_state.velocity[1] = 0.0
        self.controller.desired_state.velocity[2] = -0.5
        self.controller.desired_state.acceleration = np.array([0.0, 0.0, 0.0])
        if curr_time > 7.0:
            self.controller = PositionController(self)
            self.controller.desired_state = self.odometry
            self.status.status = DroneStatus.FLYING
            self.get_logger().warn("Landing failed. Null vertical velocity not reached. Hover in position control.")
            return
        # Check if we are lower than the hovering thrust with no vertical velocity
        if self.controller.desired_scaled_thrust * self.max_thrust() < 0.95 * self.mass() * 9.81:
            self.thrust_landing_counter += 1
        else:
            self.thrust_landing_counter = 0
        if self.thrust_landing_counter > 100 and self.odometry.velocity[2] > -0.01:
            self.status.status = DroneStatus.STOPPING
            self.get_logger().info("Stopping drone")
            self.controller = Controller(self)
            self.stopping_time_init = self.get_clock().now().nanoseconds
            self.stopping_thrust_init = self.controller.desired_scaled_thrust

    def stop(self):
        duration = 0.5
        curr_time = (self.get_clock().now().nanoseconds - self.land_time_init) / 1.0e9
        alpha = 1.0 - curr_time / duration
        if alpha < 0:
            alpha = 0
        self.com.set_attitude(self.odometry, self.stopping_thrust_init * alpha)

    # This loop is triggered each time odometry is received (driven by the drone)
    def main_loop(self, odom):
        # Enforce the emergency stop
        if self.status.status == DroneStatus.EMERGENCY_STOP:
            self.controller = None
        # Update data
        self.update_states(odom)
        time_step = (odom.sec + odom.nanosec / 1e9) - self.previous_loop
        self.previous_loop = odom.sec + odom.nanosec / 1e9
        # Arming
        if self.controller is not None and self.status.status in {DroneStatus.PRE_ARMED, DroneStatus.ARMED}:
            desired_scaled_thrust = 0.05
            self.com.set_attitude(self.odometry, desired_scaled_thrust)
            if self.status.status == DroneStatus.PRE_ARMED:
                self.arm_and_spin()
        # Taking off, Flying and Landing
        if self.controller is not None and self.status.status in {DroneStatus.TAKE_OFF,
                                                                  DroneStatus.FLYING, DroneStatus.LANDING}:
            self.controller.do_control(time_step)
            if self.status.status == DroneStatus.TAKE_OFF:
                self.take_off()
            if self.status.status == DroneStatus.LANDING:
                self.land()
        # Stopping
        if self.controller is not None and self.status.status == DroneStatus.STOPPING:
            self.stop()

    # Other ros topic callbacks
    def attitude_thrust_set_point_callback(self, msg):
        if self.controller is not None:
            if self.controller.type == ControllerType.ATTITUDE_THRUST:
                if self.status.status == DroneStatus.FLYING:
                    self.controller.update_attitude_thrust(msg)

    def rates_thrust_set_point_callback(self, msg):
        if self.controller is not None:
            if self.controller.type == ControllerType.RATES_THRUST:
                if self.status.status == DroneStatus.FLYING:
                    self.controller.update_rates_thrust(msg)

    def trajectory_callback(self, msg):
        if self.controller is not None:
            if self.controller.type in [ControllerType.POSITION, ControllerType.VELOCITY, ControllerType.ACCELERATION]:
                if self.status.status == DroneStatus.FLYING:
                    self.controller.update_setpoints(msg)

    def disturbances_callback(self, msg):
        if self.controller is not None:
            self.controller.update_disturbances(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DroneBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutting down drone bridge')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == 'main':
    main()
