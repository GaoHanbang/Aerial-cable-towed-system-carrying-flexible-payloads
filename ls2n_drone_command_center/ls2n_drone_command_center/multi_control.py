import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Empty
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from functools import partial
from ls2n_drone_bridge.common import DroneRequestString, DroneStatusString
from ls2n_interfaces.srv import *
from ls2n_interfaces.msg import *

qos_profile_sensor_data.depth = 1


class MultiControl(Node):
    """
    Multi control algo
    """

    def __init__(self):
        #
        super().__init__('multi_control')
        drones_to_control_des = ParameterDescriptor(type=ParameterType.PARAMETER_STRING_ARRAY,
                                                    description='Array of drones you want to control')
        self.declare_parameter('drones_to_control', None, descriptor=drones_to_control_des)
        self.drones = self.get_parameter('drones_to_control').value
        self.get_logger().info("Starting multi control drones")
        self.drone_request_client = []
        self.drone_keep_alive_pub = []
        self.status_drone_sub = []
        self.drone_status = []
        self.status = DroneStatus()
        self.status.status = DroneStatus.IDLE
        self.arming_time_init = None
        self.takeoff_time_init = None
        self.land_time_init = None
        self.emergency_stop = False
        self.loop_arm = False
        self.land_loop = False
        self.loop_arm_timer = None
        self.loop_land_timer = None

        self.status_pub = self.create_publisher(
            DroneStatus,
            "Status",
            qos_profile_sensor_data
        )
        self.create_service(
            DroneRequest,
            "Request",
            self.handle_request
        )
        self.keep_alive_sub = self.create_subscription(
            KeepAlive,
            "KeepAlive",
            self.keep_alive,
            qos_profile_sensor_data
        )
        for index, drone_namespace in enumerate(self.drones):
            self.drone_status.append(DroneStatus.IDLE)
            self.drone_request_client.append(self.create_client(
                DroneRequest,
                "/" + drone_namespace + "/Request"
            ))
            self.drone_keep_alive_pub.append(self.create_publisher(
                KeepAlive,
                "/" + drone_namespace + "/KeepAlive",
                qos_profile_sensor_data
            ))
            self.status_drone_sub.append(self.create_subscription(
                DroneStatus,
                "/" + drone_namespace + "/Status",
                partial(self.update_drone_status, drone_index=index),
                qos_profile_sensor_data
            ))
        self.timer_watch = self.create_timer(0.05, self.watch_drones_status)
        self.timer_wait = None

    def handle_request(self, request: DroneRequest.Request, response: DroneRequest.Response):
        request_str = DroneRequestString[request.request]
        """
        This function manages user requests
        :return response:
        """
        conditions = [None] * DroneRequest.Request.LAST
        conditions[DroneRequest.Request.SPIN_MOTORS] = {DroneStatus.IDLE}
        conditions[DroneRequest.Request.TAKE_OFF] = {DroneStatus.ARMED}
        conditions[DroneRequest.Request.LAND] = {DroneStatus.FLYING, DroneStatus.TAKE_OFF}
        conditions[DroneRequest.Request.POSITION_CONTROL] = {DroneStatus.FLYING}
        conditions[DroneRequest.Request.VELOCITY_CONTROL] = {DroneStatus.FLYING}
        conditions[DroneRequest.Request.ACCELERATION_CONTROL] = {DroneStatus.FLYING}
        conditions[DroneRequest.Request.ATTITUDE_THRUST_CONTROL] = {DroneStatus.ARMED, DroneStatus.FLYING}
        conditions[DroneRequest.Request.RATES_THRUST_CONTROL] = {DroneStatus.ARMED, DroneStatus.FLYING}

        actions = [None] * DroneRequest.Request.LAST
        actions[DroneRequest.Request.SPIN_MOTORS] = self.start_arm_spin_motors
        actions[DroneRequest.Request.TAKE_OFF] = self.start_take_off
        actions[DroneRequest.Request.LAND] = self.start_land
        actions[DroneRequest.Request.POSITION_CONTROL] = self.start_position_control
        actions[DroneRequest.Request.VELOCITY_CONTROL] = self.start_velocity_control
        actions[DroneRequest.Request.ACCELERATION_CONTROL] = self.start_acceleration_control
        actions[DroneRequest.Request.ATTITUDE_THRUST_CONTROL] = self.start_attitude_thrust_control
        actions[DroneRequest.Request.RATES_THRUST_CONTROL] = self.start_rates_thrust_control

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

        return response

    def keep_alive(self, keepAlive):
        """
        This function sends keep alive to each drone
        :param keepAlive:
        """
        msg = KeepAlive()
        msg.keep_alive = keepAlive.keep_alive
        msg.feedback = keepAlive.feedback
        for i in range(len(self.drone_keep_alive_pub)):
            self.drone_keep_alive_pub[i].publish(msg)

    def watch_drones_status(self):
        """
        This function monitors the status of the drones and applies the associated procedure and the
        updates the status of the node.
        """
        if self.status.status in {DroneStatus.ARMED, DroneStatus.STOPPING} \
                and all(self.drone_status[i] == DroneStatus.IDLE for i in range(len(self.drone_status))):
            self.status.status = DroneStatus.IDLE

        if self.status.status == DroneStatus.PRE_ARMED:
            self.arm_spin_process()

        if self.status.status == DroneStatus.TAKE_OFF:
            self.take_off_process()

        if self.status.status == DroneStatus.LANDING:
            self.land_process()

        if self.status.status != DroneStatus.IDLE and any(self.drone_status[i] == DroneStatus.EMERGENCY_STOP \
                                                          for i in range(len(self.drone_status))):
            self.status.status = DroneStatus.EMERGENCY_STOP

        self.update_status()

    def update_status(self):
        """
        This function publishes the state of the node
        """
        msg = DroneStatus()
        msg.status = self.status.status
        self.status_pub.publish(msg)

    def update_drone_status(self, msg: DroneStatus, drone_index: int):
        """
        This function updates the status of the drones
        :param msg: Status of drone
        :param drone_index: Number of drone
        """
        self.drone_status[drone_index] = msg.status

    def send_request_drone(self, request, desired_status=None):
        """
        This function send requests to each drone
        :param desired_status: The desired status for faulty drones that require a request again
        :param request: Request to send to the drone
        """
        request_out = DroneRequest.Request()
        request_out.request = request
        if desired_status is None:
            for i in range(len(self.drone_request_client)):
                self.drone_request_client[i].call_async(request_out)
        else:
            for i in range(len(self.drone_request_client)):
                if self.drone_status[i] not in {DroneStatus.IDLE, desired_status}:
                    self.drone_request_client[i].call_async(request_out)

    def start_arm_spin_motors(self):
        """
        This function starts the pre-arming procedure of the node
        :return response: The response of the service.
        """
        response = DroneRequest.Response()
        self.arming_time_init = self.get_clock().now().nanoseconds
        self.status.status = DroneStatus.PRE_ARMED
        self.send_request_drone(DroneRequest.Request.SPIN_MOTORS)
        response.success = True
        response.message = "Starting arming procedure and spin motors ..."
        return response

    def repeat_arm_spin(self):
        self.get_logger().info("Arming and spin procedure failed: Repeat the command again ...")
        self.send_request_drone(DroneRequest.Request.SPIN_MOTORS)

    def arm_spin_process(self):
        """
        This procedure is executed when the status of the node is PRE_ARMED
        """
        if all(self.drone_status[i] == DroneStatus.ARMED for i in range(len(self.drone_status))):
            if self.loop_arm:
                self.loop_arm = False
                self.loop_arm_timer.destroy()
            self.status.status = DroneStatus.ARMED
            self.get_logger().info("Successful Arming !")

        if any(self.drone_status[i] in {DroneStatus.IDLE, DroneStatus.PRE_ARMED} \
               for i in range(len(self.drone_status))):
            time = (self.get_clock().now().nanoseconds - self.arming_time_init) / 1e9
            if time > 1 and not self.loop_arm:
                if self.status.status != DroneStatus.ARMED:
                    self.get_logger().info("Arming and spin procedure failed: Repeat the command again ...")
                    self.send_request_drone(DroneRequest.Request.SPIN_MOTORS)
                    self.loop_arm_timer = self.create_timer(1, self.repeat_arm_spin)
                    self.loop_arm = True
            if time > 3:
                self.loop_arm = False
                self.loop_arm_timer.destroy()
                self.timer_watch.destroy()
                self.get_logger().info("Arming procedure and spin motors failed : EMERGENCY STOP")
                self.status.status = DroneStatus.EMERGENCY_STOP
                self.update_status()

        if any(self.drone_status[i] not in {DroneStatus.IDLE, DroneStatus.PRE_ARMED, DroneStatus.ARMED} \
               for i in range(len(self.drone_status))):
            self.get_logger().info("Emergency stop")
            self.timer_watch.destroy()
            self.status.status = DroneStatus.EMERGENCY_STOP
            self.update_status()

    def start_take_off(self):
        """
        This function starts the takeoff procedure of the node
        :return response: The response of the service
        """
        response = DroneRequest.Response()
        self.takeoff_time_init = self.get_clock().now().nanoseconds
        self.status.status = DroneStatus.TAKE_OFF
        self.send_request_drone(DroneRequest.Request.TAKE_OFF)
        response.success = True
        response.message = "Starting takeoff procedure ..."
        return response

    def take_off_process(self):
        """
        This procedure is executed when the status of the node is TAKE_OFF
        """
        if all(self.drone_status[i] == DroneStatus.FLYING for i in range(len(self.drone_status))):
            self.status.status = DroneStatus.FLYING
            self.get_logger().info("Successful take-off !")

        if any(self.drone_status[i] not in {DroneStatus.TAKE_OFF, DroneStatus.FLYING} \
               for i in range(len(self.drone_status))) \
                and (self.get_clock().now().nanoseconds - self.takeoff_time_init) > 4e9:
            self.emergency_landing()

    def start_position_control(self):
        """
        This function starts position control procedure of node
        :return response: The response of the service
        """
        response = DroneRequest.Response()
        self.send_request_drone(DroneRequest.Request.POSITION_CONTROL)
        response.success = True
        response.message = "Starting position control procedure ..."
        return response

    def start_velocity_control(self):
        """
       This function starts velocity control procedure of node
       :return response: The response of the service
       """
        response = DroneRequest.Response()
        self.send_request_drone(DroneRequest.Request.VELOCITY_CONTROL)
        response.success = True
        response.message = "Starting velocity control procedure ..."
        return response

    def start_acceleration_control(self):
        """
        This function starts acceleration control of node
        :return response: The response of the service
        """
        response = DroneRequest.Response()
        self.send_request_drone(DroneRequest.Request.ACCELERATION_CONTROL)
        response.success = True
        response.message = "Starting acceleration control procedure ..."
        return response

    def start_attitude_thrust_control(self):
        """
        :return:
        """
        response = DroneRequest.Response()
        self.send_request_drone(DroneRequest.Request.ATTITUDE_THRUST_CONTROL)
        self.status.status = DroneStatus.FLYING
        response.success = True
        response.message = "Starting attitude thrust control procedure"

    def start_rates_thrust_control(self):
        """
        :return:
        """
        response = DroneRequest.Response()
        self.send_request_drone(DroneRequest.Request.RATES_THRUST_CONTROL)
        self.status.status = DroneStatus.FLYING
        response.success = True
        response.message = "Starting rates thrust control procedure"

    def start_land(self):
        """
        :return:
        """
        response = DroneRequest.Response()
        self.land_time_init = self.get_clock().now().nanoseconds
        self.status.status = DroneStatus.LANDING
        self.send_request_drone(DroneRequest.Request.LAND)
        response.success = True
        response.message = "Starting landing procedure ..."
        return response

    def land_process(self):
        """
        This procedure is executed when the status of the node is LANDING
        """
        if all(self.drone_status[i] == DroneStatus.STOPPING for i in range(len(self.drone_status))):
            if self.land_loop:
                self.land_loop = True
                self.loop_land_timer.destroy()
            self.status.status = DroneStatus.STOPPING
            self.get_logger().info("Successful landing ! ")

        if any(self.drone_status[i] != DroneStatus.STOPPING for i in range(len(self.drone_status))):
            time = (self.get_clock().now().nanoseconds - self.land_time_init) / 1e9
            if 4 < time < 15 and not self.land_loop:
                self.loop_land_timer = self.create_timer(3, self.repeat_land)
                self.land_loop = True
            if time > 15:
                self.land_loop = False
                self.loop_land_timer.destroy()
                self.get_logger().info("Landing procedure failed")
                self.emergency_landing()

    def repeat_land(self):
        self.get_logger().info("Landing procedure failed: Repeat the command again ...")
        self.send_request_drone(DroneRequest.Request.LAND, DroneStatus.STOPPING)

    def emergency_landing(self):
        """
        :return:
        """
        self.status.status = DroneStatus.EMERGENCY_STOP
        self.timer_watch.destroy()
        self.get_logger().info("Emergency landing")
        self.send_request_drone(DroneRequest.Request.LAND)
        self.update_status()


def main(args=None):
    rclpy.init(args=args)
    multi_control = MultiControl()
    try:
        rclpy.spin(multi_control)
    except KeyboardInterrupt:
        print('Shutting down drone multi control drone')
    finally:
        multi_control.destroy_node()
        rclpy.shutdown()


if __name__ == 'main':
    main()
