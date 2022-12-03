import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from ls2n_interfaces.srv import *
from ls2n_interfaces.msg import *

qos_profile_sensor_data.depth = 1


class DronesCable(Node):
    def __init__(self):
        super().__init__('drones_cable')
        self.mutli_control_status = DroneStatus.IDLE
        self.active = False
        self.create_service(
            Empty,
            'StartExperiment',
            self.start_experimentation
        )
        self.create_service(
            Empty,
            'StopExperiment',
            self.stop_experimentation
        )
        self.create_service(
            Empty,
            "SpinMotors",
            self.spin_motors
        )
        self.create_subscription(
            DroneStatus,
            'Status',
            self.update_status,
            qos_profile_sensor_data
        )
        self.multi_control_client = self.create_client(
            DroneRequest,
            'Request'
        )

    def update_status(self, msg):
        """

        :param msg:
        :return:
        """
        self.mutli_control_status = msg.status
        if self.mutli_control_status == DroneStatus.FLYING:
            pass
            # self.start_attitude_thrust()

    def start_attitude_thrust(self):
        """
        :return:
        """
        if not self.active:
            self.get_logger().info("Activate control")
            request_out = DroneRequest.Request()
            request_out.request = DroneRequest.Request.ATTITUDE_THRUST_CONTROL
            self.multi_control_client.call_async(request_out)
            self.active = True

    def spin_motors(self, request, response):
        """
        :return:
        """
        request_out = DroneRequest.Request()
        request_out.request = DroneRequest.Request.SPIN_MOTORS
        self.multi_control_client.call_async(request_out)

        return response

    def start_experimentation(self, request, response):
        """
        :return:
        """
        request_out = DroneRequest.Request()
        request_out.request = DroneRequest.Request.TAKE_OFF
        self.multi_control_client.call_async(request_out)

        return response

    def stop_experimentation(self, request, response):
        """
        :return:
        """
        self.active = False
        request_out = DroneRequest.Request()
        request_out.request = DroneRequest.Request.LAND
        self.multi_control_client.call_async(request_out)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = DronesCable()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutting down drones and cable')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
