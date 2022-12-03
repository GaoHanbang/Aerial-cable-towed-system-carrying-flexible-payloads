import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Empty
from ls2n_interfaces.srv import *
from ls2n_interfaces.msg import *
qos_profile_sensor_data.depth = 1


class DronePositionControl(Node):
    def __init__(self):
        super().__init__('topic_position_control')
        self.create_service(
            Empty,
            "SpinMotors",
            self.spin_motors,
        )
        self.create_service(
            Empty,
            "StartExperiment",
            self.start_experiment
        )
        self.create_service(
            Empty,
            "StopExperiment",
            self.stop_experiment
        )
        self.drone_request_client = self.create_client(
            DroneRequest,
            "Request",
        )
        self.start_trajectory_client = self.create_client(
            Empty,
            "StartTrajectory"
        )
        self.reset_trajectory_client = self.create_client(
            Empty,
            "ResetTrajectory"
        )
        self.create_subscription(
            DroneStatus,
            "Status",
            self.update_status,
            qos_profile_sensor_data,
        )
        self.experiment_started = False
        self.landing = False

    def update_status(self, msg: DroneStatus):
        if msg.status == DroneStatus.FLYING and not self.experiment_started and not self.landing:
            self.experiment_started = True
            self.start_trajectory_client.call_async(Empty.Request())
        if msg.status == DroneStatus.IDLE and self.landing:
            self.landing = False

    def spin_motors(self, request, response):
        if not self.experiment_started:
            request_out = DroneRequest.Request()
            request_out.request = DroneRequest.Request.SPIN_MOTORS
            self.drone_request_client.call_async(request_out)
        else:
            self.get_logger().info("Experiment already started. Stop experiment to reset.")
        return response

    def start_experiment(self, request, response):
        if not self.experiment_started:
            request_out = DroneRequest.Request()
            request_out.request = DroneRequest.Request.TAKE_OFF
            self.drone_request_client.call_async(request_out)
        else:
            self.get_logger().info("Experiment already started. Stop experiment to reset.")
        return response

    def stop_experiment(self, request, response):
        self.get_logger().info("Stopping drone")
        request_out = DroneRequest.Request()
        request_out.request = DroneRequest.Request.LAND
        self.drone_request_client.call_async(request_out)
        self.reset_trajectory_client.call_async(Empty.Request())
        self.experiment_started = False
        self.landing = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DronePositionControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutting down drone position control')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == 'main':
    main()
