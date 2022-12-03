import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Empty
from ls2n_interfaces.srv import *
from ls2n_interfaces.msg import *
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from trajectory_msgs.msg import JointTrajectory,JointTrajectoryPoint
from std_msgs.msg import Float32
qos_profile_sensor_data.depth = 1


class DronePositionControl(Node):
    def __init__(self):
        super().__init__('joystick_velocity_control')
        max_velocity_param_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description="Maximum drone velocity")
        self.declare_parameter('max_velocity', 1.0, max_velocity_param_descriptor)
        self.v_max = self.get_parameter('max_velocity').value
        self.velocity_command = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}

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
        self.create_subscription(
            DroneStatus,
            "Status",
            self.update_status,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Float32,
            'Joystick/LeftVertical',
            self.x_command,
            qos_profile_sensor_data
        )
        self.create_subscription(
            Float32,
            'Joystick/LeftHorizontal',
            self.y_command,
            qos_profile_sensor_data
        )
        self.create_subscription(
            Float32,
            'Joystick/RightVertical',
            self.z_command,
            qos_profile_sensor_data
        )
        self.create_subscription(
            Float32,
            'Joystick/RightHorizontal',
            self.yaw_command,
            qos_profile_sensor_data
        )
        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            'Trajectory',
            qos_profile_sensor_data
        )
        self.create_timer(0.01, self.publish_trajectory)
        self.experiment_started = False

    def update_status(self, msg: DroneStatus):
        if msg.status == DroneStatus.FLYING and not self.experiment_started:
            self.experiment_started = True
            request = DroneRequest.Request()
            request.request = DroneRequest.Request.VELOCITY_CONTROL
            self.drone_request_client.call_async(request)

    def x_command(self, msg: Float32):
        self.velocity_command["x"] = msg.data * self.v_max

    def y_command(self, msg: Float32):
        self.velocity_command["y"] = msg.data * self.v_max

    def z_command(self, msg: Float32):
        self.velocity_command["z"] = msg.data * self.v_max

    def yaw_command(self, msg: Float32):
        self.velocity_command["yaw"] = msg.data * self.v_max

    def publish_trajectory(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        point_traj = JointTrajectoryPoint()
        msg.joint_names = ["x", "y", "z", "yaw"]
        point_traj.velocities = [
            self.velocity_command['x'],
            self.velocity_command['y'],
            self.velocity_command['z'],
            self.velocity_command['yaw']]
        msg.points = [point_traj]
        self.trajectory_pub.publish(msg)

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
        self.experiment_started = False
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DronePositionControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutting down drone velocity control')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == 'main':
    main()
