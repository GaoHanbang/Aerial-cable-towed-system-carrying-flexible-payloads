import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from builtin_interfaces.msg import Duration
from ament_index_python.packages import get_package_share_directory
from math import floor
import os
import pickle

qos_profile_sensor_data.depth = 1


class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        trajname_descriptor = ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                                  description='Name of the trajectory')
        self.declare_parameter("trajectory", None, trajname_descriptor)
        trajectory_name = self.get_parameter("trajectory").value
        if trajectory_name is None:
            exit("Please provide a trajectory name")
        filename = os.path.join(get_package_share_directory('ls2n_tools'),
                                'trajectories', trajectory_name + ".traj.pickle")
        try:
            file = open(filename, 'rb')
        except FileNotFoundError:
            exit("This trajectory does not exist.")
        self.trajectory = pickle.load(file)

        self.get_logger().info("Starting trajectory publisher")
        self.publisher = self.create_publisher(
            JointTrajectory,
            "Trajectory",
            qos_profile_sensor_data
        )
        self.create_service(
            Empty,
            "StartTrajectory",
            self.start_trajectory
        )
        self.create_service(
            Empty,
            "ResetTrajectory",
            self.reset_trajectory
        )
        self.init_timer = self.create_timer(0.5, self.init_publish)
        self.publish_timer = None
        self.time_start = self.get_clock().now()
        self.index = 0
        self.index_max = len(self.trajectory["time"]) - 1
        self.experiment_started = False

    def init_publish(self):
        self.publish_trajectory()

    def start_trajectory(self, request, response):
        if not self.experiment_started:
            self.get_logger().info("Starting trajectory")
            self.destroy_timer(self.init_timer)
            self.time_start = self.get_clock().now()
            self.publish_timer = self.create_timer(0.001, self.check_and_publish)
            self.index = 0
            self.experiment_started = True
        else:
            self.get_logger().info("Trajectory already started.")
        return response

    def reset_trajectory(self, request, response):
        if self.experiment_started:
            self.get_logger().info("Reset trajectory")
            # Reset everything
            self.experiment_started = False
            self.destroy_timer(self.publish_timer)
            self.index = 0
            self.init_timer = self.create_timer(0.5, self.init_publish)
        return response

    def check_and_publish(self):
        time_elapsed = (self.get_clock().now() - self.time_start).nanoseconds / 1e9
        if self.index < self.index_max:
            if self.trajectory["time"][self.index + 1] < time_elapsed:
                self.index += 1
                self.publish_trajectory()

    def publish_trajectory(self):
        msg = JointTrajectory()
        msg.header.stamp = self.get_clock().now().to_msg()
        point_traj = JointTrajectoryPoint()
        positions = []
        velocities = []
        accelerations = []
        time_traj = self.trajectory["time"][self.index]
        point_traj.time_from_start = Duration(sec=floor(time_traj), nanosec=int((time_traj % 1)*1e9))
        for coordinate in self.trajectory.keys():
            if coordinate != "time" and coordinate[-1] != "D":
                msg.joint_names.append(coordinate)
                positions.append(self.trajectory[coordinate][self.index])
                if coordinate + "D" in self.trajectory:
                    velocities.append(self.trajectory[coordinate + "D"][self.index])
                if coordinate + "DD" in self.trajectory:
                    accelerations.append(self.trajectory[coordinate + "DD"][self.index])
        point_traj.positions = positions
        point_traj.velocities = velocities
        point_traj.accelerations = accelerations
        msg.points.append(point_traj)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher = TrajectoryPublisher()
    try:
        rclpy.spin(trajectory_publisher)
    except KeyboardInterrupt:
        print('Shutting down trajectory publisher')
    finally:
        trajectory_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == 'main':
    main()
