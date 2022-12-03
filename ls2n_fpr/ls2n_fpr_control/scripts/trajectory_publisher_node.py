#!/usr/bin/env python3
import os
import pickle
import rclpy
import numpy as np
from enum import IntEnum
from scipy.spatial.transform import Rotation
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_srvs.srv import Empty
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterDescriptor
from ls2n_fpr_interfaces.msg import FprTrajectory as FprTrajectoryMsg
from ls2n_fpr_interfaces.msg import FprTeleop as FprTeleopMsg
from ament_index_python.packages import get_package_share_directory

qos_profile_sensor_data.depth = 1


class TrajectoryMode(IntEnum):
    AUTO = 0  # autopilot mode, trajectory loaded from file
    MANUAL = 1  # manual mode, pilot by joystick
    MIX = 2  # mix mode, load trajectory and allow piloting by joystick meanwhile


class FprTrajectoryData():
    def __init__(self):
        self.pose = np.zeros(10)
        self.velocity = np.zeros(9)
        self.acceleration = np.zeros(9)

    def parseEulerAngle(self, eul, eulD, eulDD):
        # mapping_mat = np.zeros((3,3))
        mapping_mat = np.array([[1, 0, -np.sin(eul[1])],
                                [0, np.cos(eul[0]), np.sin(eul[0]) * np.cos(eul[1])],
                                [0, -np.sin(eul[0]), np.cos(eul[0]) * np.cos(eul[1])]
                                ])
        mapping_matD = np.zeros((3, 3))
        mapping_matD[0, 2] = -np.cos(eul[1]) * eulD[1]
        mapping_matD[1, 1] = -np.sin(eul[0]) * eulD[0]
        mapping_matD[2, 1] = -np.cos(eul[0]) * eulD[0]
        mapping_matD[1, 2] = np.cos(eul[0]) * np.cos(eul[1]) * eulD[0] - np.sin(eul[0]) * np.sin(eul[1]) * eulD[1]
        mapping_matD[2, 2] = -np.sin(eul[0]) * np.cos(eul[1]) * eulD[0] - np.cos(eul[0]) * np.sin(eul[1]) * eulD[1]

        quat = Rotation.from_euler('xyz', eul, degrees=False).as_quat()  # roll, pitch, yaw angles to quaternion
        self.pose[3:7] = quat
        self.velocity[3:6] = np.dot(mapping_mat, eulD)
        self.acceleration[3:6] = np.dot(mapping_mat, eulDD) + np.dot(mapping_matD, eulD)


class FprTrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('fpr_trajectory_publisher')
        traj_mode_des = ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                            description='Trajectory(control) Mode')
        self.declare_parameter("trajectory_mode", None, traj_mode_des)
        traj_mode = self.get_parameter("trajectory_mode").value
        if traj_mode is None:
            exit("Trajectory mode is not found")
        if traj_mode == "autopilot":
            self.trajectory_mode = TrajectoryMode.AUTO
            pilot_mode_msg = "Autopilot Mode"
        elif traj_mode == "manual":
            self.trajectory_mode = TrajectoryMode.MANUAL
            pilot_mode_msg = "Manual Mode"
        elif traj_mode == "mix":
            self.trajectory_mode = TrajectoryMode.MIX
            pilot_mode_msg = "Mix Mode"

        if self.trajectory_mode == TrajectoryMode.AUTO or self.trajectory_mode == TrajectoryMode.MIX:
            trajfile_des = ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                               description='Trajectory File')
            self.declare_parameter("trajectory_file", None, trajfile_des)
            traj_file = self.get_parameter("trajectory_file").value
            if traj_file is None:
                exit("Trajectory file is not found")
            filename = os.path.join(get_package_share_directory('ls2n_tools'),
                                    'trajectories', traj_file + ".traj.pickle")
            try:
                file = open(filename, 'rb')
            except FileNotFoundError:
                exit("Trajectory file %s does not exist, please generate with ls2n_tools package.", str(traj_file))
            self.trajectory = pickle.load(file)
            self.variables = list(self.trajectory.keys())
            if self.variables[13] != "roll" or self.variables[17] != "pitch" or self.variables[21] != "yaw":
                exit("Orientation only allows roll-pitch-yaw Euler angle representation")
            self.time_index = 0
            self.time_index_max = len(self.trajectory["time"])
            self.trajectory_updated = False

        if self.trajectory_mode == TrajectoryMode.MANUAL or self.trajectory_mode == TrajectoryMode.MIX:
            # subscription to teleoperation msg
            self.teleop_sub = self.create_subscription(FprTeleopMsg,
                                                       "/CommandCenter/fpr_teleop",
                                                       self.teleop_callback,
                                                       qos_profile_sensor_data)
            self.eul_angles = np.zeros(3)  #  platform euler angles
            self.teleop_trajectory = FprTrajectoryData()
            self.teleop_prev_time = -1
            self.teleop_curr_time = -1

        # Trajectory publisher
        self.traj_pub = self.create_publisher(
            FprTrajectoryMsg,
            "fpr_trajectory",
            qos_profile_sensor_data
        )
        # Start experiment service
        self.create_service(
            Empty,
            "StartExperiment",
            self.start_trajectory
        )
        self.fpr_trajectory = FprTrajectoryData()
        self.start_time = -1
        self.trajectory_started = False
        self.get_logger().info("Trajectory publisher initialized in %s, waiting to start" % pilot_mode_msg)

    def start_trajectory(self, request, response):
        if self.trajectory_mode == TrajectoryMode.MANUAL:
            self.get_logger().info("Manual pilot mode")
            return response
        if not self.trajectory_started:
            self.get_logger().info("Start publishing trajectory")
            self.start_time = self.get_clock().now()
            self.publish_timer = self.create_timer(0.001, self.check_and_publish)
            self.trajectory_started = True
        else:
            self.get_logger().info("Publisher already started")
        return response

    def check_and_publish(self):
        time_elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if self.time_index == 0:
            self.update_trajectory()
            self.time_index += 1
        elif self.time_index < self.time_index_max:
            if self.trajectory["time"][self.time_index] < time_elapsed:
                self.update_trajectory()
                self.time_index += 1
        else:
            self.get_logger().warn("Trajectory finished")
            self.publish_timer.destroy()
            # if self.trajectory_mode == TrajectoryMode.MIX:
            #     self.trajectory_mode = TrajectoryMode.MANUAL  # back to manual mode for safety reason
            #     self.get_logger().warn("Back to Manual Mode")

        if self.trajectory_mode == TrajectoryMode.AUTO:
            if self.trajectory_updated:
                self.publish_trajectory()
                self.trajectory_updated = False

    def update_trajectory(self):
        pos_raw = np.zeros(9)
        vel_raw = np.zeros(9)
        accel_raw = np.zeros(9)
        var_index = 0
        for var in self.variables:
            if var != "time" and var[-1] != "D":
                pos_raw[var_index] = self.trajectory[var][self.time_index]
                if var + "D" in self.trajectory:
                    vel_raw[var_index] = self.trajectory[var + "D"][self.time_index]
                if var + "DD" in self.trajectory:
                    accel_raw[var_index] = self.trajectory[var + "DD"][self.time_index]
                var_index += 1
        self.fpr_trajectory.pose[0:3] = pos_raw[0:3]
        self.fpr_trajectory.pose[7:10] = pos_raw[6:9]
        self.fpr_trajectory.velocity[0:3] = vel_raw[0:3]
        self.fpr_trajectory.velocity[6:9] = vel_raw[6:9]
        self.fpr_trajectory.acceleration[0:3] = accel_raw[0:3]
        self.fpr_trajectory.acceleration[6:9] = accel_raw[6:9]
        self.fpr_trajectory.parseEulerAngle(pos_raw[3:6], vel_raw[3:6], accel_raw[3:6])
        self.trajectory_updated = True

    def publish_trajectory(self):
        msg = FprTrajectoryMsg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.platform_pose.position.x = self.fpr_trajectory.pose[0]
        msg.platform_pose.position.y = self.fpr_trajectory.pose[1]
        msg.platform_pose.position.z = self.fpr_trajectory.pose[2]
        msg.platform_pose.orientation.x = self.fpr_trajectory.pose[3]
        msg.platform_pose.orientation.y = self.fpr_trajectory.pose[4]
        msg.platform_pose.orientation.z = self.fpr_trajectory.pose[5]
        msg.platform_pose.orientation.w = self.fpr_trajectory.pose[6]
        msg.platform_twist.linear.x = self.fpr_trajectory.velocity[0]
        msg.platform_twist.linear.y = self.fpr_trajectory.velocity[1]
        msg.platform_twist.linear.z = self.fpr_trajectory.velocity[2]
        msg.platform_twist.angular.x = self.fpr_trajectory.velocity[3]
        msg.platform_twist.angular.y = self.fpr_trajectory.velocity[4]
        msg.platform_twist.angular.z = self.fpr_trajectory.velocity[5]
        msg.platform_accel.linear.x = self.fpr_trajectory.acceleration[0]
        msg.platform_accel.linear.y = self.fpr_trajectory.acceleration[1]
        msg.platform_accel.linear.z = self.fpr_trajectory.acceleration[2]
        msg.platform_accel.angular.x = self.fpr_trajectory.acceleration[3]
        msg.platform_accel.angular.y = self.fpr_trajectory.acceleration[4]
        msg.platform_accel.angular.z = self.fpr_trajectory.acceleration[5]
        for i in range(3):
            msg.leg_angles[i].data = self.fpr_trajectory.pose[i + 7]
            msg.leg_angles_vel[i].data = self.fpr_trajectory.velocity[i + 6]
            msg.leg_angles_accel[i].data = self.fpr_trajectory.acceleration[i + 6]
        self.traj_pub.publish(msg)

    def teleop_callback(self, msg):
        if not self.trajectory_started:  # not receiving any teleoperation commands if trajectory is not started
            return
        if self.teleop_prev_time <= 0 and self.teleop_curr_time <= 0:  # prevent first callback
            self.teleop_prev_time = self.get_clock().now().nanoseconds / 1e9
            self.teleop_curr_time = self.get_clock().now().nanoseconds / 1e9
            return
        # update teleoperation commands
        self.teleop_curr_time = self.get_clock().now().nanoseconds / 1e9
        dt = self.teleop_curr_time - self.teleop_prev_time
        self.teleop_trajectory.pose[0] += msg.platform_linear_vel.x * dt
        self.teleop_trajectory.pose[1] += msg.platform_linear_vel.y * dt
        self.teleop_trajectory.pose[2] += msg.platform_linear_vel.z * dt
        self.teleop_trajectory.velocity[0] = msg.platform_linear_vel.x
        self.teleop_trajectory.velocity[1] = msg.platform_linear_vel.y
        self.teleop_trajectory.velocity[2] = msg.platform_linear_vel.z
        self.teleop_trajectory.acceleration[0] = msg.platform_linear_accel.x
        self.teleop_trajectory.acceleration[1] = msg.platform_linear_accel.y
        self.teleop_trajectory.acceleration[2] = msg.platform_linear_accel.z
        self.eul_angles[0] = msg.platform_roll.data
        self.eul_angles[1] = msg.platform_pitch.data
        self.eul_angles[2] += msg.platform_yaw_rate.data * dt
        eul_rate = np.zeros(3)
        eul_rate[0] = msg.platform_roll_rate.data
        eul_rate[1] = msg.platform_pitch_rate.data
        eul_rate[2] = msg.platform_yaw_rate.data
        eul_accel = np.zeros(3)
        eul_accel[2] = msg.platform_yaw_accel.data
        self.teleop_trajectory.parseEulerAngle(self.eul_angles, eul_rate, eul_accel)
        for i in range(3):
            self.teleop_trajectory.pose[i + 7] = msg.leg_angles[i].data
            self.teleop_trajectory.velocity[i + 6] = msg.leg_angles_vel[i].data
        self.teleop_prev_time = self.teleop_curr_time

        if self.trajectory_mode == TrajectoryMode.MANUAL:
            self.fpr_trajectory = self.teleop_trajectory
            self.publish_trajectory()
        elif self.trajectory_mode == TrajectoryMode.MIX:
            self.fpr_trajectory.pose[0:3] += self.teleop_trajectory.pose[0:3]
            if self.fpr_trajectory.pose[2] < 0.05:
                self.fpr_trajectory.pose[2] = 0.05
            self.fpr_trajectory.pose[3:7] = self.performOrientations(self.fpr_trajectory.pose[3:7],
                                                                     self.teleop_trajectory.pose[3:7])
            self.fpr_trajectory.pose[7:10] = self.teleop_trajectory.pose[7:10]
            self.fpr_trajectory.velocity += self.teleop_trajectory.velocity
            self.fpr_trajectory.acceleration += self.teleop_trajectory.acceleration
            self.publish_trajectory()

    def performOrientations(self, q_a, q_b):
        # perform two orientations given by two unit quaternions, simply by quternion multiplication
        quat = np.zeros(4)
        quat[0] = q_a[3] * q_b[0] + q_a[0] * q_b[3] + q_a[1] * q_b[2] - q_a[2] * q_b[
            1]  # x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y
        quat[1] = q_a[3] * q_b[1] - q_a[0] * q_b[2] + q_a[1] * q_b[3] + q_a[2] * q_b[
            0]  # y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x
        quat[2] = q_a[3] * q_b[2] + q_a[0] * q_b[1] - q_a[1] * q_b[0] + q_a[2] * q_b[
            3]  # z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
        quat[3] = q_a[3] * q_b[3] - q_a[0] * q_b[0] - q_a[1] * q_b[1] - q_a[2] * q_b[
            2]  # w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z
        return quat


def main(args=None):
    rclpy.init(args=args)
    trajectory_publisher = FprTrajectoryPublisher()
    try:
        rclpy.spin(trajectory_publisher)
    except KeyboardInterrupt:
        print('Shutting down FprTrajectory publisher')
    finally:
        trajectory_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
