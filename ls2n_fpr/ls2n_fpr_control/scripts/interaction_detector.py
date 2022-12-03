#!/usr/bin/env python3
import numpy as np
import transforms3d as tf3d

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
qos_profile_sensor_data.depth = 1


class InteractionDetector(Node):
    def __init__(self):
        super().__init__('fpr_interaction_detector')
        self.get_logger().info('Detecting interaction board')
        self.interaction_pose_pub = self.create_publisher(
            Pose,
            "/CommandCenter/fpr_interaction_pose",
            qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry,
            "/interaction_board",
            self.update_interaction_pose,
            qos_profile_sensor_data
        )

    def update_interaction_pose(self, msg_in):
        msg_out = Pose()
        msg_out.position = msg_in.pose.pose.position
        orientation_in = msg_in.pose.pose.orientation
        quat = np.array([orientation_in.w, 
                        orientation_in.x, 
                        orientation_in.y, 
                        orientation_in.z])
        eul = tf3d.euler.quat2euler(quat)
        # print("Euler: %.3f, %.3f, %.3f" % (eul[0], eul[1], eul[2]))
        rot_mat = tf3d.euler.euler2mat(eul[0], eul[1], eul[2])
        # print("Rotation matrix: ", rot_mat)
        axis_z = np.array([0, 0, 1])
        rot_zero_yaw = tf3d.axangles.axangle2mat(axis_z, -eul[2])
        # print("Rotation zero yaw: ", rot_zero_yaw)
        rot_mat_out = rot_mat.dot(rot_zero_yaw)
        orientation_out = tf3d.quaternions.mat2quat(rot_mat_out)
        # print("Rotation matrix Out: ", rot_mat_out)
        # print("Quaternion Out: %.3f, %.3f, %.3f, %.3f" % (orientation_out[0], orientation_out[1], orientation_out[2], orientation_out[3]))
        msg_out.orientation.w = orientation_out[0]
        msg_out.orientation.x = orientation_out[1]
        msg_out.orientation.y = orientation_out[2]
        msg_out.orientation.z = orientation_out[3]
        self.interaction_pose_pub.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    detector = InteractionDetector()
    try:
        rclpy.spin(detector)
    except KeyboardInterrupt:
        print("Shutting down fpr interaction detector")
    finally:
        detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
