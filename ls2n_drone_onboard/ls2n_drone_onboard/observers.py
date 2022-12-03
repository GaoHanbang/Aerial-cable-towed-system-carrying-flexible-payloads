import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from ls2n_interfaces.msg import *
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import Vector3Stamped
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from ls2n_drone_bridge.px4_com import qos_profile_sensor_data
from rclpy.callback_groups import ReentrantCallbackGroup
import transforms3d as tf3d


class DroneThrustObserver(Node):
    def __init__(self):
        super().__init__('drone_observer')
        # Load parameters
        disturbances_broadcast_param_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_BOOL,
            description="Broadcast the estimation of the disturbances")
        dist_gain_param_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description="Disturbances observer gain")
        thrust_gain_param_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_DOUBLE,
            description="Max thrust observer gain")
        self.declare_parameter('disturbances_gain', 2.0, dist_gain_param_descriptor)
        self.declare_parameter('max_thrust_gain', 0.2, thrust_gain_param_descriptor)
        self.declare_parameter('disturbances_broadcast', False, disturbances_broadcast_param_descriptor)
        self.k = self.get_parameter('disturbances_gain').value
        self.k_thrust = self.get_parameter('max_thrust_gain').value

        self.mass = 1.
        self.max_thrust = 45.
        self.g = np.array([0.0, 0.0, -9.81])

        self.f_ext = np.zeros(3)  # Observed external force (world frame)
        self.max_thrust_est = 0.0  # Observed max thrust
        self.u = 0.  # Drone input
        self.u_cumulated = 0.  # Cumulated drone inputs
        self.time_u_cumulated = 0.0  # Cumulated time of drone input computation

        self.status = DroneStatus.IDLE

        # Observer computation variables
        self.prev_time = 0.0
        self.diff_f_int = np.array([0.0, 0.0, 0.0])
        self.prev_time_u = 0.0
        self.diff_thrust_int = 0.0

        # create a client for requesting drone parameters
        self.drone_param_client = self.create_client(
            GetParameters,
            'drone_bridge/get_parameters'
        )
        # subscription to drone status
        self.status_sub = self.create_subscription(
            DroneStatus,
            'Status',
            self.status_callback,
            qos_profile_sensor_data
        )
        # subscription to odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            'EKF/odom',
            self.odometry_callback,
            qos_profile_sensor_data
        )
        # subscription to drone commands
        self.commanded_control_sub = self.create_subscription(
            AttitudeThrustSetPoint,
            'AttitudeThrustCommanded',
            self.command_callback,
            qos_profile_sensor_data
        )
        # create publisher for the estimated disturbance
        if self.get_parameter('disturbances_broadcast').value:
            observer_ns = "Observer"
        else:
            observer_ns = "ObserverMonitor"
        self.disturbance_pub = self.create_publisher(
            Vector3Stamped,
            observer_ns + '/DisturbancesWorld',
            qos_profile_sensor_data
        )
        # Max thrust observed is only for information purpose
        self.max_thrust_pub = self.create_publisher(
            Float32,
            "ObserverMonitor" + '/MaxThrustObserved',
            qos_profile_sensor_data
        )
        self.get_logger().info("Starting drone observer node")
        while not self.drone_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('DroneParameters service not available, waiting again...')
        self.async_cb_group = ReentrantCallbackGroup()
        self.param_request_timer = self.create_timer(0.1, self.request_parameters, callback_group=self.async_cb_group)

    async def request_parameters(self):
        req = GetParameters.Request()
        req.names = ["mass", "max_thrust"]
        future = self.drone_param_client.call_async(req)
        result = await future
        self.mass = result.values[0].double_value
        self.max_thrust = result.values[1].double_value

    def status_callback(self, msg):
        self.status = msg.status
        if msg.status != DroneStatus.FLYING:
            self.prev_time = 0  # Reset the observers when we are not flying
            self.prev_time_u = 0

    def odometry_callback(self, msg):
        # Update the parameters (if received)
        q = np.array([msg.pose.pose.orientation.w,
                      msg.pose.pose.orientation.x,
                      msg.pose.pose.orientation.y,
                      msg.pose.pose.orientation.z])
        v = np.array([msg.twist.twist.linear.x,
                      msg.twist.twist.linear.y,
                      msg.twist.twist.linear.z])
        curr_time = msg.header.stamp.sec + float(msg.header.stamp.nanosec) / 1.0e9
        # Compute the force observer
        dt = curr_time - self.prev_time
        if self.prev_time != 0.0:
            self.u_cumulated += (curr_time - self.prev_time_u) * self.u
            self.time_u_cumulated += curr_time - self.prev_time_u
            u_mean = self.u_cumulated / self.time_u_cumulated
            # External disturbances estimator (first order) - Assuming max_thrust known
            input_thrust_cumulated = np.array([0.0, 0.0, self.u_cumulated * self.max_thrust])
            self.diff_f_int += self.f_ext * dt + self.mass * self.g * dt \
                               + tf3d.quaternions.quat2mat(q) @ input_thrust_cumulated
            self.f_ext = -self.k * (self.diff_f_int - self.mass * v)
            # max thrust estimator (only for information, not reliable in presence of perturbations)
            # We start from [0.0, 0.0, max_thrust] = m/u*(-dot{R^-1*v} - dot{R}⁻1*v + R⁻1g
            # Then first order estimator. u is considered constant for one step of computation (mean taken)
            inv_rot = tf3d.quaternions.quat2mat(tf3d.quaternions.qinverse(q))
            omega_skew = np.array([[0, -msg.twist.twist.angular.z, msg.twist.twist.angular.y],
                                   [msg.twist.twist.angular.z, 0, -msg.twist.twist.angular.x],
                                   [-msg.twist.twist.angular.y, msg.twist.twist.angular.x, 0]])
            if u_mean != 0.0:
                self.diff_thrust_int += self.max_thrust_est * dt + self.mass / u_mean \
                                        * (-inv_rot @ omega_skew @ v + inv_rot @ self.g)[2] * dt
                self.max_thrust_est = -self.k_thrust * (self.diff_thrust_int - self.mass * (inv_rot @ v)[2])
        else:
            self.diff_f_int = np.array([0.0, 0.0, 0.0])
            self.f_ext = np.array([0.0, 0.0, 0.0])
            self.max_thrust_est = 0.0
            self.diff_thrust_int = 0.0

        if self.status == DroneStatus.FLYING:
            self.publish_estimated_forces()
            self.publish_max_thrust_coeff()

        self.prev_time = curr_time
        self.prev_time_u = curr_time
        self.u_cumulated = 0.0
        self.time_u_cumulated = 0.0

    def command_callback(self, msg):
        curr_time_u = msg.header.stamp.sec + float(msg.header.stamp.nanosec) / 1.0e9
        dt = curr_time_u - self.prev_time_u
        if self.prev_time_u != 0.0:
            self.u_cumulated += self.u * dt
            self.time_u_cumulated += dt
        self.u = msg.thrust
        self.prev_time_u = curr_time_u

    def publish_max_thrust_coeff(self):
        msg = Float32()
        msg.data = self.max_thrust_est
        self.max_thrust_pub.publish(msg)

    def publish_estimated_forces(self):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vector.x = self.f_ext[0]
        msg.vector.y = self.f_ext[1]
        msg.vector.z = self.f_ext[2]
        self.disturbance_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    drone_observer = DroneThrustObserver()
    try:
        rclpy.spin(drone_observer)
    except (KeyboardInterrupt, RuntimeError):
        print('Shutting down drone observer node')
    finally:
        drone_observer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
