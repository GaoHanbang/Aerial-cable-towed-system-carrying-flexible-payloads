import transforms3d as tf3d
from ls2n_drone_bridge.common import *
from ls2n_interfaces.msg import *
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Vector3Stamped


class Controller:
    def __init__(self, node):
        self.node = node

    type = ControllerType.NONE
    desired_state = FullState()
    desired_scaled_thrust = 0.0
    disturbances = np.zeros(3)

    # Estimation of the lateral disturbances computed by the oserver
    def update_disturbances(self, msg: Vector3Stamped):
        self.disturbances[0] = msg.vector.x
        self.disturbances[1] = msg.vector.y
        self.disturbances[2] = msg.vector.z

    # Set attitude thrust from a force vector
    def set_attitude_thrust_from_vec(self, desired_force):
        # Computes desired orientation from desired force
        # We first build the rotation matrix then convert it to quaternion
        # Define the new frame - z is along the force
        z = desired_force / np.linalg.norm(desired_force)
        # x is uses the current rotation x value - will be corrected to the desired yaw in the end
        rotation = tf3d.quaternions.quat2mat(self.node.odometry.orientation)
        x = rotation[:, 0]
        y = np.cross(z, x)
        y = y / np.linalg.norm(y)
        x = np.cross(y, z)
        x = x / np.linalg.norm(x)
        new_rotation = np.transpose(np.array([x, y, z]))
        # Get the proper yaw
        roll, pitch, yaw = tf3d.euler.mat2euler(new_rotation, axes='sxyz')
        new_rotation = tf3d.euler.euler2mat(roll, pitch, self.desired_state.yaw, axes='sxyz')
        # Export desired thrust and orientation
        scaled_thrust = np.linalg.norm(desired_force) / self.node.max_thrust()
        if scaled_thrust < 0.05:
            scaled_thrust = 0.05
        if scaled_thrust > 1.0:
            scaled_thrust = 1.0
        self.desired_scaled_thrust = scaled_thrust
        self.desired_state.orientation = tf3d.quaternions.mat2quat(new_rotation)
        # Publish the command sent to the PX4 for monitoring/observer
        msg = AttitudeThrustSetPoint()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.thrust = self.desired_scaled_thrust
        msg.attitude.w, msg.attitude.x, msg.attitude.y, msg.attitude.z = self.desired_state.orientation
        self.node.attitude_thrust_publisher.publish(msg)
        # Send input to the PX4
        self.node.com.set_attitude(self.desired_state, self.desired_scaled_thrust)

    def update_setpoints(self, msg: JointTrajectory):
        for index, coordinate in enumerate(msg.joint_names):
            position = 0.0
            velocity = 0.0
            acceleration = 0.0
            if len(msg.points[0].positions) > index:
                position = msg.points[0].positions[index]
            if len(msg.points[0].velocities) > index:
                velocity = msg.points[0].velocities[index]
            if len(msg.points[0].accelerations) > index:
                acceleration = msg.points[0].accelerations[index]
            if coordinate == "x":
                self.desired_state.position[0] = position
                self.desired_state.velocity[0] = velocity
                self.desired_state.acceleration[0] = acceleration
            elif coordinate == "y":
                self.desired_state.position[1] = position
                self.desired_state.velocity[1] = velocity
                self.desired_state.acceleration[1] = acceleration
            elif coordinate == "z":
                self.desired_state.position[2] = position
                self.desired_state.velocity[2] = velocity
                self.desired_state.acceleration[2] = acceleration
            elif coordinate == "yaw":
                self.desired_state.yaw = position
            else:
                self.node.get_logger.warn("Invalid coordinate" + coordinate + " received.")

    # Default do control
    def do_control(self, _):
        pass


# This position controller is build on attitude/thrust. It is not using the internal drone position control
class PositionController(Controller):
    def __init__(self, node):
        super().__init__(node)
        self.pos_err_integral = 0
        self.type = ControllerType.POSITION

    def do_control(self, time_step):
        # Position controller - stupid basic PID TO IMPROVE (windup, etc)
        kp = self.node.kp_position()
        kd = self.node.kd_position()
        ki = self.node.ki_position()
        error = self.desired_state.position - self.node.odometry.position
        error_d = self.desired_state.velocity - self.node.odometry.velocity
        feed_forward = self.desired_state.acceleration
        self.pos_err_integral += error * time_step
        desired_force = (feed_forward + kp * error + kd * error_d + ki * self.pos_err_integral) * self.node.mass()
        desired_force += [0.0, 0.0, self.node.mass() * 9.81]  # Weight compensation
        # compute the corrected desired force under the influence of disturbances
        desired_force -= self.disturbances
        # Computes and set desired orientation from desired force
        self.set_attitude_thrust_from_vec(desired_force)


# A velocity controller, based on the drone onboard controller
class VelocityController(Controller):
    def __init__(self, node):
        super().__init__(node)
        self.type = ControllerType.VELOCITY
        self.vel_err_integral = 0

    def do_control(self, time_step):
        # Velocity controller
        kp = self.node.kp_velocity()
        ki = self.node.ki_velocity()
        error = self.desired_state.velocity - self.node.odometry.velocity
        feed_forward = self.desired_state.acceleration
        self.vel_err_integral += error * time_step
        desired_force = (feed_forward + kp * error + ki * self.vel_err_integral) * self.node.mass()
        desired_force += [0.0, 0.0, self.node.mass() * 9.81]  # Weight compensation
        # Compute the corrected desired force under the influence of disturbances
        desired_force -= self.disturbances
        # Computes and set desired orientation from desired force
        self.set_attitude_thrust_from_vec(desired_force)


class AccelerationController(Controller):
    def __init__(self, node):
        super().__init__(node)
        self.type = ControllerType.ACCELERATION

    def do_control(self, time_step):
        # Computes desired force
        desired_force = self.desired_state.acceleration * self.node.mass()
        # Weight compensation
        desired_force += [0.0, 0.0, self.node.mass() * 9.81]
        # Compute the corrected desired force under the influence of disturbances
        desired_force -= self.disturbances
        # Computes and set desired orientation from desired force
        self.set_attitude_thrust_from_vec(desired_force)


class AttitudeThrustController(Controller):
    def __init__(self, node):
        super().__init__(node)
        self.type = ControllerType.ATTITUDE_THRUST

    def update_attitude_thrust(self, msg: AttitudeThrustSetPoint):
        # desired orientation represented by rotation matrix
        attitude = [msg.attitude.w, msg.attitude.x, msg.attitude.y, msg.attitude.z]
        r_des = tf3d.quaternions.quat2mat(attitude)
        # commanded force in world frame
        f_cmd_0 = r_des.dot(np.array([0.0, 0.0, msg.thrust]))
        # compute the corrected commands
        f_corrected_0 = f_cmd_0 - self.disturbances
        # remember the desired yaw
        _, _, self.desired_state.yaw = tf3d.euler.quat2euler(attitude, axes='sxyz')
        # Set the new inputs
        self.set_attitude_thrust_from_vec(f_corrected_0)


class RatesThrustController(Controller):
    def __init__(self, node):
        super().__init__(node)
        self.type = ControllerType.RATES_THRUST

    def update_rates_thrust(self, msg: RatesThrustSetPoint):
        self.desired_state.angular_velocity = [msg.rates.x, msg.rates.y, msg.rates.z]
        # Scale the thrust in 0 to 1
        fscale = msg.thrust / self.node.max_thrust()
        if fscale < 0.05:
            fscale = 0.05
        if fscale > 1.0:
            fscale = 1.0
        self.desired_scaled_thrust = fscale
        self.node.com.set_rates(self.desired_state, self.desired_scaled_thrust)
