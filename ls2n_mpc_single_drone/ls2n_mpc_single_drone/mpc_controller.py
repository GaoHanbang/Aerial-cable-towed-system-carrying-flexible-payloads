import os
import pickle
import rclpy
from rclpy.node import Node
import numpy as np
import transforms3d as tf3d
from scipy import signal
from ls2n_interfaces.msg import *
from ls2n_interfaces.srv import *
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from rcl_interfaces.srv import GetParameters
from ls2n_mpc_single_drone.acados_settings import ocp_settings
from ls2n_drone_bridge.common import FullState
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from ament_index_python.packages import get_package_share_directory
from rclpy.callback_groups import ReentrantCallbackGroup

from rclpy.qos import qos_profile_sensor_data
qos_profile_sensor_data.depth = 1


class MPCParameters:
    WeightQ = np.array([    # state weights
        50,     # position weights
        50,
        50,
        10,      # orientation weights
        10,
        10,
        10,
        5,     # linear velocity weights
        5,
        5
    ])
    WeightR = np.array([    # control weights
        1,
        10,
        10,
        10
    ])
    Tf = 0.5                # prediction horizon
    N = 50                  # discretization number
    omega_xy_max = 3.       # maximum angular rate (x,y axis)
    omega_z_max = 1.5       # maximum angular rate (z axis)
    v_max = 3.              # maximum linear velocity


class DroneMPC(Node):
    def __init__(self):
        super().__init__('drone_mpc_controller')
        self.cb_group = ReentrantCallbackGroup()
        # declare ros parameters
        trajfile_description = ParameterDescriptor(type=ParameterType.PARAMETER_STRING,
                                                  description='Trajectory File')
        self.declare_parameter("trajectory_file", None, trajfile_description)
        trajectory_file = self.get_parameter("trajectory_file").value
        if trajectory_file is None:
                exit("Trajectory file is not found")
        filename = os.path.join(get_package_share_directory('ls2n_tools'),
                            'trajectories', trajectory_file + ".traj.pickle")
        
        # create ros service and client
        self.create_service(
            Empty,
            "SpinMotors",
            self.spin_motors
        )
        self.create_service(
            Empty,
            "StartExperiment",
            self.start_experiment
        )
        self.create_service(
            Empty,
            "StopExperiment",
            self.stop_experiment,
            callback_group=self.cb_group
        )
        self.drone_request_client = self.create_client(
            DroneRequest,
            'Request'
        )
        self.drone_param_client = self.create_client(
            GetParameters,
            'drone_bridge/get_parameters'
        )

        # initialize parameters for mpc
        mpc_params = MPCParameters()
        self.mass = 1. # drone mass
        self.max_thrust = 47. # drone maximum thrust
        self.request_params() # request from drone bridge

        # set ocp solver
        x0 = np.array([0., 0., 0., 1., 0., 0., 0., 0., 0., 0.]) # false initial state (real values updated by setting initial constraints afterwards) 
        self.acados_solver = ocp_settings(self.mass, mpc_params.Tf, mpc_params.N, mpc_params.WeightQ, mpc_params.WeightR, x0, 
            self.max_thrust, mpc_params.omega_xy_max, mpc_params.omega_z_max, mpc_params.v_max)
        #self.params = mpc_params

        # time constants for control
        self.Tf = mpc_params.Tf
        self.N = mpc_params.N
        self.delta_t = self.Tf/self.N  # control interval

        # load trajectory
        self.trajectory = None
        self.load_trajectory(filename)

        # trajectory reference for each control loop
        self.traj_ref = np.zeros((self.N, 10)) # position, orientation (quaternion) and velocity references over the prediction horizon

        # drone state
        self.drone_state = FullState()

        # drone status
        self.drone_status = DroneStatus.IDLE

        # drone thrust rates command
        self.drone_command = np.array([9.81*self.mass,0.,0.,0.])

        # create ros publishers and subscribers
        self.create_subscription(
            Odometry,
            'EKF/odom',
            self.odometry_callback,
            qos_profile_sensor_data
        )
        self.create_subscription(
            DroneStatus,
            'Status',
            self.status_callback,
            qos_profile_sensor_data
        )
        self.command_publisher = self.create_publisher(
            RatesThrustSetPoint, 
            'RatesThrustSetPoint',
            qos_profile_sensor_data
        )

        self.start_time = 0.
        self.controller_timer = None
        self.experiment_started = False

    def request_params(self):
        req = GetParameters.Request()
        req.names = ['mass', 'max_thrust']
        while not self.drone_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('DroneParameters service not available, waiting again...')
        future = self.drone_param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        self.mass = result.values[0].double_value
        self.max_thrust = result.values[1].double_value

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
            self.get_logger().info('Starting experiment')
            request_out = DroneRequest.Request()
            request_out.request = DroneRequest.Request.TAKE_OFF
            self.drone_request_client.call_async(request_out)
        else:
            self.get_logger().info("Experiment already started.")
        return response

    async def stop_experiment(self, request, response):
        self.get_logger().info("Stopping experiment")
        request_out = DroneRequest.Request()
        request_out.request = DroneRequest.Request.LAND
        future = self.drone_request_client.call_async(request_out)
        await future
        if self.experiment_started and future.result().success:
            # Reset everything
            self.controller_timer.destroy()
            self.start_time = 0.
            self.experiment_started = False
        return response

    def load_trajectory(self, filename):
        try:
            file = open(filename, 'rb')
        except FileNotFoundError:
            exit('Trajectory file %s does not exist, please generate with ls2n_tools package.', str(filename))
        self.trajectory = pickle.load(file)
        timestamp = self.trajectory['time']
        traj_dt = timestamp[1] - timestamp[0]
        self.end_time = timestamp[-1]
        self.traj_sampling_rate = 1
        if self.delta_t % traj_dt > 1e-7:  # resample the trajectory
            timestamp = np.arange(timestamp[0], timestamp[-1]+self.delta_t/2, self.delta_t)
            traj_length = np.size(timestamp)
            self.trajectory['time'] = timestamp
            variables = self.trajectory.keys()
            for var in variables:
                if var != 'time':
                    self.trajectory[var] = signal.resample(self.trajectory[var], traj_length)
        else:
            self.traj_sampling_rate = int(self.delta_t/traj_dt)

    def update_trajectory(self):
        time_elapsed = (self.get_clock().now() - self.start_time).nanoseconds/1e9
        if self.end_time - time_elapsed >= self.Tf:
            start_index = np.argmax(self.trajectory['time'] >= time_elapsed)
            for i in range(self.N):
                index = i*self.traj_sampling_rate
                self.traj_ref[i,0] = self.trajectory['x'][start_index+index]
                self.traj_ref[i,1] = self.trajectory['y'][start_index+index]
                self.traj_ref[i,2] = self.trajectory['z'][start_index+index]
                yaw = self.trajectory['yaw'][start_index+index]
                quat = tf3d.euler.euler2quat(0, 0, yaw)
                self.traj_ref[i,3] = quat[0]
                self.traj_ref[i,4] = quat[1]
                self.traj_ref[i,5] = quat[2]
                self.traj_ref[i,6] = quat[3]
                self.traj_ref[i,7] = self.trajectory['xD'][start_index+index]
                self.traj_ref[i,8] = self.trajectory['yD'][start_index+index]
                self.traj_ref[i,9] = self.trajectory['zD'][start_index+index]
        else: # reach the final state of trajectory
            for i in range(self.N):
                self.traj_ref[i,0] = self.trajectory['x'][-1]
                self.traj_ref[i,1] = self.trajectory['y'][-1]
                self.traj_ref[i,2] = self.trajectory['z'][-1]
                yaw = self.trajectory['yaw'][-1]
                quat = tf3d.euler.euler2quat(0, 0, yaw)
                self.traj_ref[i,3] = quat[0]
                self.traj_ref[i,4] = quat[1]
                self.traj_ref[i,5] = quat[2]
                self.traj_ref[i,6] = quat[3]
                self.traj_ref[i,7] = self.trajectory['xD'][-1]
                self.traj_ref[i,8] = self.trajectory['yD'][-1]
                self.traj_ref[i,9] = self.trajectory['zD'][-1]

    def spin_controller(self):
        if self.drone_status == DroneStatus.FLYING:
            # update trajectory
            self.update_trajectory()        
            # update current state for each control loop
            curr_state = np.array([
                self.drone_state.position[0],
                self.drone_state.position[1],
                self.drone_state.position[2],
                self.drone_state.orientation[0],
                self.drone_state.orientation[1],
                self.drone_state.orientation[2],
                self.drone_state.orientation[3],
                self.drone_state.velocity[0],
                self.drone_state.velocity[1],
                self.drone_state.velocity[2]
            ])
            self.acados_solver.set(0, "lbx", curr_state)
            self.acados_solver.set(0, "ubx", curr_state)    
            # update reference
            u_ref = np.array([0., 0., 0., 0.])
            for i in range(self.N):
                x_ref = self.traj_ref[i,:].reshape(-1)
                y_ref = np.concatenate((x_ref, u_ref))
                self.acados_solver.set(i, 'yref', y_ref)
                if i == (self.N-1):
                    y_ref_N = x_ref
                    self.acados_solver.set(self.N, 'yref', y_ref_N)
            # solve ocp by acados solver
            acados_status = self.acados_solver.solve()
            if acados_status != 0:
                self.get_logger().warn('Acados solver failed, returned status {}'.format(acados_status))      
            # send the control solution from the solver
            self.drone_command = self.acados_solver.get(0, 'u')
            self.publish_command()

        elif self.drone_status == DroneStatus.EMERGENCY_STOP:
            self.drone_command = np.array([0., 0., 0., 0.])
            self.controller_timer.destroy()
            self.start_time = 0.
            self.experiment_started = False
        else:
            self.get_logger().warn('Drone status not flying, controller failed')
            return

    def publish_command(self):
        control_msg = RatesThrustSetPoint()
        control_msg.thrust = self.drone_command[0]
        control_msg.rates.x = self.drone_command[1]
        control_msg.rates.y = self.drone_command[2]
        control_msg.rates.z = self.drone_command[3]
        self.command_publisher.publish(control_msg)

    def status_callback(self, msg):
        self.drone_status = msg.status
        if msg.status == DroneStatus.FLYING and not self.experiment_started:
            request_out = DroneRequest.Request()
            request_out.request = DroneRequest.Request.RATES_THRUST_CONTROL
            self.drone_request_client.call_async(request_out)
            self.experiment_started = True
            self.get_logger().info("Starting MPC")
            self.start_time = self.get_clock().now()
            self.controller_timer = self.create_timer(self.delta_t, self.spin_controller)

    def odometry_callback(self, msg):
        self.drone_state.position = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])
        self.drone_state.orientation = np.array([
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        ])
        self.drone_state.velocity = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ])


def main(args=None):
    rclpy.init(args=args)
    drone_mpc_controller = DroneMPC()
    try:
        rclpy.spin(drone_mpc_controller)
    except (KeyboardInterrupt, RuntimeError):
        print('Shutting down drone mpc controller')
    finally:
        drone_mpc_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
