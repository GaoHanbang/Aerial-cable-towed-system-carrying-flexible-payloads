#!/usr/bin/env python3
import numpy as np
from scipy import signal
from enum import IntEnum

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Float32, Bool
from ls2n_fpr_interfaces.msg import FprTeleop as FprTeleopMsg
from ls2n_drone_command_center.joystick_connect import Joystick
qos_profile_sensor_data.depth = 1

# Input order: [LH, LV, RH, RV, LT, RT, LB, RB, HP, VP, Ab, Bb, Xb, Yb, BK, ST, BIG]

# Link the teleoperated variables to the joystick inputs indexes
class FprTeleopIndexes():
    vx = 3  # rv (right vertical)
    vy = 2  # rh (right horizontal)
    vz = 1  # lv
    v_yaw = 0  # lh
    roll = 8   # hp
    pitch = 9  # vp
    rp_reset = 14  # back button to reset the roll and pitch
    legs_up = 7    # rb
    legs_down = 6  # lb
    mode_select = 13  # Y button
    desired_force = 4   # lt
    start_interaction = 15 # start button

# Create teleoperation constants and limits for the FPR
ANG_INC = 0.5  # angle increment (in degrees)
MIN_LEG = 30  # minimum leg angles (in degrees)
MAX_LEG = 60  # maximum leg angles
PLATFORM_MAX_ANG = 20  # platform maximum roll and pitch angles (in degrees)

# Update rate (Hz)
RATE = 50

# setpoints and limits for leg angles
class FprLegAngles():
    value = np.deg2rad(50)
    velocity = 0.
    max_value = np.deg2rad(MAX_LEG)
    min_value = np.deg2rad(MIN_LEG)
    increment = np.deg2rad(ANG_INC)

# setpoints and limits for platform angles
class FprPlatformAngles():
    roll = np.deg2rad(0)
    pitch = np.deg2rad(0)
    roll_rate = 0.
    pitch_rate = 0.
    max_roll = np.deg2rad(PLATFORM_MAX_ANG)
    max_pitch = np.deg2rad(PLATFORM_MAX_ANG)
    increment = np.deg2rad(ANG_INC)

# Create limits for the velocity control
class FprVelocityLimits():
    fast_v = 0.5
    fast_a = 1
    med_v = 0.25
    med_a = 0.75
    slow_v = 0.15
    slow_a = 0.45
    # z velocity and acceleration limits
    v_z = 0.25  # m/s
    a_z = 1.  # m/s/s
    # xy velocity and acceleration limits
    v_xy = 0.25  # m/s
    a_xy = 1  # m/s/s
    # yaw limits
    v_yaw = 0.5  # rad/s
    a_yaw = 1.  # rad/s/s

# define a class for holding the velocity setpoints
class FprVelocityValues():
    time = -1.
    vx = 0.
    ax = 0.
    vy = 0.
    ay = 0.
    vz = 0.
    az = 0.
    v_yaw = 0.
    a_yaw = 0.
    buffer_length = 100
    ax_buffer = []
    ay_buffer = []
    az_buffer = []
    a_yaw_buffer = []

class FprDesiredForce():
    value = 0.
    limit = 10.

# Transforms Joystick commands
class FprJoystickControl(Joystick):
    def __init__(self):
        Joystick.__init__(self)
        interact_description = ParameterDescriptor(type=ParameterType.PARAMETER_BOOL,
                                              description='Activate Interaction')
        self.declare_parameter("interaction", False, interact_description)
        self.interaction_activated = self.get_parameter("interaction").value
        if self.interaction_activated:
            self.force_pub = self.create_publisher(
                Float32, 
                "fpr_desired_force",
                qos_profile_sensor_data
            )
            self.start_interaction_pub = self.create_publisher(
                Bool,
                "fpr_start_interaction",
                qos_profile_sensor_data
            )
            self.interaction_started = False
            self.pressed_time_button_st = 0. # start button pressed time
            self.previous_data_button_st = 0 # previous data of start button

        # create publisher for teleop msg
        self.teleop_pub = self.create_publisher(
            FprTeleopMsg,
            "fpr_teleop",
            qos_profile_sensor_data
        )

        # filtering coefficients
        b, a = signal.butter(8, 0.3)
        self.butter_a = a
        self.butter_b = b
        self.filter_init = signal.lfilter_zi(self.butter_b, self.butter_a)

        # velocity mode
        class VelocityMode(IntEnum):
            SLOW = 0
            MEDIUM = 1
            FAST = 2

        self.idx = FprTeleopIndexes()
        self.limits = FprVelocityLimits()
        self.velocity_mode = VelocityMode.SLOW
        self.modes = VelocityMode
        self.select_mode_time = -1.

        self.velocity_data = FprVelocityValues()
        self.platform_angles = FprPlatformAngles()
        self.leg_angles = FprLegAngles()
        self.desired_force = FprDesiredForce()

        self.dt = 1./RATE
        self.publisher_timer = self.create_timer(1./RATE, self.update_and_publish)
        return

    def update_and_publish(self):
        self.update_command()
        self.publish_command()
        # If interaction is activated, publish the desired force
        if self.interaction_activated:
            self.start_interaction()
            msg = Float32()
            msg.data = self.desired_force.value
            self.force_pub.publish(msg)

    # Start interaction experiment
    def start_interaction(self):
        t_now = self.get_clock().now().nanoseconds/1e9
        data_button_st = self.inputs[self.idx.start_interaction]       
        if self.previous_data_button_st == data_button_st and data_button_st:
            if self.pressed_time_button_st < 1.:
                self.pressed_time_button_st = t_now
                return
            else:
                if t_now - self.pressed_time_button_st > 1.0: # press 1s to stop the interaction experiment
                    self.pressed_time_button_st = 0.
                    self.interaction_started = False
        else:
            self.pressed_time_button_st = 0.
            if data_button_st:
                if not self.interaction_started:
                    self.interaction_started = True
        self.previous_data_button_st = data_button_st
        msg = Bool()
        msg.data = self.interaction_started
        self.start_interaction_pub.publish(msg)

    # Update joystick commands
    def update_command(self):
        t_now = self.get_clock().now().nanoseconds/1e9
        if self.select_mode_time <= 0:
            self.select_mode_time = t_now
        # set velocity mode
        if(self.inputs[self.idx.mode_select]): 
            if t_now - self.select_mode_time > 0.5:
                if self.velocity_mode == self.modes.SLOW:
                    self.velocity_mode = self.modes.MEDIUM
                elif self.velocity_mode == self.modes.MEDIUM:
                    self.velocity_mode = self.modes.FAST
                else:
                    self.velocity_mode = self.modes.SLOW
                self.select_mode_time = t_now

        # update velocity scale
        if self.velocity_mode == self.modes.FAST:
            self.limits.v_xy = self.limits.fast_v
            self.limits.v_z = self.limits.fast_v
            self.limits.a_xy = self.limits.fast_a
            self.limits.a_z = self.limits.fast_a
        elif self.velocity_mode == self.modes.MEDIUM:
            self.limits.v_xy = self.limits.med_v
            self.limits.v_z = self.limits.med_v
            self.limits.a_xy = self.limits.med_a
            self.limits.a_z = self.limits.med_a
        elif self.velocity_mode == self.modes.SLOW:
            self.limits.v_xy = self.limits.slow_v
            self.limits.v_z = self.limits.slow_v
            self.limits.a_xy = self.limits.slow_a
            self.limits.a_z = self.limits.slow_a

        # prevent large acceleration on first iteration
        if self.velocity_data.time <= 0:
            self.velocity_data.time = self.get_clock().now().nanoseconds/1e9

        t_now = self.get_clock().now().nanoseconds/1e9
        # vx
        vxd = self.inputs[self.idx.vx]*self.limits.v_xy
        vxd, axd = self.processRateCommand(vxd, self.velocity_data.vx,  
                                          -1*self.limits.v_xy, self.limits.v_xy, self.limits.a_xy, 
                                          self.velocity_data.time, t_now)
        self.velocity_data.vx = vxd
        self.velocity_data.ax = axd
        # vy
        vyd = self.inputs[self.idx.vy]*self.limits.v_xy
        vyd, ayd = self.processRateCommand(vyd, self.velocity_data.vy, 
                                           -1*self.limits.v_xy, self.limits.v_xy, self.limits.a_xy, 
                                           self.velocity_data.time, t_now)
        self.velocity_data.vy = vyd
        self.velocity_data.ay = ayd
        # vz
        vzd = self.inputs[self.idx.vz]*self.limits.v_z
        vzd, azd = self.processRateCommand(vzd, self.velocity_data.vz, 
                                           -1*self.limits.v_z, self.limits.v_z, self.limits.a_z, 
                                           self.velocity_data.time, t_now)
        self.velocity_data.vz = vzd
        self.velocity_data.az = azd
        # v_yaw
        v_yawd = self.inputs[self.idx.v_yaw]*self.limits.v_yaw
        v_yawd, a_yawd = self.processRateCommand(v_yawd, self.velocity_data.v_yaw, 
                                                 -1*self.limits.v_yaw, self.limits.v_yaw, self.limits.a_yaw, 
                                                 self.velocity_data.time, t_now)
        self.velocity_data.v_yaw = v_yawd
        self.velocity_data.a_yaw = a_yawd

        # filtering accelerations
        self.velocity_data.ax_buffer.append(self.velocity_data.ax)
        self.velocity_data.ay_buffer.append(self.velocity_data.ay)
        self.velocity_data.az_buffer.append(self.velocity_data.az)
        self.velocity_data.a_yaw_buffer.append(self.velocity_data.a_yaw)
        if len(self.velocity_data.ax_buffer) > self.velocity_data.buffer_length:
            self.velocity_data.ax_buffer.pop(0)
            self.velocity_data.ay_buffer.pop(0)
            self.velocity_data.az_buffer.pop(0)
            self.velocity_data.a_yaw_buffer.pop(0)
        if len(self.velocity_data.ax_buffer) > 20:
            # filter ax
            z, _ = signal.lfilter(self.butter_b, self.butter_a, self.velocity_data.ax_buffer,
                                    zi=self.filter_init*self.velocity_data.ax_buffer[0])
            self.velocity_data.ax = z[-1]
            # filter ay
            z, _ = signal.lfilter(self.butter_b, self.butter_a, self.velocity_data.ay_buffer,
                                    zi=self.filter_init*self.velocity_data.ay_buffer[0])
            self.velocity_data.ay = z[-1]
            # filter az
            z, _ = signal.lfilter(self.butter_b, self.butter_a, self.velocity_data.az_buffer,
                                    zi=self.filter_init*self.velocity_data.az_buffer[0])
            self.velocity_data.az = z[-1]
            # filter a_yaw
            z, _ = signal.lfilter(self.butter_b, self.butter_a, self.velocity_data.a_yaw_buffer,
                                    zi=self.filter_init*self.velocity_data.a_yaw_buffer[0])
            self.velocity_data.a_yaw = z[-1]

        #  set desired roll and pitch
        self.platform_angles.roll = self.platform_angles.roll + self.inputs[self.idx.roll]*self.platform_angles.increment
        self.platform_angles.pitch = self.platform_angles.pitch + self.inputs[self.idx.pitch]*self.platform_angles.increment
        self.platform_angles.roll_rate = self.inputs[self.idx.roll] * self.platform_angles.increment / self.dt
        self.platform_angles.pitch_rate = self.inputs[self.idx.pitch] * self.platform_angles.increment / self.dt
        if(np.abs(self.platform_angles.roll) > self.platform_angles.max_roll):
            self.platform_angles.roll = np.sign(self.platform_angles.roll)*self.platform_angles.max_roll
            self.platform_angles.roll_rate = 0.
        if(np.abs(self.platform_angles.pitch) > self.platform_angles.max_pitch):
            self.platform_angles.pitch = np.sign(self.platform_angles.pitch)*self.platform_angles.max_pitch
            self.platform_angles.pitch_rate = 0.

        # reset roll and pitch angles
        if(self.inputs[self.idx.rp_reset] == 1):
            self.platform_angles.roll = 0.
            self.platform_angles.pitch = 0.
            self.platform_angles.roll_rate = 0.
            self.platform_angles.pitch_rate = 0.
            
        #  set desired leg angles
        self.leg_angles.value = self.leg_angles.value + \
            self.inputs[self.idx.legs_up]*self.leg_angles.increment - \
            self.inputs[self.idx.legs_down]*self.leg_angles.increment
        self.leg_angles.velocity = self.inputs[self.idx.legs_up] * self.leg_angles.increment / self.dt - \
            self.inputs[self.idx.legs_down] * self.leg_angles.increment / self.dt
        if(self.leg_angles.value < self.leg_angles.min_value):
            self.leg_angles.value = self.leg_angles.min_value
            self.leg_angles.velocity = 0.
        elif(self.leg_angles.value > self.leg_angles.max_value):
            self.leg_angles.value = self.leg_angles.max_value
            self.leg_angles.velocity = 0.

        # set desired force value
        self.desired_force.value = self.inputs[self.idx.desired_force]*self.desired_force.limit
        # update current timestamp
        self.velocity_data.time = self.get_clock().now().nanoseconds/1e9

    def publish_command(self):
        msg = FprTeleopMsg()
        mode_msg = ""
        if self.velocity_mode == self.modes.SLOW:
            mode_msg = "Slow"
        elif self.velocity_mode == self.modes.MEDIUM:
            mode_msg = "Medium"
        elif self.velocity_mode == self.modes.FAST:
            mode_msg = "Fast"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.velocity_mode.data = mode_msg
        msg.platform_linear_vel.x = self.velocity_data.vx
        msg.platform_linear_accel.x = self.velocity_data.ax
        msg.platform_linear_vel.y = self.velocity_data.vy
        msg.platform_linear_accel.y = self.velocity_data.ay
        msg.platform_linear_vel.z = self.velocity_data.vz
        msg.platform_linear_accel.z = self.velocity_data.az
        msg.platform_yaw_rate.data = self.velocity_data.v_yaw
        msg.platform_yaw_accel.data = self.velocity_data.a_yaw
        msg.platform_roll.data = self.platform_angles.roll
        msg.platform_roll_rate.data = self.platform_angles.roll_rate
        msg.platform_pitch.data = self.platform_angles.pitch
        msg.platform_pitch_rate.data = self.platform_angles.pitch_rate
        msg.leg_angles[0].data = self.leg_angles.value
        msg.leg_angles[1].data = self.leg_angles.value
        msg.leg_angles[2].data = self.leg_angles.value
        msg.leg_angles_vel[0].data = self.leg_angles.velocity
        msg.leg_angles_vel[1].data = self.leg_angles.velocity
        msg.leg_angles_vel[2].data = self.leg_angles.velocity
        self.teleop_pub.publish(msg)

    def processRateCommand(self, val_in, val_old, val_min, val_max, val_rate_lim, t_old, t_now):
        if (val_in > val_max):
            return val_max, 0.0
        elif (val_in < val_min):
            return val_min, 0.0
        else:
            dx = val_in - val_old
            dt = t_now - t_old
            if (abs(dx/dt) < abs(val_rate_lim)):
                return val_in, dx/dt
            else:
                return val_old + np.sign(dx)*val_rate_lim*dt, np.sign(dx)*val_rate_lim

def main(args=None):
    rclpy.init(args=args)
    fpr_teleop = FprJoystickControl()
    try:
        rclpy.spin(fpr_teleop)
    except KeyboardInterrupt:
        print("Shutting down fpr teleoperation")
    finally:
        fpr_teleop.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
