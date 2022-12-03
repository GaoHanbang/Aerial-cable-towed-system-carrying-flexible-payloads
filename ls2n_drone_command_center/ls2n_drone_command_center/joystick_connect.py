import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from ls2n_interfaces.msg import KeepAlive
from std_srvs.srv import Empty
from sensor_msgs.msg import Joy as JoyMsg
from std_msgs.msg import Float32

qos_profile_sensor_data.depth = 1

# Default indexes for joystick axes and buttons 
# (Checked with gamepad EasySMX, might need to be verified if using different brands)
# axes:
vx = 3  # rv (right vertical)
vy = 2  # rh (right horizontal)
vz = 1  # lv
v_yaw = 0  # lh
LH = 0  # left horizontal
LV = 1  # left vertical
LT = 2  # left trigger
RH = 3  # right horizontal
RV = 4  # right vertical
RT = 5  # right trigger (reserved for safety button)
HP = 6  # horizontal pad
VP = 7  # vertical pad
# buttons:
BUTTON_A = 0  # reserved for spinning motors
BUTTON_B = 1  # reserved for starting experiment
BUTTON_X = 2
BUTTON_Y = 3
LB = 4  # left trigger button
RB = 5  # right trigger button
BACK = 6  # back button
START = 7  # start button
CENTER = 8  # center big button

# update rate (Hz)
RATE = 50


class Joystick(Node):
    def __init__(self):
        super().__init__('Joystick_Node')
        self.get_logger().info("Starting joystick connection")
        # Subscription to joytick inputs
        self.joy_sub = self.create_subscription(JoyMsg, "/CommandCenter/Joystick",
                                                self.joystick_callback,
                                                qos_profile_sensor_data)
        # Keep alive
        self.keep_alive_publisher = self.create_publisher(KeepAlive, 'KeepAlive',
                                                          qos_profile_sensor_data)
        self.create_timer(1. / RATE, self.keep_alive_callback)
        self.keep_alive = False
        # Service clients
        self.spin_motor_client = self.create_client(Empty, 'SpinMotors')
        self.button_A = 0
        self.start_experiment_client = self.create_client(Empty, 'StartExperiment')
        self.button_B = 0
        self.stop_experiment_client = self.create_client(Empty, 'StopExperiment')
        self.button_X = 0
        # Broadcast of the axis values
        self.lh_pub = self.create_publisher(Float32, 'Joystick/LeftHorizontal', qos_profile_sensor_data)
        self.lv_pub = self.create_publisher(Float32, 'Joystick/LeftVertical', qos_profile_sensor_data)
        self.rh_pub = self.create_publisher(Float32, 'Joystick/RightHorizontal', qos_profile_sensor_data)
        self.rv_pub = self.create_publisher(Float32, 'Joystick/RightVertical', qos_profile_sensor_data)

    def joystick_callback(self, data):
        lh = np.round(1 * data.axes[LH], 3)  # left horizontal
        lv = np.round(data.axes[LV], 3)  # left vertical
        rh = np.round(1 * data.axes[RH], 3)  # right horizontal
        rv = np.round(data.axes[RV], 3)  # right vertical
        lt = np.round((1 - data.axes[LT]) / 2, 3)  # left trigger
        rt = -np.round(data.axes[RT])
        hp = -int(data.axes[HP])  # horizontal pad
        vp = int(data.axes[VP])  # vertical pad
        Ab = data.buttons[BUTTON_A]  # A Button
        Bb = data.buttons[BUTTON_B]  # B Button
        Xb = data.buttons[BUTTON_X]  # X Button
        Yb = data.buttons[BUTTON_Y]  # Y Button
        lb = data.buttons[LB]  # left trigger Button
        rb = data.buttons[RB]  # right trigger Button
        bk = data.buttons[BACK]  # Back Button
        st = data.buttons[START]  # Start Button
        big = data.buttons[CENTER]  # Center big Button

        if rt == 1.:  # KeepAlive
            self.keep_alive = True
        else:
            self.keep_alive = False
        # Buttons are activated on realease
        # button A for spinning motors
        if Ab == 1 and self.button_A == 0:
            self.spin_motor()
        # button B for starting experiment             
        if Bb == 1 and self.button_B == 0:
            self.start_experiment()
        # button B for stopping experiment
        if Xb == 1 and self.button_X == 0:
            self.stop_experiment()
        self.button_A = Ab
        self.button_B = Bb
        self.button_X = Xb

        # Published the normalized axis data
        self.lh_pub.publish(Float32(data=lh))
        self.rh_pub.publish(Float32(data=rh))
        self.lv_pub.publish(Float32(data=lv))
        self.rv_pub.publish(Float32(data=rv))

    def keep_alive_callback(self):
        if self.keep_alive:
            msg = KeepAlive()
            msg.keep_alive = True
            msg.feedback = False
            msg.stamp = self.get_clock().now().to_msg()
            self.keep_alive_publisher.publish(msg)

    def spin_motor(self):
        self.spin_motor_client.call_async(Empty.Request())

    def start_experiment(self):
        self.start_experiment_client.call_async(Empty.Request())

    def stop_experiment(self):
        self.stop_experiment_client.call_async(Empty.Request())


def main(args=None):
    rclpy.init(args=args)
    joystick = Joystick()
    try:
        rclpy.spin(joystick)
    except KeyboardInterrupt:
        print("Shutting down joystick connection")
    joystick.destroy_node()
    rclpy.shutdown()


if __name__ == 'main':
    main()
