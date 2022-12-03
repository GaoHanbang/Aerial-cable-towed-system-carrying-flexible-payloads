from rclpy.node import Node
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding import loadUi
from ament_index_python import get_package_share_directory
from std_srvs.srv import Empty
from ls2n_interfaces.msg import KeepAlive
from std_msgs.msg import String
import os
from rclpy.qos import qos_profile_sensor_data

qos_profile_sensor_data.depth = 1


class FJWidget(QWidget):
    def __init__(self, node, plugin):
        super(FJWidget, self).__init__()
        self.node = node
        self.plugin = plugin
        self.setObjectName('FJWidget')
        ui_file = os.path.join(get_package_share_directory('ls2n_rqt_fake_joystick'), 'resource', 'fake_joystick.ui')
        loadUi(ui_file, self)
        self.start_experiment_button.clicked.connect(self.plugin.start_experiment)
        self.stop_experiment_button.clicked.connect(self.plugin.stop_experiment)
        self.spin_motors_button.clicked.connect(self.plugin.spin_motors)


class FakeJoystick(Plugin):
    def __init__(self, context):
        super(FakeJoystick, self).__init__(context)
        self.widget = FJWidget(context.node, self)
        context.add_widget(self.widget)
        self.node: Node = context.node

        self.namespace = "CommandCenter"
        # Keep alive
        self.keep_alive_publisher = self.node.create_publisher(KeepAlive, '/CommandCenter/KeepAlive',
                                                               qos_profile_sensor_data)
        self.node.create_timer(0.01, self.keep_alive_callback)
        # Service clients
        self.spin_motors_client = self.node.create_client(Empty, '/CommandCenter/SpinMotors')
        self.start_experiment_client = self.node.create_client(Empty, '/CommandCenter/StartExperiment')
        self.stop_experiment_client = self.node.create_client(Empty, '/CommandCenter/StopExperiment')
        # Subscription
        self.node.create_subscription(
            String,
            "/SingleDroneNamespace",
            self.update_namespace,
            1
        )

        self.keep_alive = self.widget.keep_alive_cb

    def update_namespace(self, msg):
        if self.namespace != msg.data:
            self.namespace = msg.data
            self.change_namespace(self.namespace)

    def change_namespace(self, namespace):
        self.node.destroy_publisher(self.keep_alive_publisher)
        self.keep_alive_publisher = self.node.create_publisher(KeepAlive, namespace + '/KeepAlive',
                                                               qos_profile_sensor_data)
        self.node.destroy_client(self.spin_motors_client)
        self.node.destroy_client(self.stop_experiment_client)
        self.node.destroy_client(self.start_experiment_client)
        self.spin_motors_client = self.node.create_client(Empty, namespace + '/SpinMotors')
        self.start_experiment_client = self.node.create_client(Empty, namespace + '/StartExperiment')
        self.stop_experiment_client = self.node.create_client(Empty, namespace + '/StopExperiment')

    def keep_alive_callback(self):
        if self.keep_alive is not None:
            if self.keep_alive.isChecked():
                msg = KeepAlive()
                msg.keep_alive = True
                msg.feedback = False
                msg.stamp = self.node.get_clock().now().to_msg()
                self.keep_alive_publisher.publish(msg)

    def spin_motors(self):
        self.spin_motors_client.call_async(Empty.Request())

    def stop_experiment(self):
        self.stop_experiment_client.call_async(Empty.Request())

    def start_experiment(self):
        self.start_experiment_client.call_async(Empty.Request())
