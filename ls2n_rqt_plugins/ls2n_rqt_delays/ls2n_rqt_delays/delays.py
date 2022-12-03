from rclpy.node import Node
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding import loadUi
from ament_index_python import get_package_share_directory
from ls2n_interfaces.msg import KeepAlive
import os
from std_msgs.msg import UInt64
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

qos_profile_sensor_data.depth = 1


class DelaysWidget(QWidget):
    def __init__(self, node, plugin):
        super(DelaysWidget, self).__init__()
        self.node = node
        self.plugin = plugin
        self.setObjectName('DelaysWidget')
        ui_file = os.path.join(get_package_share_directory('ls2n_rqt_delays'), 'resource', 'delays.ui')
        loadUi(ui_file, self)


class Delays(Plugin):
    def __init__(self, context):
        super(Delays, self).__init__(context)
        self.widget = DelaysWidget(context.node, self)
        context.add_widget(self.widget)
        self.node: Node = context.node
        self.wifi_delay_counter = 0
        self.wifi_delay = 0
        self.namespace = "CommandCenter"

        # Subscription
        self.node.create_subscription(
            String,
            "/SingleDroneNamespace",
            self.update_namespace,
            1
        )
        self.ka_subs = None
        self.bd_subs = None

    def update_namespace(self, msg):
        if self.namespace != msg.data:
            self.namespace = msg.data
            self.change_namespace(self.namespace)

    def change_namespace(self, namespace):
        if self.ka_subs is not None:
            self.node.destroy_subscription(self.ka_subs)
        if self.bd_subs is not None:
            self.node.destroy_subscription(self.bd_subs)
        self.ka_subs = self.node.create_subscription(KeepAlive,
                                                     namespace + "/KeepAlive",
                                                     self.wifi_delay_callback,
                                                     qos_profile=qos_profile_sensor_data,
                                                     )
        self.bd_subs = self.node.create_subscription(UInt64,
                                                     namespace + "/RTPSBridgeDelay",
                                                     self.rtps_delay_callback,
                                                     qos_profile=qos_profile_sensor_data,
                                                     )

    def rtps_delay_callback(self, msg):
        delay = int(msg.data / 1000)  # Delay in ms
        self.widget.rtps_delay_bar.setValue(delay)

    def wifi_delay_callback(self, msg):
        if msg.feedback:
            time_now = self.node.get_clock().now().to_msg()
            # Delay in ms
            delay = ((time_now.sec - msg.origin_stamp.sec) * 1000 + (
                    time_now.nanosec - msg.origin_stamp.nanosec) * 1e-6) / 2.0
            self.wifi_delay += delay / 10.0
            self.wifi_delay_counter += 1
            if self.wifi_delay_counter >= 10:
                self.wifi_delay = int(self.wifi_delay)
                self.widget.wifi_delay_bar.setValue(self.wifi_delay)
                self.wifi_delay_counter = 0
                self.wifi_delay = 0
