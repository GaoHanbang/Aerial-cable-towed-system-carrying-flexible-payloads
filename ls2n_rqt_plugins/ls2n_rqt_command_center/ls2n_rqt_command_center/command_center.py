from rclpy.node import Node
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding import loadUi
from ament_index_python import get_package_share_directory
from std_msgs.msg import String
import os


class CCWidget(QWidget):
    def __init__(self, node, plugin):
        super(CCWidget, self).__init__()
        self.node = node
        self.plugin = plugin
        self.setObjectName('CCWidget')
        ui_file = os.path.join(get_package_share_directory('ls2n_rqt_command_center'), 'resource', 'command_center.ui')
        loadUi(ui_file, self)
        self.refresh_namspace_list()
        self.namespace_select.activated[str].connect(self.plugin.change_namespace)
        self.refresh_button.clicked.connect(self.refresh_namspace_list)

    def refresh_namspace_list(self):
        namespace_list = ["CommandCenter"]
        node_list = self.node.get_node_names_and_namespaces()
        for node, namespace in node_list:
            if "drone_bridge" in node:
                namespace_list.append(namespace)
        self.namespace_select.clear()
        self.namespace_select.addItems(namespace_list)


class CommandCenter(Plugin):
    def __init__(self, context):
        super(CommandCenter, self).__init__(context)
        self.widget = CCWidget(context.node, self)
        context.add_widget(self.widget)
        self.node: Node = context.node
        self.namespace = "CommandCenter"
        self.ns_pub = self.node.create_publisher(
            String,
            "/SingleDroneNamespace",
            1
        )
        self.node.create_timer(0.5, self.publish_namespace)

    def change_namespace(self, namespace):
        self.namespace = namespace

    def publish_namespace(self):
        msg = String()
        msg.data = self.namespace
        self.ns_pub.publish(msg)
