from rclpy.node import Node
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import *
from python_qt_binding.QtGui import QDoubleValidator
from px4_msgs.msg import *
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
import pyqtgraph as pg

qos_profile_sensor_data.depth = 1


class TMWidget(QWidget):
    def __init__(self, node, plugin):
        super(TMWidget, self).__init__()
        self.node = node
        self.plugin = plugin
        self.setObjectName('TMWidget')
        self.layout = QVBoxLayout()
        self.settings_layout = QHBoxLayout()
        self.min_mot_set = QLineEdit("1000")
        self.max_mot_set = QLineEdit("2000")
        self.min_mot_set.setValidator(QDoubleValidator(bottom=0, decimals=0))
        self.max_mot_set.setValidator(QDoubleValidator(bottom=0, decimals=0))
        self.settings_layout.addWidget(self.min_mot_set)
        self.settings_layout.addWidget(QLabel("Motor min"))
        self.settings_layout.addWidget(self.max_mot_set)
        self.settings_layout.addWidget(QLabel("Motor max"))
        self.plot = pg.PlotWidget()
        self.layout.addLayout(self.settings_layout)
        self.layout.addWidget(self.plot)
        self.setLayout(self.layout)
        self.min_mot_set.textChanged.connect(self.plugin.update_min_mot)
        self.max_mot_set.textChanged.connect(self.plugin.update_max_mot)


class ScopeData:
    def __init__(self, max_duration):
        self.max_duration = max_duration
        self.time = []
        self.data = []
        self.refresh_graph = False
        self.counter_refresh = 0

    def add_data(self, time, data):
        self.time.append(time)
        self.data.append(data)
        while self.time[-1] - self.time[0] > self.max_duration:
            self.time.pop(0)
            self.data.pop(0)
        self.counter_refresh += 1
        if self.counter_refresh > 50:
            self.refresh_graph = True

    def must_refresh(self):
        if self.refresh_graph:
            self.counter_refresh = 0
            self.refresh_graph = False
            return True
        return False


class ThrustToMotors(Plugin):
    def __init__(self, context):
        super(ThrustToMotors, self).__init__(context)
        self.widget = TMWidget(context.node, self)
        context.add_widget(self.widget)
        self.node: Node = context.node
        self.namespace = "CommandCenter"
        # Subscription
        self.node.create_subscription(
            String,
            "/SingleDroneNamespace",
            self.update_namespace,
            1
        )
        self.thrust_subs = None
        self.motors_subs = None
        self.thrust_data = ScopeData(10.0)
        self.motors_data = ScopeData(10.0)
        self.motor_max = 2000
        self.motor_min = 1000
        self.widget.plot.getPlotItem().addLegend()
        self.plot_interface = {
            "thrust": self.widget.plot.getPlotItem().plot(pen=pg.mkPen('g', width=2), name="Scaled thrust input"),
            "motors": self.widget.plot.getPlotItem().plot(pen=pg.mkPen('r', width=2), name="Scaled motors input"),
            }
        self.time_init = self.node.get_clock().now().nanoseconds / 1e9

    def update_min_mot(self, new_val):
        self.motor_min = int(new_val)

    def update_max_mot(self, new_val):
        self.motor_max = int(new_val)

    def update_namespace(self, msg):
        if self.namespace != msg.data:
            self.namespace = msg.data
            self.change_namespace(self.namespace)

    def change_namespace(self, namespace):
        for subscriber in [self.thrust_subs, self.motors_subs]:
            if subscriber is not None:
                self.node.destroy_subscription(subscriber)
        self.thrust_subs = self.node.create_subscription(VehicleAttitudeSetpoint,
                                                         namespace + "/VehicleAttitudeSetpoint_PubSubTopic",
                                                         self.thrust_update_callback,
                                                         qos_profile=qos_profile_sensor_data
                                                         )
        self.motors_subs = self.node.create_subscription(ActuatorOutputs,
                                                         namespace + "/ActuatorOutputs_PubSubTopic",
                                                         self.motors_update_callback,
                                                         qos_profile=qos_profile_sensor_data
                                                         )

    def thrust_update_callback(self, msg: VehicleAttitudeSetpoint):
        time_data = self.node.get_clock().now().nanoseconds / 1e9 - self.time_init
        self.thrust_data.add_data(time_data, -msg.thrust_body[2])
        if self.thrust_data.must_refresh():
            curve = self.plot_interface["thrust"]
            curve.setData(self.thrust_data.time, self.thrust_data.data)

    def motors_update_callback(self, msg: ActuatorOutputs):
        time_data = self.node.get_clock().now().nanoseconds / 1e9 - self.time_init
        motor_range = self.motor_max - self.motor_min
        motor_sum = sum([((msg.output[i] - self.motor_min)/motor_range)**2 for i in range(4)]) / 4.0
        self.motors_data.add_data(time_data, motor_sum)
        if self.motors_data.must_refresh():
            curve = self.plot_interface["motors"]
            curve.setData(self.motors_data.time, self.motors_data.data)
