from rclpy.node import Node
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import *
from std_msgs.msg import String, Float32
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import BatteryStatus
import pyqtgraph as pg
import numpy as np
from scipy.interpolate import CubicSpline
from scipy.stats import linregress

qos_profile_sensor_data.depth = 1


class TEWidget(QWidget):
    def __init__(self, node, plugin):
        super(TEWidget, self).__init__()
        self.node = node
        self.plugin = plugin
        self.setObjectName('TEWidget')
        self.te_layout = QVBoxLayout()
        self.plot_te = pg.PlotWidget()
        self.te_layout.addWidget(self.plot_te)
        self.buttons_layout = QHBoxLayout()
        self.button_start = QPushButton("Start record")
        self.button_stop = QPushButton("Stop record")
        self.button_clear = QPushButton("Clear")
        self.result_label = QLabel("")
        self.buttons_layout.addWidget(self.button_start)
        self.buttons_layout.addWidget(self.button_stop)
        self.buttons_layout.addWidget(self.button_clear)
        self.buttons_layout.addWidget(self.result_label)
        self.button_start.clicked.connect(self.plugin.start_record)
        self.button_stop.clicked.connect(self.plugin.compute_estimation)
        self.button_clear.clicked.connect(self.plugin.clear)
        self.te_layout.addLayout(self.buttons_layout)
        self.setLayout(self.te_layout)


class ScopeData:
    def __init__(self):
        self.time = []
        self.data = []

    def add_data(self, time, data):
        self.time.append(time)
        self.data.append(data)


class ThrustEstimator(Plugin):
    def __init__(self, context):
        super(ThrustEstimator, self).__init__(context)
        self.widget = TEWidget(context.node, self)
        context.add_widget(self.widget)
        self.node: Node = context.node
        self.namespace = "CommandCenter"
        # Namespace subscription
        self.node.create_subscription(
            String,
            "/SingleDroneNamespace",
            self.update_namespace,
            1
        )
        self.mt_mon_subs = None
        self.battery_subs = None
        self.max_thrust_data = ScopeData()
        self.battery_data = ScopeData()
        self.max_thrust_sampled = []
        self.battery_data_sampled = []
        self.data_curve = self.widget.plot_te.getPlotItem().plot(pen=pg.mkPen('g', width=2))
        self.widget.plot_te.getPlotItem().setLabel('bottom', 'Battery voltage', 'V')
        self.widget.plot_te.getPlotItem().setLabel('left', 'Maximum thrust', 'N')
        self.time_init = self.node.get_clock().now().nanoseconds / 1e9
        self.timer = None
        self.record_data = False

    def clear(self):
        self.max_thrust_sampled = []
        self.battery_data_sampled = []

    def update_namespace(self, msg):
        if self.namespace != msg.data:
            self.namespace = msg.data
            self.change_namespace(self.namespace)

    def change_namespace(self, namespace):
        for subscriber in [self.mt_mon_subs, self.battery_subs]:
            if subscriber is not None:
                self.node.destroy_subscription(subscriber)
        self.mt_mon_subs = self.node.create_subscription(Float32,
                                                         namespace + "/ObserverMonitor/MaxThrustObserved",
                                                         self.max_thrust_callback,
                                                         qos_profile=qos_profile_sensor_data
                                                         )
        self.battery_subs = self.node.create_subscription(BatteryStatus,
                                                          self.namespace + "/BatteryStatus_PubSubTopic",
                                                          self.battery_callback,
                                                          qos_profile=qos_profile_sensor_data)

    def max_thrust_callback(self, msg):
        if self.record_data:
            time_data = self.node.get_clock().now().nanoseconds / 1e9 - self.time_init
            self.max_thrust_data.add_data(time_data, msg.data)

    def battery_callback(self, msg: BatteryStatus):
        if self.record_data:
            time_data = self.node.get_clock().now().nanoseconds / 1e9 - self.time_init
            self.battery_data.add_data(time_data, msg.voltage_filtered_v)

    def start_record(self):
        self.timer = self.node.create_timer(5, self.refresh_graph)
        self.max_thrust_data = ScopeData()
        self.battery_data = ScopeData()
        self.record_data = True

    def refresh_graph(self):
        if self.record_data:
            # Resample data and plot
            t_min = max(self.max_thrust_data.time[0], self.battery_data.time[0])
            t_max = min(self.max_thrust_data.time[-1], self.battery_data.time[-1])
            time_sampled = np.arange(t_min, t_max, 0.05)
            max_thrust_cs = CubicSpline(self.max_thrust_data.time, self.max_thrust_data.data)
            battery_cs = CubicSpline(self.battery_data.time, self.battery_data.data)
            self.battery_data_sampled = np.concatenate((self.battery_data_sampled, battery_cs(time_sampled)), axis=0)
            self.max_thrust_sampled = np.concatenate((self.max_thrust_sampled, max_thrust_cs(time_sampled)), axis=0)
            self.data_curve.setData(self.battery_data_sampled, self.max_thrust_sampled)
            self.time_init = self.node.get_clock().now().nanoseconds / 1e9
            self.max_thrust_data = ScopeData()
            self.battery_data = ScopeData()

    def compute_estimation(self):
        # Get the estimation of the thrust from data, plot and displays result
        if self.timer is not None:
            self.node.destroy_timer(self.timer)
        self.record_data = False
        res = linregress(self.battery_data_sampled, self.max_thrust_sampled)
        self.widget.result_label.setText('T =' + str(res.slope) + ' * V + ' + str(res.intercept))

