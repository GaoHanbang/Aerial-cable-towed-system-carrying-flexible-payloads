from rclpy.node import Node
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtWidgets import *
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Vector3Stamped
from rclpy.qos import qos_profile_sensor_data
import pyqtgraph as pg

qos_profile_sensor_data.depth = 1


class DOWidget(QWidget):
    def __init__(self, node, plugin):
        super(DOWidget, self).__init__()
        self.node = node
        self.plugin = plugin
        self.setObjectName('DOWidget')
        self.observer_layout = QVBoxLayout()
        self.plot_max_thrust = pg.PlotWidget()
        self.plot_disturbance = pg.PlotWidget()
        self.observer_layout.addWidget(QLabel("Max Thrust"))
        self.observer_layout.addWidget(self.plot_max_thrust)
        self.observer_layout.addWidget(QLabel("Disturbances"))
        self.observer_layout.addWidget(self.plot_disturbance)
        self.setLayout(self.observer_layout)


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


class DroneObservers(Plugin):
    def __init__(self, context):
        super(DroneObservers, self).__init__(context)
        self.widget = DOWidget(context.node, self)
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
        self.dist_subs = None
        self.dist_mon_subs = None
        self.mt_subs = None
        self.mt_mon_subs = None
        self.max_thrust_data = ScopeData(10.0)
        self.disturbances_x_data = ScopeData(10.0)
        self.disturbances_y_data = ScopeData(10.0)
        self.disturbances_z_data = ScopeData(10.0)
        self.gui_interface = {
            "max_thrust_curve": self.widget.plot_max_thrust.getPlotItem().plot(pen=pg.mkPen('g', width=2)),
            "disturbances_curve_x": self.widget.plot_disturbance.getPlotItem().plot(pen=pg.mkPen('g', width=2)),
            "disturbances_curve_y": self.widget.plot_disturbance.getPlotItem().plot(pen=pg.mkPen('r', width=2)),
            "disturbances_curve_z": self.widget.plot_disturbance.getPlotItem().plot(pen=pg.mkPen('w', width=2)),
        }
        self.time_init = self.node.get_clock().now().nanoseconds / 1e9

    def update_namespace(self, msg):
        if self.namespace != msg.data:
            self.namespace = msg.data
            self.change_namespace(self.namespace)

    def change_namespace(self, namespace):
        for subscriber in [self.dist_subs, self.dist_mon_subs, self.mt_subs, self.mt_mon_subs]:
            if subscriber is not None:
                self.node.destroy_subscription(subscriber)
        self.mt_subs = self.node.create_subscription(Float32,
                                                     namespace + "/Observer/MaxThrustObserved",
                                                     self.max_thrust_callback,
                                                     qos_profile=qos_profile_sensor_data
                                                     )
        self.mt_mon_subs = self.node.create_subscription(Float32,
                                                         namespace + "/ObserverMonitor/MaxThrustObserved",
                                                         self.max_thrust_callback,
                                                         qos_profile=qos_profile_sensor_data
                                                         )
        self.dist_subs = self.node.create_subscription(Vector3Stamped,
                                                       namespace + "/Observer/DisturbancesWorld",
                                                       self.disturbances_callback,
                                                       qos_profile=qos_profile_sensor_data
                                                       )
        self.dist_mon_subs = self.node.create_subscription(Vector3Stamped,
                                                           namespace + "/ObserverMonitor/DisturbancesWorld",
                                                           self.disturbances_callback,
                                                           qos_profile=qos_profile_sensor_data
                                                           )

    def max_thrust_callback(self, msg):
        time_data = self.node.get_clock().now().nanoseconds / 1e9 - self.time_init
        self.max_thrust_data.add_data(time_data, msg.data)
        if self.max_thrust_data.must_refresh():
            curve = self.gui_interface["max_thrust_curve"]
            curve.setData(self.max_thrust_data.time, self.max_thrust_data.data)

    def disturbances_callback(self, msg: Vector3Stamped):
        time_data = self.node.get_clock().now().nanoseconds / 1e9 - self.time_init
        self.disturbances_x_data.add_data(time_data, msg.vector.x)
        self.disturbances_y_data.add_data(time_data, msg.vector.y)
        self.disturbances_z_data.add_data(time_data, msg.vector.z)
        if self.disturbances_x_data.must_refresh():
            curve = self.gui_interface["disturbances_curve_x"]
            curve.setData(self.disturbances_x_data.time, self.disturbances_x_data.data)
            curve = self.gui_interface["disturbances_curve_y"]
            curve.setData(self.disturbances_y_data.time, self.disturbances_y_data.data)
            curve = self.gui_interface["disturbances_curve_z"]
            curve.setData(self.disturbances_z_data.time, self.disturbances_z_data.data)
