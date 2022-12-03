from rclpy.qos import qos_profile_sensor_data
import rclpy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rclpy.node import Node
from rclpy.exceptions import ParameterUninitializedException
from statistics import mean
import os
from collections import deque
from std_msgs.msg import Float64

qos_profile_sensor_data.depth = 1


class MovingAverage(Node):
    def __init__(self):
        super().__init__('moving_average')
        self.get_logger().info("Starting moving average")
        self.declare_parameter('topic', descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING, description="Topic name"
        ))
        self.declare_parameter('topic_variable', descriptor=ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING, description="Topic variable (optional)"
        ))
        self.declare_parameter('sample_number', 1, ParameterDescriptor(
            type=ParameterType.PARAMETER_INTEGER, description="Number of sample"
        ))
        try:
            topic_name = self.get_parameter("topic").value
            self.topic_var = self.get_parameter('topic_variable').value
        except ParameterUninitializedException:
            self.get_logger().error("Please provide a topic name and a topic variable to mean")
            exit(os.EX_USAGE)
        self.sn = self.get_parameter('sample_number').value
        self.buffer = deque(maxlen=self.sn)
        topic_info = []
        self.get_logger().info("Waiting for topic")
        while len(topic_info) == 0:
            topic_info = self.get_publishers_info_by_topic(topic_name)
        self.get_logger().info("Topic found")
        message_package, message_sub_package, message_type = topic_info[0].topic_type.split("/")
        try:
            exec('from ' + message_package + '.' + message_sub_package + ' import ' + message_type)
        except ImportError:
            self.get_logger().error("Message type not found")
            exit(os.EX_DATAERR)
        exec(
            "self.create_subscription(" +
            message_type +
            ",topic_name, self.compute_mean, topic_info[0].qos_profile)"
        )
        self.average_publisher = self.create_publisher(
            Float64,
            topic_name + "_" + self.topic_var + "_average",
            10
        )

    def compute_mean(self, msg):
        try:
            exec("self.msg_data = msg." + self.topic_var)
        except AttributeError:
            self.get_logger().error("Invalid topic variable")
            exit(os.EX_DATAERR)
        try:
            data_float = float(self.msg_data)
        except ValueError:
            self.get_logger().error("Topic value is not a number")
            exit(os.EX_DATAERR)
        self.buffer.append(data_float)
        if len(self.buffer) == self.sn:
            msg_out = Float64()
            msg_out.data = mean(self.buffer)
            self.average_publisher.publish(msg_out)


def main(args=None):
    # ROS2 node
    rclpy.init(args=args)
    node = MovingAverage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('Shutting down moving average')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == 'main':
    main()
