import asyncio
import qtm
from rclpy.qos import qos_profile_sensor_data
from threading import Thread
import rclpy
from rclpy.node import Node
import xml.etree.ElementTree as ET
from nav_msgs.msg import Odometry
import transforms3d as tf3d
import numpy as np
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
qos_profile_sensor_data.depth = 1


class QualisysPacketReceiver(Thread):
    def __init__(self, ros2_node):
        self.active = True
        Thread.__init__(self)
        self.connection = None
        asyncio.ensure_future(self.setup())
        self.node = ros2_node
        self.index_to_body_name = {}
        self.index_to_body_data = {}
        self.body_count = 0
        try:
            asyncio.get_event_loop().run_forever()
        except KeyboardInterrupt:
            self.active = False

    async def setup(self):
        """ Main function """
        self.connection = await qtm.connect(self.node.ip_address, version=1.13)  # This version works fine, don't change

        if self.connection is None:
            return
        asyncio.ensure_future(self.update_body_refs())
        await self.connection.stream_frames(components=["6d"], on_packet=self.on_packet)

    async def update_body_refs(self):
        while self.active:
            """ Extract a index to name dictionary from 6dof settings xml """
            xml_string = await self.connection.get_parameters(('6d',))
            if xml_string != b'Ok':
                xml = ET.fromstring(xml_string)
                self.index_to_body_name = {}
                for index, body in enumerate(xml.findall("*/Body/Name")):
                    self.index_to_body_name[index] = body.text.strip()
            await asyncio.sleep(0.5)

    def on_packet(self, packet):
        """ Callback function that is called everytime a data packet arrives from QTM """
        _, bodies = packet.get_6d()
        for index, body in enumerate(bodies):
            self.index_to_body_data[index] = body
        self.node.publish_bodies(self.index_to_body_name, self.index_to_body_data)


class QualysisBridge(Node):
    def __init__(self):
        super().__init__('qualysis_bridge')
        self.get_logger().info("Starting qualysis bridge")
        ip_address_param_descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description="Qualisys server IP address")
        self.declare_parameter('ip_address', '0.0.0.0', ip_address_param_descriptor)
        self.ip_address = self.get_parameter('ip_address').value
        # We will store the publishers as dict[body_name] = publisher
        self.publisher_dict = {}

    def publish_bodies(self, body_names, body_data):
        if bool(body_names):
            for index, body in body_names.items():
                # Check if publisher exists otherwise create it
                if body not in self.publisher_dict.keys():
                    self.publisher_dict[body] = self.create_publisher(Odometry,
                                                                      body,
                                                                      qos_profile=qos_profile_sensor_data)
                ori_matrix = np.array(body_data[index][1].matrix)
                pos_vector = np.array([body_data[index][0].x, body_data[index][0].y, body_data[index][0].z])
                if not np.isnan(np.sum(ori_matrix)) and not np.isnan(np.sum(pos_vector)):
                    # Parse data
                    msg = Odometry()
                    msg.pose.pose.position.x = body_data[index][0].x*0.001
                    msg.pose.pose.position.y = body_data[index][0].y*0.001
                    msg.pose.pose.position.z = body_data[index][0].z*0.001
                    q = tf3d.quaternions.qinverse(tf3d.quaternions.mat2quat(ori_matrix))
                    msg.pose.pose.orientation.w = q[0]
                    msg.pose.pose.orientation.x = q[1]
                    msg.pose.pose.orientation.y = q[2]
                    msg.pose.pose.orientation.z = q[3]
                    # Publish
                    self.publisher_dict[body].publish(msg)


def main(args=None):
    # ROS2 node
    rclpy.init(args=args)
    node = QualysisBridge()
    # Qualisys thread
    qual_th = QualisysPacketReceiver(node)
    qual_th.start()
    while qual_th.active:
        rclpy.spin_once(node)
    print('Shutting down trajectory publisher')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == 'main':
    main()
