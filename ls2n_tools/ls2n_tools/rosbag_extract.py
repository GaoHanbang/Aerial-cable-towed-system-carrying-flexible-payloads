import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message


# Extract a rosbag as a python dictionary with topic as keys and (timestamp, msg) array as values
# Classic usage of the output is
# for timestamp, msg in (output["Topic"]):
#   do_something(msg.value1)
def deserialize_rosbag(filename):
    output = {}
    bag = sqlite3.connect(filename)
    for topic in bag.execute("SELECT name FROM topics"):
        output[topic[0]] = []
        type_str = bag.execute(r'SELECT type FROM topics WHERE name is "'+topic[0] + r'"').fetchone()[0]
        msg_type = get_message(type_str)
        topic_id = bag.execute(r'SELECT id FROM topics WHERE name is "'+topic[0] + r'"').fetchone()[0]
        msgs = bag.execute(r'SELECT data, timestamp FROM messages WHERE topic_id is ' + str(topic_id))
        for msg in msgs:
            output[topic[0]].append((msg[1], deserialize_message(msg[0], msg_type)))

    return output
