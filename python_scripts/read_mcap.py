#!/usr/bin/python

import subprocess
subprocess.run("source ~/.bashrc; source /opt/ros/jazzy/setup.bash; sleep 2", shell = True, executable="/bin/bash")


# import subprocess; subprocess.run("python3", shell=True, executable="/bin/bash"); subprocess.run('import subprocess; subprocess.run("python3", shell=True, executable="/bin/bash")', shell=True, executable="/usr/bin/python")
# Run ^ in Python terminal for infinite series of nested bash and python shells.



# read-mcap
#AB example modified from https://github.com/foxglove/mcap/blob/main/python/examples/ros2/py_mcap_demo/py_mcap_demo/reader.py

"""script that reads ROS2 messages from an MCAP bag using the rosbag2_py API."""
# import argparse

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_messages(input_bag: str):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=input_bag, storage_id="mcap"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topic_types = reader.get_all_topics_and_types()

    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(typename(topic))
        msg = deserialize_message(data, msg_type)
        yield topic, msg, timestamp
    del reader


def main():
    # parser = argparse.ArgumentParser(description=__doc__)
    # parser.add_argument(
    #     "input", help="input bag path (folder or filepath) to read from"
    # )

    # args = parser.parse_args()
    path = input("Enter the path to your .mcap file here:")
    for topic, msg, timestamp in read_messages(path):
        print(f"{topic} ({type(msg).__name__}) [{timestamp}]: '{msg.data}'")


if __name__ == "__main__":
    main()