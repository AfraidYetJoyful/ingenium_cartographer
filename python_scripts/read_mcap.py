#!/usr/bin/python

import subprocess
subprocess.run("source ~/.bashrc; source /opt/ros/jazzy/setup.bash; sleep 2", shell = True, executable="/bin/bash")


# import subprocess; subprocess.run("python3", shell=True, executable="/bin/bash"); subprocess.run('import subprocess; subprocess.run("python3", shell=True, executable="/bin/bash")', shell=True, executable="/usr/bin/python")
# Run ^ in Python terminal for infinite series of nested bash and python shells.



# read-mcap
#AB example modified from https://github.com/foxglove/mcap/blob/main/python/examples/ros2/py_mcap_demo/py_mcap_demo/reader.py

"""script that reads ROS2 messages from an MCAP bag using the rosbag2_py API."""
import argparse
import rosbag2_py #AB Each of these import errors resolve themselves once Jazzy is sourced in the same terminal the script is run in.
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import os



# def exportFile(file, exportPath):
#     with open(exportPath, 'a') as outputDocument: # Change the 'x' to 'w' to allow overwrites.
#         outputDocument.write(file)
#         outputDocument.close()

     


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

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "input", help="input bag path (folder or filepath) to read from"
    )

    args = parser.parse_args()

    with open(r"/home/lidar/Documents/Data/rosbag2_2025_07_07-16_16_48/rosbag2_2025_07_07-16_16_48_0_test#2.txt", 'a') as outputDocument: 
        for topic, msg, timestamp in read_messages(args.input):
            # output_string = str(topic) + " " + str(type(msg).__name__) + f"[{timestamp}]: " + str(msg) + "\n"
            outputDocument.write(str(msg) + "\n")
        outputDocument.close()
        


if __name__ == "__main__":
    main()
    print("Done.")



#AB example: /usr/bin/python3 /home/lidar/Documents/Github/ingenium_cartographer/python_scripts/read_mcap.py /home/lidar/Documents/Data/rosbag2_2025_07_07-16_16_48/rosbag2_2025_07_07-16_16_48_0.mcap
#AB: Note to self for morning: working here. VelodyneScan object has no attribute "data", which means you gotta read the timestamps, positions, xs ys and zs separately. Look up the specs for the topics published. 

# 'x', 'y', 'z', 'intensity', 'time', 'column', 'ring', 'return_type'