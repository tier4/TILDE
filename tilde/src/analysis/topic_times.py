#!/usr/bin/python3

import argparse
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message


def get_rosbag_options(path, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    return storage_options, converter_options

def main(args):
    bag_path = args.bag_path
    topic = args.topic

    storage_options, converter_options = get_rosbag_options(bag_path)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    # Create a map for quicker lookup
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    storage_filter = rosbag2_py.StorageFilter(topics=[topic])
    reader.set_filter(storage_filter)

    print("topic: {}".format(topic))
    if_first = True
    cnt = 0

    while reader.has_next() and cnt <= args.cnt:
        (topic, data, t) = reader.read_next()
        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)
        cnt += 1

        if args.types_only:
            print(msg_type)
            continue

        if not hasattr(msg, "header"):
            print("{} does not have header".format(msg_type))
            print(msg)
            continue

        print("{}.{}: frame_id: {} msg_type: {}".format(
            msg.header.stamp.sec,
            msg.header.stamp.nanosec,
            msg.header.frame_id,
            msg_type))

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_path")
    parser.add_argument("topic")
    parser.add_argument("--types-only", action="store_true")
    parser.add_argument("--cnt", type=int, default=10)
    args = parser.parse_args()

    main(args)
