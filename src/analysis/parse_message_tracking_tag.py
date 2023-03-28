#!/usr/bin/python3

import pickle
import argparse

import rosbag2_py
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

from message_tracking_tag import Message_Tracking_Tag, Message_Tracking_Tags


def get_rosbag_options(path, serialization_format='cdr'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id='sqlite3')

    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format)

    return storage_options, converter_options


def main(args):
    bag_path = args.bag_path

    storage_options, converter_options = get_rosbag_options(bag_path)

    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    # Create a map for quicker lookup
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}

    cnt = 0

    # topic => list of record
    out_per_topic = Message_Tracking_Tags()

    skip_topic_vs_count = {}

    while reader.has_next() and cnt <= args.cnt:
        (topic, data, t) = reader.read_next()
        # TODO: need more accurate check
        if "/message_tracking_tag" not in topic:
            continue

        msg_type = get_message(type_map[topic])
        msg = deserialize_message(data, msg_type)

        out_topic = msg.output_info.topic_name
        out_stamp = msg.output_info.header_stamp

        message_tracking_tag = Message_Tracking_Tag(out_topic, out_stamp)

        for input_info in msg.input_infos:
            in_topic = input_info.topic_name
            in_has_stamp = input_info.has_header_stamp
            if not in_has_stamp:
                if in_topic not in skip_topic_vs_count.keys():
                    print(f"skip {in_topic} as no header.stamp, out_topic = {out_topic}")
                    skip_topic_vs_count[in_topic] = 1
                else:
                    skip_topic_vs_count[in_topic] += 1
                continue
            in_stamp = input_info.header_stamp

            message_tracking_tag.add_input_info(in_topic, in_has_stamp, in_stamp)

        out_per_topic.add(message_tracking_tag)

    pickle.dump(out_per_topic, open("topic_infos.pkl", "wb"), protocol=pickle.HIGHEST_PROTOCOL)

    for topic, count in skip_topic_vs_count.items():
        print(f"skipped {topic} {count} times")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_path")
    parser.add_argument("--cnt", type=int, default=10)
    args = parser.parse_args()

    main(args)
